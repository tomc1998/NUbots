#include "FieldLineDetector.h"

#include <fmt/format.h>
#include <Eigen/Core>

#include "extension/Configuration.h"

// TODO: Replace VisualMesh with GreenHorizon
#include "message/vision/GreenHorizon.h"
#include "message/vision/Line.h"

#include "utility/math/geometry/ConvexHull.h"
#include "utility/math/ransac/Ransac.h"
#include "utility/math/ransac/RansacLineModel.h"
#include "utility/math/ransac/RansacResult.h"
#include "utility/support/eigen_armadillo.h"
#include "utility/vision/visualmesh/VisualMesh.h"

namespace module {
namespace vision {

    using extension::Configuration;

    // TODO: Replace VisualMesh with GreenHorizon
    using message::vision::GreenHorizon;
    using message::vision::Line;
    using message::vision::Lines;

    using utility::math::ransac::Ransac;
    using utility::math::ransac::RansacLineModel;
    using utility::math::ransac::RansacResult;

    static constexpr int LINE_INDEX = 2;

    FieldLineDetector::FieldLineDetector(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("FieldLineDetector.yaml").then([this](const Configuration& cfg) {
            // Use configuration here from file FieldLineDetector.yaml
            config.confidence_threshold       = cfg["confidence_threshold"].as<double>();
            config.cluster_points             = cfg["cocluster_points"].as<int>();
            config.min_points_for_consensus   = cfg["min_points_for_consensus"].as<uint>();
            config.max_iterations_per_fitting = cfg["max_iterations_per_fitting"].as<uint>();
            config.max_fitted_models          = cfg["max_fitted_models"].as<uint>();
            config.consensus_error_threshold  = cfg["consensus_error_threshold"].as<double>();
            config.max_angle_difference       = cfg["max_angle_difference"].as<double>();
            config.max_line_distance          = cfg["max_line_distance"].as<double>();
            config.debug                      = cfg["debug"].as<bool>();
        });

        // TODO: Replace VisualMesh with GreenHorizon
        on<Trigger<GreenHorizon>, Buffer<2>>().then("Field Line Detector", [this](const GreenHorizon& horizon) {
            // Convenience variables
            const auto& cls                                     = horizon.mesh->classifications;
            const auto& neighbours                              = horizon.mesh->neighbourhood;
            const Eigen::Matrix<float, Eigen::Dynamic, 3>& rays = horizon.mesh->rays;
            const float world_offset                            = std::atan2(horizon.Hcw(0, 1), horizon.Hcw(0, 0));

            // Get some indices to partition
            std::vector<int> indices(horizon.mesh->indices.size());
            std::iota(indices.begin(), indices.end(), 0);

            // Partition the indices such that we only have the ball points that dont have ball surrounding them
            auto boundary = utility::vision::visualmesh::partition_points(
                indices.begin(), indices.end(), neighbours, [&](const int& idx) {
                    return idx == indices.size() || (cls(LINE_INDEX, idx) >= config.confidence_threshold);
                });

            // Discard indices that are not on the boundary and are not below the green horizon
            indices.resize(std::distance(indices.begin(), boundary));

            // Cluster all points into line candidates
            // Points are clustered based on their connectivity to other line points
            // Clustering is done in two steps
            // 1) We take the set of line points found above and partition them into potential clusters by
            //    a) Add the first point and its line neighbours to a cluster
            //    b) Find all other line points who are neighbours of the points in the cluster
            //    c) Partition all of the indices that are in the cluster
            //    d) Repeat a-c for all points that were not partitioned
            //    e) Delete all partitions smaller than a given threshold
            // 2) Discard all clusters are entirely above the green horizon
            std::vector<std::vector<int>> clusters;
            utility::vision::visualmesh::cluster_points(
                indices.begin(), indices.end(), neighbours, config.cluster_points, clusters);

            if (config.debug) {
                log<NUClear::DEBUG>(fmt::format("Found {} clusters", clusters.size()));
            }

            auto green_boundary = utility::vision::visualmesh::check_green_horizon_side(
                clusters.begin(), clusters.end(), horizon.horizon.begin(), horizon.horizon.end(), rays, false, true);
            clusters.resize(std::distance(clusters.begin(), green_boundary));

            if (config.debug) {
                log<NUClear::DEBUG>(fmt::format("Found {} clusters below green horizon", clusters.size()));
            }

            std::vector<RansacResult<std::vector<RansacLineModel::DataPoint>::iterator, RansacLineModel>> fitted_models;
            for (const auto& cluster : clusters) {
                // Project 3D unit vectors to 2D points
                std::vector<RansacLineModel::DataPoint> line_points;
                std::transform(cluster.begin(), cluster.end(), line_points.end(), [&](const int& idx) {
                    const Eigen::Vector2f p = utility::math::geometry::project_vector(rays.row(idx));
                    return RansacLineModel::DataPoint{p.x(), p.y()};
                });
                // RANSAC the crap out of the line points
                // TODO: Write a VisualMeshLineModel that works with the indices rather than the raw points
                auto models = Ransac<RansacLineModel>::fitModels(line_points.begin(),
                                                                 line_points.end(),
                                                                 config.min_points_for_consensus,
                                                                 config.max_iterations_per_fitting,
                                                                 config.max_fitted_models,
                                                                 config.consensus_error_threshold);
                fitted_models.insert(fitted_models.end(), models.begin(), models.end());
            }

            // Merge any lines which are approximately colinear
            for (auto it = std::next(fitted_models.begin()); it != fitted_models.end();) {
                const RansacLineModel& l0 = std::prev(it)->model;
                const RansacLineModel& l1 = it->model;

                // Lines are nearly parallel and close to each other, discard the second line
                if ((l0.angleBetween(l1) < config.max_angle_difference)
                    && (std::abs(l0.distance - l1.distance) < config.max_line_distance)) {
                    it = fitted_models.erase(it);
                }
                else {
                    it = std::next(it);
                }
            }

            auto msg = std::make_unique<Lines>();

            for (auto it = fitted_models.begin(); it != fitted_models.end(); it = std::next(it)) {
                // Sort line points in order of increasing x and then increasing y
                std::vector<RansacLineModel::DataPoint> points(it->begin(), it->end());
                std::sort(points.begin(),
                          points.end(),
                          [](const RansacLineModel::DataPoint& a, const RansacLineModel::DataPoint& b) {
                              return (a[0] < b[0]) || ((a[0] == b[0]) && a[1] < b[1]);
                          });

                // Add line to message
                msg->lines.emplace_back(horizon.camera_id,
                                        NUClear::clock::now(),
                                        convert(points.front()).cast<int>(),
                                        convert(points.back()).cast<int>(),
                                        Eigen::Vector4d{1.0, 1.0, 1.0, 1.0});
            }

            emit(std::move(msg));
        });
    }
}  // namespace vision
}  // namespace module
