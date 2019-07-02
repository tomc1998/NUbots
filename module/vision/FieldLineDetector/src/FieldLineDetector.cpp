#include "FieldLineDetector.h"

#include <fmt/format.h>
#include <Eigen/Core>

#include "extension/Configuration.h"

// TODO: Replace VisualMesh with GreenHorizon
#include "message/vision/FieldLine.h"
#include "message/vision/GreenHorizon.h"

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
    using message::vision::FieldLine;
    using message::vision::FieldLines;
    using message::vision::GreenHorizon;

    using utility::math::ransac::Ransac;
    using utility::math::ransac::RansacLineModel;
    using utility::math::ransac::RansacResult;

    static constexpr int LINE_INDEX = 2;

    FieldLineDetector::FieldLineDetector(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("FieldLineDetector.yaml").then([this](const Configuration& cfg) {
            // Use configuration here from file FieldLineDetector.yaml
            config.confidence_threshold = cfg["confidence_threshold"].as<float>();
            config.covariance           = cfg["covariance"].as<float>();
            config.debug                = cfg["debug"].as<bool>();
        });

        // TODO: Replace VisualMesh with GreenHorizon
        on<Trigger<GreenHorizon>, Buffer<2>>().then("Field Line Detector", [this](const GreenHorizon& horizon) {
            // Convenience variables
            const auto& cls                                     = horizon.mesh->classifications;
            const auto& neighbours                              = horizon.mesh->neighbourhood;
            const Eigen::Matrix<float, Eigen::Dynamic, 3>& rays = horizon.mesh->rays;

            // Get some indices to partition
            std::vector<int> indices(horizon.mesh->indices.size());
            std::iota(indices.begin(), indices.end(), 0);

            // Partition the indices such that we only have the ball points that dont have ball surrounding them
            auto boundary = utility::vision::visualmesh::partition_points(
                indices.begin(), indices.end(), neighbours, [&](const int& idx) {
                    return idx == indices.size() || (cls(LINE_INDEX, idx) >= config.confidence_threshold);
                });

            if (config.debug) {
                log<NUClear::DEBUG>(
                    fmt::format("Found {} field line points", std::distance(indices.begin(), boundary)));
            }

            auto msg        = std::make_unique<FieldLines>();
            msg->camera_id  = horizon.camera_id;
            msg->timestamp  = horizon.timestamp;
            msg->Hcw        = horizon.Hcw;
            msg->covariance = Eigen::MatrixXf::Identity(std::distance(indices.begin(), boundary),
                                                        std::distance(indices.begin(), boundary))
                              * config.covariance;
            for (auto it = indices.begin(); it != boundary; it = std::next(it)) {
                FieldLine point;
                point.point = horizon.Hcw.topLeftCorner<3, 3>().cast<float>() * rays.row(*it).transpose();
                msg->points.emplace_back(point);
            }

            emit(std::move(msg));
        });
    }
}  // namespace vision
}  // namespace module
