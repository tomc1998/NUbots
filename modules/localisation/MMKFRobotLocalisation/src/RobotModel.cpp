#include "RobotModel.h"

#include <armadillo>

#include "utility/math/angle.h"
#include "utility/math/matrix.h"
#include "utility/math/coordinates.h"
#include "messages/localisation/FieldObject.h"

using messages::localisation::FakeOdometry;
using utility::math::matrix::rotationMatrix;

namespace modules {
namespace localisation {
namespace robot {

arma::vec::fixed<RobotModel::size> RobotModel::timeUpdate(
    const arma::vec::fixed<RobotModel::size>& state, double deltaT, std::nullptr_t foo) {

    auto result = state;
    return result;
}

arma::vec::fixed<RobotModel::size> RobotModel::timeUpdate(
    const arma::vec::fixed<RobotModel::size>& state, double deltaT, 
    const FakeOdometry& odom) {
    auto result = state;

    // Apply robot odometry / robot position change
    arma::mat22 rot_heading = rotationMatrix(std::atan2(state[kHeadingY], state[kHeadingX]));
    result.rows(kX, kY) += rot_heading * odom.torso_displacement;

    // Rotate heading by torso_rotation.
    arma::mat22 rot = rotationMatrix(odom.torso_rotation);
    result.rows(kHeadingX, kHeadingY) = rot * result.rows(kHeadingX, kHeadingY);

    return result;
}


/// Return the predicted observation of an object at the given position
arma::vec RobotModel::predictedObservation(
    const arma::vec::fixed<RobotModel::size>& state, const arma::vec2& actual_position) {

    // // Radial coordinates
    arma::vec2 diff = actual_position - state.rows(0, 1);
    arma::vec2 radial = utility::math::coordinates::Cartesian2Radial(diff);
    // radial(1) = utility::math::angle::normalizeAngle(radial[1] - state[kHeading]);
    // return radial;

    auto heading_angle = std::atan2(state[kHeadingY], state[kHeadingX]);

    // Distance and unit vector heading
    heading_angle = utility::math::angle::normalizeAngle(radial[1] - heading_angle);

    auto heading_x = std::cos(heading_angle);
    auto heading_y = std::sin(heading_angle);

    // arma::vec2 heading = arma::normalise(diff);
    return {radial[0], heading_x, heading_y};
}

arma::vec RobotModel::predictedObservation(
    const arma::vec::fixed<RobotModel::size>& state, 
    const std::vector<arma::vec2>& actual_positions) {

    // // Radial coordinates
    arma::vec2 diff_1 = actual_positions[0] - state.rows(0, 1);
    arma::vec2 diff_2 = actual_positions[1] - state.rows(0, 1);
    arma::vec2 radial_1 = utility::math::coordinates::Cartesian2Radial(diff_1);
    arma::vec2 radial_2 = utility::math::coordinates::Cartesian2Radial(diff_2);

    auto angle_diff = utility::math::angle::difference(radial_1[1], radial_2[1]);

    return { std::abs(angle_diff) };
}


arma::vec RobotModel::observationDifference(const arma::vec& a, 
                                            const arma::vec& b){
    // // Radial coordinates
    // arma::vec2 result = a - b;
    // // result(1) = utility::math::angle::normalizeAngle(result[1]);
    // result(1) = utility::math::angle::difference(a[1], b[1]);
    // return result;

    // Distance and unit vector heading
    return a - b;
    // arma::vec2 result = a - b;
    // arma::vec2 heading_diff = {result[1], result[2]};
    // arma::vec2 heading = arma::normalise(heading_diff);
    // return {result[0], heading[0], heading[1]};


    // Matrix result;
    // switch(type)
    // {
    //     case MeasurementType::kLandmarkMeasurement:
    //         result = measurement1 - measurement2;
    //         result[1] = utility::math::angle::normalizeAngle(result[1]);
    //         break;
    //     case MeasurementType::kAngleBetweenLandmarksMeasurement:
    //         result = measurement1 - measurement2;
    //         result[0] = utility::math::angle::normalizeAngle(result[0]);
    //         break;
    // };
    // return result;
}


arma::vec::fixed<RobotModel::size> RobotModel::limitState(
    const arma::vec::fixed<RobotModel::size>& state) {
 
    // auto result = state;
    // // // // How to get clipping values from config system?
    // // // result[kX] = std::max(std::min(result[kX], 4.5 + 0.7) , -4.5 -0.7);
    // // // result[kY] = std::max(std::min(result[kY], 3 + 0.7) , -3 -0.7);

    // // // Radial coordinates
    // // result[kHeading] = utility::math::angle::normalizeAngle(result[kHeading]);
    // return result;
    

    // Unit vector orientation
    arma::vec2 heading = { state[kHeadingX], state[kHeadingY] };
    arma::vec2 unit = arma::normalise(heading);
    return { state[0], state[1], unit[0], unit[1] };
}

arma::mat::fixed<RobotModel::size, RobotModel::size> RobotModel::processNoise() {
    return arma::eye(RobotModel::size, RobotModel::size) * processNoiseFactor;
}

}
}
}
