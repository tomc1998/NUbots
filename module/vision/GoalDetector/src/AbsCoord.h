#include <math.h>
#include <eigen3/Eigen/Core>
#include "GoalMatcherConstants.h"
#include "message/localisation/Field.h"

struct AbsCoord {

    AbsCoord(float x, float y, float theta) : vec(x, y, theta) {
        var.setZero();
        var(0, 0) = sqrt(double(FULL_FIELD_LENGTH));
        var(1, 1) = sqrt(double(FULL_FIELD_WIDTH));
        var(2, 2) = sqrt(M_PI);
        weight = 1.0;
    }

    AbsCoord() : vec(0, 0, 0) {
        var.setZero();
        var(0, 0) = sqrt(double(FULL_FIELD_LENGTH));
        var(1, 1) = sqrt(double(FULL_FIELD_WIDTH));
        var(2, 2) = sqrt(M_PI);
        weight = 1.0;
    }

    Eigen::Vector3f vec;
    Eigen::Matrix3f var;
    float weight;

    const float x() const {
        return vec[0];
    }

    float& x() {
        return vec[0];
    }

    const float y() const {
        return vec[1];
    }

    float& y() {
        return vec[1];
    }

    const float theta() const {
        return vec[2];
    }

    float& theta() {
        return vec[2];
    }

    float getVar(int m, int n) const {
        return var(m, n);
    }

    bool operator==(const AbsCoord& other) const {
        return vec == other.vec;
    }

    template <class Archive>
    void serialize(Archive& ar, const unsigned int file_version) {
        ar& vec;
        ar& var;
    }
};
