#pragma once

#include <math.h>
#include <boost/serialization/version.hpp>
#include "Point.h"

struct RANSACLine {
    Point p1, p2;
    /**
     * Line defined in terms of
     * t1 * x + t2 * y + t3 = 0
     */
    int t1, t2, t3;
    float var;

    RANSACLine(Point p1, Point p2, float var = 0) : p1(p1), p2(p2), var(var) {
        t1 = p2.y() - p1.y();
        t2 = p1.x() - p2.x();
        t3 = p1.y() * (p2.x() - p1.x()) - p1.x() * (p2.y() - p1.y());
    }

    RANSACLine(){};

    RANSACLine(const RANSACLine& other) {
        this->t1  = other.t1;
        this->t2  = other.t2;
        this->t3  = other.t3;
        this->var = other.var;

        this->p1(0, 0) = other.p1(0, 0);
        this->p1(1, 0) = other.p1(1, 0);
        this->p2(0, 0) = other.p2(0, 0);
        this->p2(1, 0) = other.p2(1, 0);
    }


    template <class Archive>
    void serialize(Archive& ar, const unsigned int file_version) {
        ar& t1& t2& t3;
        ar& var;
        if (file_version >= 1) {
            ar& p1& p2;
        }
        else {
            p1 = Point(0, 0);
            p2 = Point(0, 0);
        }
    }
};

inline std::ostream& operator<<(std::ostream& os, const RANSACLine& line) {
    os << line.p1;
    os << line.p2;
    os.write((char*) &(line.t1), sizeof(int));
    os.write((char*) &(line.t2), sizeof(int));
    os.write((char*) &(line.t3), sizeof(int));
    os.write((char*) &(line.var), sizeof(float));

    return os;
}

inline std::istream& operator>>(std::istream& is, RANSACLine& line) {
    is >> line.p1;
    is >> line.p2;
    is.read((char*) &(line.t1), sizeof(int));
    is.read((char*) &(line.t2), sizeof(int));
    is.read((char*) &(line.t3), sizeof(int));
    is.read((char*) &(line.var), sizeof(float));

    return is;
}
