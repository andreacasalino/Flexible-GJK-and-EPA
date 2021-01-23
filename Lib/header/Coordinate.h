/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef FLX_COORDINATE_H
#define FLX_COORDINATE_H

#include <math.h>

namespace flx {
	constexpr float GEOMETRIC_TOLLERANCE = static_cast<float>(1e-3);
    
    struct Coordinate {
        float x;
        float y;
        float z;
    };

    constexpr Coordinate ORIGIN {0.f, 0.f, 0.f};

    inline void diff(Coordinate& res, const Coordinate& a, const Coordinate& b) {
        res.x = a.x - b.x;
        res.y = a.y - b.y;
        res.z = a.z - b.z;   
    }

    inline void cross(Coordinate& res, const Coordinate& a, const Coordinate& b) {
        res.x = a.y*b.z - a.z*b.y;
        res.y = a.z*b.x - a.x*b.z;
        res.z = a.x*b.y - a.y*b.x;
    }

    inline Coordinate cross(const Coordinate& a, const Coordinate& b) {
        Coordinate temp;
        cross(temp, a, b);
        return temp;
    }

    inline float dot(const Coordinate& a, const Coordinate& b) {
        float res = a.x*b.x;
        res += a.y*b.y;
        res += a.z*b.z;
        return res;
    }

    inline void prod(Coordinate& V, const float& coeff) {
		V.x *= coeff;
		V.y *= coeff;
		V.z *= coeff;
	}

    inline float normSquared(const Coordinate& c) {
        return dot(c, c);
    }

    inline float norm(const Coordinate& c) {
        return sqrtf(dot(c, c));
    }

    inline void invert(Coordinate& c) {
        c.x = -c.x;
        c.y = -c.y;
        c.z = -c.z;
    }

    inline void normalizeInPlace(Coordinate& c) {
        float coeff = 1.f / norm(c);
        c.x *= coeff;
        c.y *= coeff;
        c.z *= coeff;
    }

    inline float squaredDistance(const Coordinate& a, const Coordinate& b) {
        float res = (a.x - b.x) * (a.x - b.x);
        res += (a.y - b.y) * (a.y - b.y);
        res += (a.z - b.z) * (a.z - b.z);
        return res;
    }
}

#endif