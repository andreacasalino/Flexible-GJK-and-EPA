/**
 * Author:    Andrea Casalino
 * Created:   05.01.2020
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef FLX_SAMPLES_UTILS_H
#define FLX_SAMPLES_UTILS_H

#include <Coordinate.h>
#include <list>
#include <random>
#include <shape/ConvexCloud.h>
#include <shape/TransformDecorator.h>
#include <shape/RoundDecorator.h>

/**
 * @brief Just an example of coordinate representation that can be used 
 * to represent a point cloud
 **/
class Vector {
public:
    Vector(const float& x, const float& y, const float& z) 
        : v({ x, y, z }) { };

    inline float x() const { return this->v.x; };
    inline float y() const { return this->v.y; };
    inline float z() const { return this->v.z; };

    static std::shared_ptr<std::list<Vector>> getRandomCloud(const std::size_t& samplesNumber);

private:
    flx::Coordinate v;
};

#include <GjkEpa.h>
#include <sstream>

class SampleLogger {
public:
    SampleLogger(const std::string& fileName);
    ~SampleLogger();

    inline void addShape(const flx::shape::ConvexShape& shape) { this->shapes.add(getDescribingCloud(shape)); };

    inline void addQueryResult(const flx::GjkEpa::CoordinatePair& result) {
        this->results.add({ Vector(result.pointA.x, result.pointA.y, result.pointA.z)
                          , Vector(result.pointB.x, result.pointB.y, result.pointB.z) });
    };

    void doComplexQuery(flx::GjkEpa& solver, const flx::GjkEpa::ShapePair& pair);

private:
    static std::list<Vector> getDescribingCloud(const flx::shape::ConvexShape& shape);
    inline static std::list<Vector> _getDescribingCloud(const flx::shape::ConvexCloud<std::list<Vector>>& shape) { return shape.getPoints(); };
    static std::list<Vector> _getDescribingCloud(const flx::shape::TransformDecorator& shape);
    static std::list<Vector> _getDescribingCloud(const flx::shape::RoundDecorator& shape);
    
    class VerticesArray {
    public:
        VerticesArray(const std::string& name);

        void add(const std::list<Vector>& element);

        std::string str();

    private:
        bool isFirstElement = true;
        std::stringstream stream;
    };
    mutable VerticesArray shapes;
    mutable VerticesArray results;

    std::string logFile;
#ifdef FLX_LOGGER_ENABLED
    static std::size_t logCounter;
#endif
};

#endif
