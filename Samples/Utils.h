/**
 * Author:    Andrea Casalino
 * Created:   05.01.2020
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef FLX_UTILS_H
#define FLX_UTILS_H

#include <Coordinate.h>
#include <list>
#include <stdlib.h>  // for drand48_r
#include <memory>

/**
 * @brief Just an example of coordinate representation that can be used 
 * to represent a shape
 **/
class Vector {
public:
    Vector(const float& x, const float& y, const float& z) {
        this->v = {x, y, z};
    };

    inline float x() const { return this->v.x; };
    inline float y() const { return this->v.y; };
    inline float z() const { return this->v.z; };

    inline static std::shared_ptr<std::list<Vector>> getRandomCloud(const std::size_t& samplesNumber) {
        drand48_data data;
        double sample[3];
        std::shared_ptr<std::list<Vector>> samples = std::make_shared<std::list<Vector>>();
        for(std::size_t k=0; k<samplesNumber; ++k) {
            drand48_r(&data, &sample[0]);
            drand48_r(&data, &sample[1]);
            drand48_r(&data, &sample[2]);
            samples->emplace_back(static_cast<float>(sample[0]), static_cast<float>(sample[1]), static_cast<float>(sample[2]));
        }
        return samples;
    };

private:
    flx::Coordinate v;
};

#include <GjkEpa.h>
#include <iostream>
#include <fstream>

void doComplexQuery(flx::GjkEpa& solver, const flx::GjkEpa::ShapePair& pair, flx::GjkEpa::CoordinatePair& result) {
    auto res = solver.doComplexQuery(pair, result);
    if(flx::GjkEpa::closestPoint == res) {
        std::cout << "collision absent, closest points" << std::endl;
    }
    else {
        std::cout << "collision present, penetration depth" << std::endl;
    }
    std::cout << "<" << result.pointA.x << "," << result.pointA.y << "," << result.pointA.z << ">" << std::endl;;
    std::cout << "<" << result.pointB.x << "," << result.pointB.y << "," << result.pointB.z << ">" << std::endl;;
    std::cout << std::endl;
}

void doComplexQuery(flx::GjkEpa& solver, const flx::GjkEpa::ShapePair& pair, flx::GjkEpa::CoordinatePair& result, const std::string& fileName) {
    doComplexQuery(solver, pair, result);
    std::fstream f(fileName);
    if(!f.is_open()) return;
    
    auto printVertices = [&f](const std::list<Vector>& collection) {
        f << "{\"V\":[";
        auto it= collection.begin();
        f << std::endl << '[' << it->x() << ","<< it->y() << ","<< it->z() << ']';
        f << std::endl << ']}';
    };

    std::list<Vector> temp;
    auto getCloud = [&temp](const shape::ConvexShape* shp){
        const shape::TransformDecorator* trsf = dynamic_cast<const shape::TransformDecorator*>(shp);
        if(nullptr == trsf) {
            temp = dynamic_cast<const shape::ConvexCloud*>(shp)->getPoints();
            return;
        }
        // todo
    };

    f << '{' << std::endl;
    f << "\"Politopes\":[";
    getCloud(&pair.shapeA);
    printVertices(temp);
    f << ",";
    getCloud(&pair.shapeB);
    printVertices(temp);
    f << std::endl << ']';

    f << "\"Lines\":";
    temp = {result.pointA, result.pointB};
    printVertices(temp);
    f << '}';
};

#endif
