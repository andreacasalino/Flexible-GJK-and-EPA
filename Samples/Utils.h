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
#include <random>
#include <shape/TransformDecorator.h>
#include <shape/ConvexCloud.h>

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
        float sample[3];
        std::shared_ptr<std::list<Vector>> samples = std::make_shared<std::list<Vector>>();
        for(std::size_t k=0; k<samplesNumber; ++k) {
            sample[0] = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
            sample[1] = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
            sample[2] = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
            samples->emplace_back(sample[0], sample[1], sample[2]);
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
    auto getCloud = [&temp](const flx::shape::ConvexShape* shp){
        const flx::shape::TransformDecorator* trsf = dynamic_cast<const flx::shape::TransformDecorator*>(shp);
        if(nullptr == trsf) {
            temp = dynamic_cast<const flx::shape::ConvexCloud<std::list<Vector>>*>(shp)->getPoints();
        }
        else {
            temp = dynamic_cast<const flx::shape::ConvexCloud<std::list<Vector>>*>(&trsf->getShape())->getPoints();
            flx::Coordinate c;
            for (auto it = temp.begin(); it != temp.end(); ++it) {
                c = {it->x(), it->y(), it->z()};
                trsf->transform(c);
                *it = Vector(c.x, c.y, c.z);
            }
        }    
    };

    f << '{' << std::endl;
    f << "\"Politopes\":[";
    getCloud(&pair.shapeA);
    printVertices(temp);
    f << ",";
    getCloud(&pair.shapeB);
    printVertices(temp);
    f << std::endl << ']';

    f << ",\"Lines\":";
    temp = { Vector(result.pointA.x, result.pointA.y, result.pointA.z), 
             Vector(result.pointB.x, result.pointB.y, result.pointB.z) };
    printVertices(temp);
    f << '}';
};

#endif
