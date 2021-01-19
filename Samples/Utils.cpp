/**
 * Author:    Andrea Casalino
 * Created:   05.01.2020
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "Utils.h"
#include <fstream>
#include <Error.h>
#include <math.h>
#include <iostream>

std::shared_ptr<std::list<Vector>> Vector::getRandomCloud(const std::size_t& samplesNumber) {
    std::shared_ptr<std::list<Vector>> samples = std::make_shared<std::list<Vector>>();
    for (std::size_t k = 0; k < samplesNumber; ++k) {
        samples->emplace_back(static_cast<float>(rand()) / static_cast<float>(RAND_MAX), 
                              static_cast<float>(rand()) / static_cast<float>(RAND_MAX), 
                              static_cast<float>(rand()) / static_cast<float>(RAND_MAX));
    }
    return samples;
};

SampleLogger::SampleLogger(const std::string& fileName)
    : shapes("Politopes")
    , results("Lines")
    , logFile(fileName) {
}

SampleLogger::~SampleLogger() {
    std::ofstream f(this->logFile);
    if (!f.is_open()) {
        std::cout << this->logFile << " it is an invalid log file location" << std::endl;
        return;
    }
    f << '{' << std::endl;
    f << this->shapes.str() << std::endl;
    f << ',' << this->results.str() << std::endl;
    f << '}';
}

SampleLogger::VerticesArray::VerticesArray(const std::string& name) {
    this->stream << '\"' << name << "\":[";
}

void SampleLogger::VerticesArray::add(const std::list<Vector>& element) {
    if (this->isFirstElement) {
        this->isFirstElement = false;
    }
    else {
        this->stream << ',';
    }
    this->stream << "{\"V\":[" << std::endl;
    if (!element.empty()) {
        auto it = element.begin();
        this->stream << '[' << it->x() << "," << it->y() << "," << it->z() << ']' << std::endl;
        ++it;
        for (it; it != element.end(); ++it) {
            this->stream << ",[" << it->x() << "," << it->y() << "," << it->z() << ']' << std::endl;
        }
    }
    this->stream << "]}" << std::endl;
}

std::string SampleLogger::VerticesArray::str() {
    this->stream << "]";
    return this->stream.str();
}

std::list<Vector> SampleLogger::getDescribingCloud(const flx::shape::ConvexShape& shape) {
    const flx::shape::ConvexCloud<std::list<Vector>>* ptr1 = dynamic_cast<const flx::shape::ConvexCloud<std::list<Vector>>*>(&shape);
    if (nullptr != ptr1) {
        return _getDescribingCloud(*ptr1);
    }

    const flx::shape::TransformDecorator* ptr2 = dynamic_cast<const flx::shape::TransformDecorator*>(&shape);
    if (nullptr != ptr2) {
        return _getDescribingCloud(*ptr2);
    }

    const flx::shape::RoundDecorator* ptr3 = dynamic_cast<const flx::shape::RoundDecorator*>(&shape);
    if (nullptr != ptr3) {
        return _getDescribingCloud(*ptr3);
    }

    throw flx::Error("Unsupported shape");
}

std::list<Vector> SampleLogger::_getDescribingCloud(const flx::shape::TransformDecorator& shape) {
    auto temp = getDescribingCloud(shape.getShape());
    flx::Coordinate c;
    for (auto it = temp.begin(); it != temp.end(); ++it) {
        c = {it->x(), it->y(), it->z()};
        shape.transform(c);
        *it = Vector(c.x, c.y, c.z);
    }
    return temp;
}

class Interval {
public:
    Interval(const float& min, const float& max, const std::size_t& nPoints) 
        : delta( (max - min) / static_cast<float>(nPoints - 1) )
        , min(min) {
        this->reset();
    };

    inline void reset() { this->value = this->min; };

    inline Interval& operator++() {
        this->value += this->delta;
        return *this;
    };

    inline const float& eval() const { return this->value; };

private:
    float value;
    const float delta;
    const float min;
};
constexpr float PI = 3.14159f;
constexpr std::uint8_t N_ALFA = 5;
constexpr std::uint8_t N_BETA = 10;
std::list<Vector> SampleLogger::_getDescribingCloud(const flx::shape::RoundDecorator& shape) {
    auto temp = getDescribingCloud(shape.getShape());

    std::size_t dim = temp.size();
    auto it = temp.begin();
    std::uint8_t alfa, beta;
    Interval alfaInterval(-0.5f * PI, 0.5f * PI, N_ALFA);
    Interval betaInterval(0.f, 2.f * PI, N_BETA);
    float Calfa, Salfa, Cbeta, Sbeta;
    float coor[3];
    for (std::size_t d = 0; d < dim; ++d) {
        for (beta = 0; beta < N_BETA; ++beta) {
            alfaInterval.reset();
            Cbeta = std::cosf(betaInterval.eval());
            Sbeta = std::sinf(betaInterval.eval());
            for (alfa = 0; alfa < N_ALFA; ++alfa) {
                Calfa = std::cosf(alfaInterval.eval());
                Salfa = std::sinf(alfaInterval.eval());
                coor[0] = Calfa * Cbeta * shape.getRay() + it->x();
                coor[1] = Calfa * Sbeta * shape.getRay() + it->y();
                coor[2] = Salfa * shape.getRay() + it->z();
                temp.emplace_back(coor[0], coor[1], coor[2]);
                ++betaInterval;
            }
            ++alfaInterval;
        }
        ++it;
    }

    return temp;
}

void SampleLogger::doComplexQuery(flx::GjkEpa& solver, const flx::GjkEpa::ShapePair& pair) {
    flx::GjkEpa::CoordinatePair result;
    flx::GjkEpa::ResultType res = solver.doComplexQuery(pair, result
#ifdef FLX_LOGGER_ENABLED
        , std::string{ "SolverLog" + std::to_string(++this->logCounter) }
#endif
    );

    this->addShape(pair.shapeA);
    this->addShape(pair.shapeB);
    this->addQueryResult(result);

    if (flx::GjkEpa::ResultType::closestPoint == res) {
        std::cout << "collision absent, closest points" << std::endl;
    }
    else {
        std::cout << "collision present, penetration depth" << std::endl;
    }
    std::cout << "<" << result.pointA.x << "," << result.pointA.y << "," << result.pointA.z << ">" << std::endl;
    std::cout << "<" << result.pointB.x << "," << result.pointB.y << "," << result.pointB.z << ">" << std::endl;
    std::cout << std::endl;
}
