/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "Logger.h"
#include <fstream>

namespace flx {
    Logger::~Logger() {
        std::ofstream f(this->fileName);
        if(!f.is_open()) {
            return;
        }
        f << '{';
        f << this->stream.str();
        f << std::endl << '}';
    }

    void Array::add(const std::string& element) {
        this->stream << std::endl;
        if(this->isFirst) this->isFirst = false;
        else this->stream << ',';
        this->stream << element;
    }

    std::string Array::str() const {
        std::stringstream ss;
        ss << std::endl << '[';
        ss << this->stream.str();
        ss << std::endl << ']';
        return ss.str();
    }
}