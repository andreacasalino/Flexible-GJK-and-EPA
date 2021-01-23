/**
 * Author:    Andrea Casalino
 * Created:   06.01.2020
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef FLX_ERROR_H
#define FLX_ERROR_H

#include <stdexcept>

namespace flx {
    /** @brief A runtime error that can be raised when using any object in flx::
	 */
    class Error : public std::runtime_error {
    public:
        explicit Error(const std::string& what) : std::runtime_error(what) {
        };
    };
}

#endif