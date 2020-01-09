/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
*
* report any bug to andrecasa91@gmail.com.
 **/

#pragma once
#ifndef __LOGRES_H__
#define __LOGRES_H__

#include "Vector3d_basic.h"
#include <sstream>
#include <fstream>
#include <string>

 /** \brief Produces a JSON containg the results of certain proximity queries computations.
 */
class Logger {
public:
	Logger() {};

	void Add_politope(const std::list<v3>& vertices) { this->Politopes.push_back(vertices); };
	void Add_line(const v3& A, const v3& B) { this->Lines.push_back({ v3(A),v3(B) }); };
	/** \brief Removes all the data previously added.*/
	void Clear() { this->Politopes.clear(); this->Lines.clear(); };

	/** \brief Exports the JSON in the specified file (a relative or either an absolute path is accepted) */
	void Write_JSON(const std::string& target_file);
private:
	static void __add_to_Log( std::ofstream& stream,  const std::list<v3>& P);
	template <typename T>
	static std::string to_string_with_precision(const T& a_value, const int n = 3) {
		std::ostringstream out;
		out.precision(n);
		out << std::fixed << a_value;
		return out.str();
	}
// data
	std::list<std::list<v3>> Politopes;
	std::list<std::list<v3>> Lines;
};


void Logger::Write_JSON(const std::string& target_file) {

	std::ofstream f(target_file);
	if (!f.is_open())
		throw 0; //bad path for target_file

	size_t k, K;

	f << "{\n";

	f << "\"Politopes\":[\n";
	K = this->Politopes.size();
	auto it = this->Politopes.begin();
	f << "{\"V\":\n";
	__add_to_Log(f, *it);
	f << "}\n";
	it++;
	for (k = 1; k < K; k++) {
		f << ",{\"V\":\n";
		__add_to_Log(f, *it);
		f << "}\n";
		it++;
	}
	f << "]\n";

	f << ",\"Lines\":[\n";
	K = this->Lines.size();
	auto it2 = this->Lines.begin();
	f << "{\"V\":\n";
	__add_to_Log(f, *it2);
	f << "}\n";
	it2++;
	for (k = 1; k < K; k++) {
		f << ",{\"V\":\n";
		__add_to_Log(f, *it2);
		f << "}\n";
		it2++;
	}
	f << "]\n";

	f << "}";
	f.close();

}

void Logger::__add_to_Log(std::ofstream& stream, const std::list<v3>& P) {

	stream << "[\n";
	size_t K = P.size();
	auto it = P.begin();
	for (size_t k = 1; k < K; k++) {
		stream << "[" << to_string_with_precision(it->_x) << "," << to_string_with_precision(it->_y) << "," << to_string_with_precision(it->_z) << "],\n";
		it++;
	}
	stream << "[" << to_string_with_precision(it->_x) << "," << to_string_with_precision(it->_y) << "," << to_string_with_precision(it->_z)  << "]\n";
	stream << "]\n";

}

#endif