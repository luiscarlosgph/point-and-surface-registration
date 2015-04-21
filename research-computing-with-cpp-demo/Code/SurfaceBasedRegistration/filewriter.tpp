/**
 * @brief Implementation of the header-only template FileWriter.
 *        
 * @author Luis Carlos Garcia-Peraza Herrera (luis.herrera.14@ucl.ac.uk).
 * @date   14 Mar 2015. 
 */

#include <fstream>
#include <streambuf>
#include <boost/filesystem.hpp>
#include <regex>
#include <iostream>
#include <boost/lexical_cast.hpp>
#include <Eigen/Dense>

template <typename Scalar>
FileWriter<Scalar>::FileWriter(const std::string &path) {
	setFilePath(path);	
}

template <typename Scalar>
void FileWriter<Scalar>::setFilePath(const std::string &path) {
	m_path = path;
}

template <typename Scalar>
void FileWriter<Scalar>::saveHomogeneousMatrix(const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> &hom) const {
	return FileWriter::saveHomogeneousMatrix(hom, m_path);
}

template <typename Scalar>
void FileWriter<Scalar>::saveHomogeneousMatrix(const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> &hom, const std::string &path) {
	// Check if file exists
	if (boost::filesystem::exists(path))
		throw std::runtime_error("ERROR! File " + path + " already exists.");

	// Write the homogeneous matrix to the file
	std::ofstream ofs(path);
	if (!ofs.is_open())
		throw std::runtime_error("ERROR! Unable to open " + path + " file.");
	ofs << hom;
}
