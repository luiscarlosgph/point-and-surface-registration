/**
 * @brief Implementation of the header-only template FileReader.
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

template <typename Scalar>
FileReader<Scalar>::FileReader(const std::string &path) {
	setFilePath(path);	
}

template <typename Scalar>
void FileReader<Scalar>::setFilePath(const std::string &path) {
	m_path = path;
}

template <typename Scalar>
PointCloud<PointXYZ<Scalar> > FileReader<Scalar>::parseInputFile() const {
	return FileReader::parseInputFile(m_path);	
}

template <typename Scalar>
PointCloud<PointXYZ<Scalar> > FileReader<Scalar>::parseInputFile(const std::string &path) {
	const std::string FLOAT = "([+-]?\\d*\\.?\\d+(?:[eE][-+]?\\d+)?)";
	const std::string STP = "[\\s\\t]+";
	const std::string STA = "[\\s\\t]*";
	const std::string EOL = "(?:\\r\\n|\\r|\\n)";
	const std::string LINE(STA + "(?:" + FLOAT + STP + ")*" + FLOAT + STA + EOL);
	const std::regex FLOAT_REGEX(FLOAT);
	const std::regex FILE_REGEX("(?:" + LINE + ")+");

	// Check if file exists
	if (!boost::filesystem::exists(path))
		throw std::runtime_error("ERROR! File " + path + " does not exist.");

	// Read the file
	std::ifstream ifs(path);
	if (!ifs.is_open())
		throw std::runtime_error("ERROR! Unable to open file.");
	std::stringstream buffer;
	buffer << ifs.rdbuf();
	std::string str = buffer.str();

	// Parse the file
	std::smatch sm;
	PointCloud<PointXYZ<Scalar> > retval;
	if (std::regex_match(str, sm, FILE_REGEX)) {
		const std::sregex_token_iterator end;
		PointXYZ<Scalar> point;
		std::sregex_token_iterator i(str.cbegin(), str.cend(), FLOAT_REGEX);
		
		int j = 0;
		while (i != end) {
			point[j] = boost::lexical_cast<Scalar>(*i++);
			if (j == FileReader::POINT_DIM - 1)
				retval.push_back(point);
			j = (j + 1) % FileReader::POINT_DIM;
		}

		// Sanity check, the number of points has to be multiple of the dimension
		if (j != 0)
			throw std::runtime_error("ERROR! The file " + path + " contains an invalid format.");
	}
	else {
		// std::cerr << "File NOT matched!\n";
		throw std::runtime_error("ERROR! The file " + path + " contains an invalid format.");
	}

	return retval;
}
