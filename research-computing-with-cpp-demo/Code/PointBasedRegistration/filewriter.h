/**
 * @class FileWriter writes a homogeneous transformation matrix to file. 
 *        NOTE: this is a header only template.
 *
 * @author Luis Carlos Garcia-Peraza Herrera (luis.herrera.14@ucl.ac.uk).
 * @date   14 Mar 2015. 
 */

#ifndef __FILE_WRITER_H__
#define __FILE_WRITER_H__

#include <Eigen/Dense>

// My includes 
#include "pointxyz.h"
#include "pointcloud.h"

template <typename Scalar = float>
class FileWriter {

public:
	FileWriter(const std::string &path);
	
	void setFilePath(const std::string &path);

	void saveHomogeneousMatrix(const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> &hom) const;
	static void saveHomogeneousMatrix(const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> &hom, const std::string &path);

private:
	std::string m_path;
};

#include "filewriter.tpp"

#endif
