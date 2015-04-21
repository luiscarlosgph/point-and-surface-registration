/**
 * @class FileReader reads files with sets of 3D points and stores them in memory so that other
 *        parts of the program can obtain them. It also handles all the errors and exceptions
 *        that can occur during the process. Furthermore, this class establishes the
 *        appropriate syntax that the files must have. Such syntax is (regular expression):
 *        NOTE: this is a header only template.
 *
 *        (^[\s\t]*([-+]?[0-9]*\.?[0-9]+([eE][-+]?[0-9]+)?)[\s\t]+([-+]?[0-9]*\.?[0-9]+([eE][-+]?[0-9]+)?)[\s\t]+
 *         ([-+]?[0-9]*\.?[0-9]+([eE][-+]?[0-9]+)?)[\s\t\r\n]+$)+
 *        
 * @author Luis Carlos Garcia-Peraza Herrera (luis.herrera.14@ucl.ac.uk).
 * @date   14 Mar 2015. 
 */

#ifndef __FILE_READER_H__
#define __FILE_READER_H__

// My includes 
#include "pointxyz.h"
#include "pointcloud.h"

template <typename Scalar = float>
class FileReader {

public:
	FileReader(const std::string &path);
	
	void setFilePath(const std::string &path);
	PointCloud<PointXYZ<Scalar> > parseInputFile() const;

	static PointCloud<PointXYZ<Scalar> > parseInputFile(const std::string &path);

private:
	std::string m_path;
};

#include "filereader.tpp"

#endif
