/**
 * @class  PointXYZ represents a 3D point. This class is header only.
 *         WARNING! Points cannot be transposed so the transpose() method does nothing.
 * 
 * @author Luis Carlos Garcia-Peraza Herrera (luis.herrera.14@ucl.ac.uk).
 * @date   15 Mar 2015.
 */

#ifndef __POINT_XYZ_H__
#define __POINT_XYZ_H__

#include <Eigen/Dense>

// My includes
#include "exception.h"

template <typename Scalar = float>
class PointXYZ : public Eigen::Matrix<Scalar, 3, 1> {

public:
	PointXYZ() : Eigen::Matrix<Scalar, 3, 1>() {};
	/*
	PointXYZ(const PointXYZ &other) {
		if (this != &other) { 
			(*this)[0] = other[0];		
			(*this)[1] = other[1];		
			(*this)[2] = other[2];		
		}
	}
	*/
	PointXYZ(Scalar xCoord, Scalar yCoord, Scalar zCoord) : Eigen::Matrix<Scalar, 3, 1>() {
		x(xCoord);
		y(yCoord); 
		z(zCoord);
	}
	~PointXYZ() {};

	// Getters and setters
	Scalar x() const { return (*this)[0]; };
	Scalar y() const { return (*this)[1]; };
	Scalar z() const { return (*this)[2]; };
	void x(const Scalar xCoord) { (*this)[0] = xCoord; };
	void y(const Scalar yCoord) { (*this)[1] = yCoord; };
	void z(const Scalar zCoord) { (*this)[2] = zCoord; };

	// Overload transpose
	PointXYZ<Scalar>& transpose() {
		// This operation does not make sense, a vector should not be transposed
		// TODO: An exception should probably be thrown here!
		return *this;
	}

	// Overloaded operators
	PointXYZ<Scalar>& operator=(const Eigen::Matrix<Scalar, 3, 1> &other) {
		if (this != &other) { 
			(*this)(0, 0) = other(0, 0);		
			(*this)(1, 0) = other(1, 0);		
			(*this)(2, 0) = other(2, 0);		
		}
		return *this;	
	}
	/*
	float& operator[](std::size_t idx) { 
		switch(idx) {
			case 0: return (*this)(0, 0); break;
			case 1: return (*this)(1, 0); break;
			case 2: return (*this)(2, 0); break;
			default:
				throw IndexOutOfBounds();
		}
	};
	const float& operator[](std::size_t idx) const { 
		switch(idx) {
			case 0: return (*this)(0, 0); break;
			case 1: return (*this)(1, 0); break;
			case 2: return (*this)(2, 0); break;
			default:
				throw IndexOutOfBounds();
		}
	};
	PointXYZ& operator=(const PointXYZ &other) {
		if (this != &other) { 
			(*this)[0] = other[0];		
			(*this)[1] = other[1];		
			(*this)[2] = other[2];		
		}
		return *this;	
	}
	*/
	/*
	PointXYZ operator/(PointXYZ const &other) const {
		PointXYZ retval = *this;
		retval /= other;
		return result;
	}
	PointXYZ& operator/=(PointXYZ const &other) {
		(*this)[0] /= other[0];		
		(*this)[1] /= other[1];		
		(*this)[2] /= other[2];		
		return *this;
	}
	*/
};

// Typedefs
typedef PointXYZ<float> PointXYZf;
typedef PointXYZ<double> PointXYZd;

#endif
