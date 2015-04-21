/**
 * @brief  Group of exception classes. 
 *        
 * @author Luis Carlos Garcia-Peraza Herrera (luis.herrera.14@ucl.ac.uk).
 * @date   14 Mar 2015. 
 */

#ifndef __EXCEPTION_H__
#define __EXCEPTION_H__

#include <exception>

/**
 * @class IndexOutOfBounds represents an exception that is raised when trying to access the index
 *        of a vector or matrix that does not exist.
 */
class IndexOutOfBounds: public std::exception {
	virtual const char* what() const throw() {
		return "ERROR! Trying to access an index out of bounds.";
	}
};

/**
 * @class FixedSetIsEmpty represents an exception that is raised when the set of fixed points 
 *        provided is found to be empty.
 */
class FixedSetIsEmpty : public std::exception {
	virtual const char* what() const throw() {
		return "ERROR! The set of fixed points provided is empty.";
	}
};

/**
 * @class MovingSetIsEmpty represents an exception that is raised when the set of moving points 
 *        provided is found to be empty.
 */
class MovingSetIsEmpty : public std::exception {
	virtual const char* what() const throw() {
		return "ERROR! The set of moving points provided is empty.";
	}
};

/**
 * @class NotEnoughDataInFixedSet represents an exception that is raised when the set of fixed
 *        points has less than N points. N being the dimension of each point.
 */
class NotEnoughDataInFixedSet : public std::exception {
	virtual const char* what() const throw() {
		return "ERROR! The number of points in the fixed set is less than the dimension of the points.";
	}
};

/**
 * @class NotEnoughDataInMovingSet represents an exception that is raised when the set of moving
 *        points has less than N points. N being the dimension of each point.
 */
class NotEnoughDataInMovingSet : public std::exception {
	virtual const char* what() const throw() {
		return "ERROR! The number of points in the moving set is less than the dimension of the points.";
	}
};

/**
 * @class UnequalNumberOfPoints represents an exception that is raised when the number of points 
 *        in the two sets (fixed and moving).
 */
class UnequalNumberOfPoints : public std::exception {
	virtual const char* what() const throw() {
		return "ERROR! There is an unequal number of points in the two sets provided for registration.";
	}
};

/**
 * @class AlgorithmFailed represents an exception that is raised when the algorithm cannot find a
 *        solution.
 */
class AlgorithmFailed : public std::exception {
	virtual const char* what() const throw() {
		return "ERROR! This conventional least-squares algorithm based on SVD failed. An alternative "
		       "approach you could try to solve this registration is a RANSAC-like technique.";
	}
};

#endif
