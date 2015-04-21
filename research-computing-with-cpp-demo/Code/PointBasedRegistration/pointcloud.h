/**
 * @brief Typedef from std::vector to PointCloud. A PointCloud represents a set of points.
 * 
 * @author Luis Carlos Garcia-Peraza Herrera (luis.herrera.14@ucl.ac.uk).
 * @date   14 Mar 2015. 
 */

#ifndef __CLOUD_H__
#define __CLOUD_H__

#include <vector>

// My includes
#include "pointxyz.h"

template<typename T>
using PointCloud = std::vector<T>;

#endif
