/**
 * @class SurfaceBasedRegistration implements a least-squares fitting between two sets of 3D points
 *        as explained in Arun et al. (1987).
 *
 * Example of use:
 * ===============
 *
 *		
 * @author Luis Carlos Garcia-Peraza Herrera (luis.herrera.14@ucl.ac.uk).
 * @date   14 Mar 2015. 
 */

#ifndef __SURFACE_BASED_REGISTRATION_H__
#define __SURFACE_BASED_REGISTRATION_H__

#include <Eigen/Dense>

// My includes
#include "pointcloud.h"

template <typename PointFixed, typename PointMoving, typename Scalar = float>
class SurfaceBasedRegistration {
public:
	SurfaceBasedRegistration();
	~SurfaceBasedRegistration();
	
	// Getters and setters
	void setFixedCloud(const PointCloud<PointFixed> &cloud);
	void setMovingCloud(const PointCloud<PointMoving> &cloud);

	// Registration
	void align();
	Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> getHomogeneousMatrix() const;
	Scalar getFiducialRegistrationError() const;

private:
	// Aux - apply tranformation to point
	PointMoving transformPoint(const PointMoving &pm) const;

	// Aux - find the closest point in the point cloud
	PointFixed closestPoint(const PointMoving &pm, const PointCloud<PointFixed> &cloud) const;

	// RMS Error epsilon
	static const Scalar EPSILON = 0.000001;
	static const int MAX_ITER = 10;

	Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> m_hom;
	PointCloud<PointFixed> m_fixed;
	PointCloud<PointMoving> m_moving;
	Scalar m_fre; // Fiducial Registration Error
};

#include "surfacebasedregistration.tpp"

#endif
