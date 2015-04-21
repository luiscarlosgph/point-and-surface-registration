/**
 * @brief Implementation of the templated class SurfaceBasedRegistration. Note that this tenmplate
 *        file is included in the header file.
 *
 * @author Luis Carlos Garcia-Peraza Herrera (luis.herrera.14@ucl.ac.uk).
 * @date   14 Mar 2015. 
 */

// My includes
// #include "pointbasedregistration.h" 

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Geometry>
#include <cmath>
#include <limits>

// My includes
#include "exception.h"
#include "pointbasedregistration.h"

template <typename PointFixed, typename PointMoving, typename Scalar>
SurfaceBasedRegistration<PointFixed, PointMoving, Scalar>::SurfaceBasedRegistration() {

}

template <typename PointFixed, typename PointMoving, typename Scalar>
SurfaceBasedRegistration<PointFixed, PointMoving, Scalar>::~SurfaceBasedRegistration() {

}

template <typename PointFixed, typename PointMoving, typename Scalar>
void SurfaceBasedRegistration<PointFixed, PointMoving, Scalar>::setFixedCloud(const PointCloud<PointFixed> &cloud) {
	m_fixed = cloud;
}

template <typename PointFixed, typename PointMoving, typename Scalar>
void SurfaceBasedRegistration<PointFixed, PointMoving, Scalar>::setMovingCloud(const PointCloud<PointMoving> &cloud) {
	m_moving = cloud;
}

template <typename PointFixed, typename PointMoving, typename Scalar>
void SurfaceBasedRegistration<PointFixed, PointMoving, Scalar>::align() {

	// Sanity check paranoia
	if (m_fixed.empty()) 
		throw FixedSetIsEmpty();
	if (m_moving.empty()) 
		throw MovingSetIsEmpty();
	if (m_fixed.size() < (*(m_fixed.begin())).size())
		throw NotEnoughDataInFixedSet();
	if (m_moving.size() < (*(m_moving.begin())).size())
		throw NotEnoughDataInMovingSet();
	if (m_fixed.size() != m_moving.size())
		throw UnequalNumberOfPoints();	

	// Initialise transformation homogeneous matrix
	PointBasedRegistration<PointFixed, PointFixed, Scalar> pbr;
	Scalar fre;
	m_hom = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Identity(m_moving[0].size() + 1, m_moving[0].size() + 1);
	m_fre = std::numeric_limits<Scalar>::infinity();

	// Surface registration
	PointCloud<PointMoving> m_movingTransform;
	PointCloud<PointFixed> m_fixedSubset;
	PointMoving pm_;
	int iter = 0;
	do {
		// For each point in one point set, find the closest corresponding point
		// in the other point set after it has been transformed by the current 
		// transformation
		for (int i = 0; i < m_moving.size(); i++) {
			pm_ = transformPoint(m_moving[i]);
			m_movingTransform.push_back(pm_);
			m_fixedSubset.push_back(closestPoint(pm_, m_fixed));
		}		
		
		// Use point based registration (Question. 2) to get a transformation 
		// aligning the points
		pbr.setFixedCloud(m_fixedSubset);
		pbr.setMovingCloud(m_movingTransform);
		pbr.align();

		// Update the current transformation
		fre = m_fre;
		m_hom = pbr.getHomogeneousMatrix();
		m_fre = pbr.getFiducialRegistrationError();	
		iter++;
	} while (fabs(m_fre - fre) > SurfaceBasedRegistration::EPSILON && iter < SurfaceBasedRegistration::MAX_ITER);

}

template <typename PointFixed, typename PointMoving, typename Scalar>
Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> SurfaceBasedRegistration<PointFixed, PointMoving, Scalar>::getHomogeneousMatrix() const {
	return m_hom;
}

template <typename PointFixed, typename PointMoving, typename Scalar>
Scalar SurfaceBasedRegistration<PointFixed, PointMoving, Scalar>::getFiducialRegistrationError() const {
	return m_fre;
}

template <typename PointFixed, typename PointMoving, typename Scalar>
PointMoving SurfaceBasedRegistration<PointFixed, PointMoving, Scalar>::transformPoint(const PointMoving &pm) const {
	Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> pmHom = pm.homogeneous();
	pmHom = m_hom * pmHom;
	PointMoving retval;
	retval = pmHom.block(0, 0, retval.size(), 1);
	return retval;
}

template <typename PointFixed, typename PointMoving, typename Scalar>
PointFixed SurfaceBasedRegistration<PointFixed, PointMoving, Scalar>::closestPoint(const PointMoving &pm, const PointCloud<PointFixed> &cloud) const {		
	Scalar dist = std::numeric_limits<Scalar>::infinity();
	Scalar tempDist;
	int j = 0;
	for (int i = 0; i < cloud.size(); i++) {
		tempDist = (cloud[i] - pm).squaredNorm();
		if (tempDist < dist) {
			dist = tempDist;
			j = i;
		}
	}  
	return cloud[j];
}
