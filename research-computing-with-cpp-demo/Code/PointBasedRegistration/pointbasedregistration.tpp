/**
 * @brief Implementation of the templated class PointBasedRegistration. Note that this tenmplate
 *        file is included in the header file.
 *
 * @author Luis Carlos Garcia-Peraza Herrera (luis.herrera.14@ucl.ac.uk).
 * @date   14 Mar 2015. 
 */

// My includes
// #include "pointbasedregistration.h" 

#include <iostream>
#include <Eigen/SVD>
#include <cmath>

// My includes
#include "exception.h"

template <typename PointFixed, typename PointMoving, typename Scalar>
PointBasedRegistration<PointFixed, PointMoving, Scalar>::PointBasedRegistration() {

}

template <typename PointFixed, typename PointMoving, typename Scalar>
PointBasedRegistration<PointFixed, PointMoving, Scalar>::~PointBasedRegistration() {

}

template <typename PointFixed, typename PointMoving, typename Scalar>
void PointBasedRegistration<PointFixed, PointMoving, Scalar>::setFixedCloud(const PointCloud<PointFixed> &cloud) {
	m_fixed = cloud;
}

template <typename PointFixed, typename PointMoving, typename Scalar>
void PointBasedRegistration<PointFixed, PointMoving, Scalar>::setMovingCloud(const PointCloud<PointMoving> &cloud) {
	m_moving = cloud;
}

template <typename PointFixed, typename PointMoving, typename Scalar>
void PointBasedRegistration<PointFixed, PointMoving, Scalar>::align() {

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

	// Calculate centroid (p) of the source points
	typename PointCloud<PointFixed>::iterator itFixed = m_fixed.begin();
	PointFixed p(*itFixed);
	p /= m_fixed.size();
	itFixed++;
	while (itFixed != m_fixed.end()) {
		p += (*itFixed) / m_fixed.size();	
		++itFixed;
	}

	// Calculate the centroid (p_) of the target points
	typename PointCloud<PointMoving>::iterator itMov = m_moving.begin();
	PointMoving p_(*itMov);
	p_ /= m_moving.size();
	itMov++;
	while (itMov != m_moving.end()) {
		p_ += (*itMov) / m_moving.size();	
		++itMov;
	}

	// Calculate the [source points minus their centroid (p)] => qi
	PointCloud<PointFixed> qi(m_fixed);
	for (typename PointCloud<PointFixed>::iterator it = qi.begin(); it != qi.end(); ++it) {
		(*it) -= p;
	}

	// Calculate the [target points minus their centroid (p_)] => qi_
	PointCloud<PointFixed> qi_(m_moving);
	for (typename PointCloud<PointMoving>::iterator it = qi_.begin(); it != qi_.end(); ++it) {
		(*it) -= p_;
	}

	// Calculate H
	Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> H = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(p.size(), p.size());
	typename PointCloud<PointFixed>::iterator it_qi = qi.begin();
	typename PointCloud<PointMoving>::iterator it_qi_ = qi_.begin();
	while (it_qi != qi.end() && it_qi_ != qi_.end()) {
		H += (*it_qi_).matrix() * (*it_qi).matrix().transpose();
		it_qi++;
		it_qi_++;
	}

	// Calculate X 
	Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> X = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(p.size(), p.size());
	Eigen::JacobiSVD<Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> > svdOfH(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
	X = svdOfH.matrixV() * svdOfH.matrixU().transpose();

	// Calculate R 
	Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> R = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(p.size(), p.size());
	if (X.determinant() > 0) { // If determinan is +1, used > 0 to avoid numerical problems
		R = X;	
	}
	else { 
		// If this code was reached then we are mathematically sure that the determinant is -1 
		// If the dimension is 3 and H has at least a singular value equal to zero, then we can solve it by doing X' = V' * Ut, V' = [ v1, v2, -v3 ] 
		if (p.size() == 3 && svdOfH.singularValues().nonZeros() < 3) {
			Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> V_ = svdOfH.matrixV(); 
			V_.row(2) *= -1;
			R = V_ * svdOfH.matrixU().transpose();	
		}
		else {
			throw AlgorithmFailed();	
		}
	}

	// Calculate T
	Eigen::Matrix<Scalar, Eigen::Dynamic, 1> T;
	T = p - R * p_;	

	// Build homogeneous matrix for the N-dimensional case
	// Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> hom = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(p.size() + 1, p.size() + 1);
	m_hom.resize(p.size() + 1, p.size() + 1);
	m_hom.setZero();
	m_hom.col(p.size()).setOnes();
	m_hom.block(0, 0, p.size(), p.size()) = R;
	m_hom.block(0, p.size(), p.size(), 1) = T;

	// Calculate fiducial registration error (RMS error)
	m_fre = 0;
	itMov = m_moving.begin();
	itFixed = m_fixed.begin();
	Eigen::Matrix<Scalar, Eigen::Dynamic, 1> pHom;
	Eigen::Matrix<Scalar, Eigen::Dynamic, 1> p_Hom;
	pHom.resize(p.size() + 1);
	p_Hom.resize(p_.size() + 1);
	pHom(p.size()) = 1;
	p_Hom(p_.size()) = 1;
	while (itMov != m_moving.end() && itFixed != m_fixed.end()) {
		pHom.block(0, 0, p.size(), 1) = (*itFixed);
		p_Hom.block(0, 0, p_.size(), 1) = (*itMov);
		m_fre += ((pHom - m_hom * p_Hom).array().pow(2) / m_moving.size()).sum();
		itMov++;
		itFixed++;
	}
	m_fre = sqrt(m_fre);	
}

template <typename PointFixed, typename PointMoving, typename Scalar>
Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> PointBasedRegistration<PointFixed, PointMoving, Scalar>::getHomogeneousMatrix() const {
	return m_hom;
}

template <typename PointFixed, typename PointMoving, typename Scalar>
Scalar PointBasedRegistration<PointFixed, PointMoving, Scalar>::getFiducialRegistrationError() const {
	return m_fre;
}
