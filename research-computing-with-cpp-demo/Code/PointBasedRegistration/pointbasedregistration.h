/**
 * @class PointBasedRegistration implements a least-squares fitting between two sets of 3D points
 *        as explained in Arun et al. (1987).
 *
 * Example of use:
 * ===============
 *
 *		PointCloud<PointXYZ<float> > fixed;
 *		PointCloud<PointXYZ<float> > moving;
 *    
 *    // The vectors fixed and moving can be filled up in the following way:
 * 	PointXYZ<float> p;
 *    p[0] = 7.0;
 *    p[1] = 8.0;
 *    p[2] = 3.0;
 * 	fixed.push_back(p);
 *  
 *    // Or in this other way (more practical):
 * 	PointCloud<PointXYZ<float> > fixed = FileReader<float>::parseInputFile(
 *       CommandLineOptions::getInstance().getFixedPointsFilePath()
 *    );
 * 	PointCloud<PointXYZ<float> > moving = FileReader<float>::parseInputFile(
 * 		CommandLineOptions::getInstance().getMovingPointsFilePath()
 * 	);
 *
 *		PointBasedRegistration<PointXYZ<float>, PointXYZ<float>, float> pbr;
 *		pbr.setFixedCloud(fixed);
 *		pbr.setMovingCloud(moving);
 *		try {
 *			pbr.align();
 *			Eigen::Matrix<float, 4, 4> homTransformMatrix = pbr.getHomogeneousMatrix();
 *			float fre = pbr.getFiducialRegistrationError();
 *		} 
 *		catch(const std::exception &e) {
 *			std::cerr << e.what() << std::endl;	
 *		} 
 *		
 * @author Luis Carlos Garcia-Peraza Herrera (luis.herrera.14@ucl.ac.uk).
 * @date   14 Mar 2015. 
 */

#ifndef __POINT_BASED_REGISTRATION_H__
#define __POINT_BASED_REGISTRATION_H__

#include <Eigen/Dense>

// My includes
#include "pointcloud.h"

template <typename PointFixed, typename PointMoving, typename Scalar = float>
class PointBasedRegistration {
public:
	PointBasedRegistration();
	~PointBasedRegistration();
	
	// Getters and setters
	void setFixedCloud(const PointCloud<PointFixed> &cloud);
	void setMovingCloud(const PointCloud<PointMoving> &cloud);

	// Registration
	void align();
	Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> getHomogeneousMatrix() const;
	Scalar getFiducialRegistrationError() const;

private:
	Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> m_hom;
	PointCloud<PointFixed> m_fixed;
	PointCloud<PointMoving> m_moving;
	Scalar m_fre; // Fiducial Registration Error
};

#include "pointbasedregistration.tpp"

#endif
