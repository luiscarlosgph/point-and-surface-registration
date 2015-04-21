/**
 * @brief Unit tests for the Point Based Registration.
 *        
 * @author Luis Carlos Garcia-Peraza Herrera (luis.herrera.14@ucl.ac.uk).
 * @date   14 Mar 2015. 
 */

#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include "catch.hpp"

#include <stdio.h>
#include <iostream>
#include <Eigen/Dense>
#include <typeinfo>
#include <regex>

// My includes
#include "pointxyz.h"
#include "pointcloud.h"
#include "filereader.h"
#include "filewriter.h"
// #include "pointbasedregistration.h"
#include "surfacebasedregistration.h"

// Test PointXYZ template
TEST_CASE("Successful manipulation of the PointXYZ template", "[PointXYZ]") {

	PointXYZ<> a;
	PointXYZ<float> b;
	PointXYZ<float> c;
	PointXYZ<double> d;

	a.x(1.0);
	a.y(2.0);
	a.z(3.0);
	b[0] = 4.0;
	b[1] = 7.0;
	b[2] = 12.0;

	SECTION("Retrieve and set point coordinates") {

		REQUIRE(a.x() == 1.0);
		REQUIRE(a.y() == 2.0);
		REQUIRE(a.z() == 3.0);
		REQUIRE(a[0] == 1.0);
		REQUIRE(a[1] == 2.0);
		REQUIRE(a[2] == 3.0);
		REQUIRE(b.x() == 4.0);
		REQUIRE(b.y() == 7.0);
		REQUIRE(b.z() == 12.0);

	}

	SECTION("Testing PointXYZ operators: +, -, * (scalar), / (scalar)") {
		
		std::cout << "Testing substraction of two PointXYZ" << std::endl;
		c = b - a;
		REQUIRE(c[0] == 3.0);
		REQUIRE(c[1] == 5.0);
		REQUIRE(c[2] == 9.0);
		
		std::cout << "Testing sum of two PointXYZ" << std::endl;
		c = a + b;
		REQUIRE(c[0] == 5.0);
		REQUIRE(c[1] == 9.0);
		REQUIRE(c[2] == 15.0);

		std::cout << "Testing multiplication of PointXYZ by a scalar" << std::endl;
		c = b + a * 10.0;
		REQUIRE(c[0] == 14.0);
		REQUIRE(c[1] == 27.0);
		REQUIRE(c[2] == 42.0);

		std::cout << "Testing division of PointXYZ by a scalar" << std::endl;
		c = (a + b) / 2.0;  
		REQUIRE(c[0] == 2.5);
		REQUIRE(c[1] == 4.5);
		REQUIRE(c[2] == 7.5);

	}

	SECTION("Testing PointXYZ operators: +=, -=, *= (scalar), /= (scalar)") {
			
		std::cout << "Testing += for PointXYZ" << std::endl;
		a += b;
		REQUIRE(a[0] == 5.0);
		REQUIRE(a[1] == 9.0);
		REQUIRE(a[2] == 15.0);

		std::cout << "Testing -= for PointXYZ" << std::endl;
		a -= b;
		REQUIRE(a[0] == 1.0);
		REQUIRE(a[1] == 2.0);
		REQUIRE(a[2] == 3.0);
		
		std::cout << "Testing *= for PointXYZ" << std::endl;
		a *= 3.0;
		REQUIRE(a[0] == 3.0);
		REQUIRE(a[1] == 6.0);
		REQUIRE(a[2] == 9.0);

		std::cout << "Testing /= for PointXYZ" << std::endl;
		a /= 2.0;
		REQUIRE(a[0] == 1.5);
		REQUIRE(a[1] == 3.0);
		REQUIRE(a[2] == 4.5);

	}

	SECTION("Testing the no-transpose of a PointXYZ") {
		
		// This should not transpose anything because a and b are column vectors, not a matrices
		std::cout << "Testing that transpose that nothing on PointXYZ" << std::endl;
		c = a.transpose(); 
		REQUIRE(c(0, 0) == 1.0);
		REQUIRE(c(1, 0) == 2.0);
		REQUIRE(c(2, 0) == 3.0);
		REQUIRE(c[0] == 1.0);
		REQUIRE(c[1] == 2.0);
		REQUIRE(c[2] == 3.0);
	
	}

}

// Testing the PointBasedRegistration template
TEST_CASE("Successful registration of two 3D point sets", "[PointBasedRegistration]") {
	
	PointBasedRegistration<PointXYZ<float>, PointXYZ<float>, float> pbr;
	PointXYZ<float> a, a_;
	PointXYZ<float> b, b_;
	PointXYZ<float> c, c_;
	PointXYZ<float> d, d_;

	// Point A (fixed)
	a[0] = 5.40;
	a[1] = 5.2398;
	a[2] = 12.6902;

	// Point B (fixed)
	b[0] = 8.40;
	b[1] = 3.9914;
	b[2] = 15.4180;

	// Point C (fixed)
	c[0] = 11.40;
	c[1] = 2.7429;
	c[2] = 18.1459;

	// Point D (fixed)
	d[0] = 14.40;
	d[1] = 1.4945;
	d[2] = 20.8738;

	// Point A_ (moving)
	a_[0] = 1.0;
	a_[1] = 2.0;
	a_[2] = 3.0;

	// Point B_ (moving)
	b_[0] = 4.0;
	b_[1] = 5.0;
	b_[2] = 3.0;

	// Point C_ (moving)
	c_[0] = 7.0;
	c_[1] = 8.0;
	c_[2] = 3.0;

	// Point D_ (moving)
	d_[0] = 10.0;
	d_[1] = 11.0;
	d_[2] = 3.0;

	// Real homogeneous transformation matrix
	Eigen::Matrix<float, 4, 4> realHomTransform;
	realHomTransform(0, 0) = 1.0;
	realHomTransform(0, 1) = 0.0;
	realHomTransform(0, 2) = 0.0;
	realHomTransform(0, 3) = 4.0;
	realHomTransform(1, 0) = 0.0;
	realHomTransform(1, 1) = -0.4161;
	realHomTransform(1, 2) = -0.9093;
	realHomTransform(1, 3) = 8.8;
	realHomTransform(2, 0) = 0.0;
	realHomTransform(2, 1) = 0.9093;
	realHomTransform(2, 2) = -0.4161;
	realHomTransform(2, 3) = 12.12;
	realHomTransform(3, 0) = 0.0;
	realHomTransform(3, 1) = 0.0;
	realHomTransform(3, 2) = 0.0;
	realHomTransform(3, 3) = 1.0;
	
	SECTION("Testing successful registration case") {
		std::cout << "Testing a successful registration case" << std::endl;
		bool caught = false;
		
		// Give the sets to the template
		PointCloud<PointXYZ<float> > fixed;
		PointCloud<PointXYZ<float> > moving;
		fixed.push_back(a);
		fixed.push_back(b);
		fixed.push_back(c);
		fixed.push_back(d);
		moving.push_back(a_);
		moving.push_back(b_);
		moving.push_back(c_);
		moving.push_back(d_);

		pbr.setFixedCloud(fixed);
		pbr.setMovingCloud(moving);
		
		// Print fixed
		std::cout << "\nFixed:" << std::endl;
		for (int i = 0; i < fixed.size(); i++){
			std:: cout << fixed[i][0] << " " << fixed[i][1] << " " << fixed[i][2] << std::endl;
		}

		// Print moving
		std::cout << "\nMoving:" << std::endl;
		for (int i = 0; i < fixed.size(); i++){
			std:: cout << moving[i][0] << " " << moving[i][1] << " " << moving[i][2] << std::endl;
		}

		try {
			pbr.align();
			Eigen::Matrix<float, 4, 4> homTransformMatrix = pbr.getHomogeneousMatrix();
			float fre = pbr.getFiducialRegistrationError();

			// Print registration matrix and fiducial registration error to the user
			std::cout << std::endl << "The real homogeneous matrix is:" << std::endl << realHomTransform << std::endl << std::endl;
			std::cout << "The homogeneous registration matrix is:" << std::endl << homTransformMatrix << std::endl << std::endl;
			std::cout << "The fiducial registration error is: " << fre << " pixels." << std::endl << std::endl;
			REQUIRE(fre < 0.0001);
		} 
		catch(const std::exception &e) {
			std::cerr << e.what() << std::endl;	
			caught = true;
		}
		REQUIRE(!caught);
	}

	SECTION("Testing unequal number of points case") {
		std::cout << "Testing registration with an unequal number of points" << std::endl;
		bool caught = false;
		
		// Give the sets to the template
		PointCloud<PointXYZ<float> > fixed;
		PointCloud<PointXYZ<float> > moving;
		fixed.push_back(a);
		fixed.push_back(b);
		fixed.push_back(c);
		fixed.push_back(d);
		moving.push_back(a_);
		moving.push_back(b_);
		moving.push_back(c_);
		pbr.setFixedCloud(fixed);
		pbr.setMovingCloud(moving);
		try {
			pbr.align();
			Eigen::Matrix<float, 4, 4> homTransformMatrix = pbr.getHomogeneousMatrix();
			float fre = pbr.getFiducialRegistrationError();

			// Print registration matrix and fiducial registration error to the user
			std::cout << std::endl << "The real homogeneous matrix is:" << std::endl << realHomTransform << std::endl << std::endl;
			std::cout << "The homogeneous registration matrix is:" << std::endl << homTransformMatrix << std::endl << std::endl;
			std::cout << "The fiducial registration error is: " << fre << " pixels." << std::endl << std::endl;
		} 
		catch(const std::exception &e) {
			std::cerr << e.what() << std::endl;	
			std::cerr << "Type of the exception: " << typeid(e).name() << std::endl;
			REQUIRE(std::regex_match(typeid(e).name(), std::regex(".*UnequalNumberOfPoints.*")));
			caught = true;
		}		
		REQUIRE(caught);
		std::cout << std::endl;
	}

	SECTION("Testing not enough data case, i.e. number of points lower than dimension of points") {
		std::cout << "Testing registration without enough data (only two points per set)" << std::endl;
		bool caught = false;
		
		// Give the sets to the template
		PointCloud<PointXYZ<float> > fixed;
		PointCloud<PointXYZ<float> > moving;
		fixed.push_back(a);
		fixed.push_back(b);
		moving.push_back(a_);
		moving.push_back(b_);
		pbr.setFixedCloud(fixed);
		pbr.setMovingCloud(moving);
		try {
			pbr.align();
			Eigen::Matrix<float, 4, 4> homTransformMatrix = pbr.getHomogeneousMatrix();
			float fre = pbr.getFiducialRegistrationError();

			// Print registration matrix and fiducial registration error to the user
			std::cout << std::endl << "The real homogeneous matrix is:" << std::endl << realHomTransform << std::endl << std::endl;
			std::cout << "The homogeneous registration matrix is:" << std::endl << homTransformMatrix << std::endl << std::endl;
			std::cout << "The fiducial registration error is: " << fre << " pixels." << std::endl << std::endl;
		} 
		catch(const std::exception &e) {
			std::cerr << e.what() << std::endl;	
			std::cerr << "Type of the exception: " << typeid(e).name() << std::endl;
			REQUIRE(std::regex_match(typeid(e).name(), std::regex(".*NotEnoughDataInFixedSet.*")));
			caught = true;
		}
		REQUIRE(caught);
		std::cout << std::endl;
	}
	
	SECTION("Testing not enough data case, i.e. number of points lower than dimension of points") {
		std::cout << "Testing registration without enough data (only two points in the moving set)" << std::endl;
		bool caught = false;
		
		// Give the sets to the template
		PointCloud<PointXYZ<float> > fixed;
		PointCloud<PointXYZ<float> > moving;
		fixed.push_back(a);
		fixed.push_back(b);
		fixed.push_back(c);
		fixed.push_back(d);
		moving.push_back(a_);
		moving.push_back(b_);
		pbr.setFixedCloud(fixed);
		pbr.setMovingCloud(moving);
		try {
			pbr.align();
			Eigen::Matrix<float, 4, 4> homTransformMatrix = pbr.getHomogeneousMatrix();
			float fre = pbr.getFiducialRegistrationError();

			// Print registration matrix and fiducial registration error to the user
			std::cout << std::endl << "The real homogeneous matrix is:" << std::endl << realHomTransform << std::endl << std::endl;
			std::cout << "The homogeneous registration matrix is:" << std::endl << homTransformMatrix << std::endl << std::endl;
			std::cout << "The fiducial registration error is: " << fre << " pixels." << std::endl << std::endl;
		} 
		catch(const std::exception &e) {
			std::cerr << e.what() << std::endl;	
			std::cerr << "Type of the exception: " << typeid(e).name() << std::endl;
			REQUIRE(std::regex_match(typeid(e).name(), std::regex(".*NotEnoughDataInMovingSet.*")));
			caught = true;
		}
		REQUIRE(caught);
		std::cout << std::endl;
	}

	SECTION("Testing no data case in the fixed set") {
		std::cout << "Testing with no data in the fixed set." << std::endl;
		bool caught = false;
		
		// Give the sets to the template
		PointCloud<PointXYZ<float> > fixed;
		PointCloud<PointXYZ<float> > moving;
		pbr.setFixedCloud(fixed);
		pbr.setMovingCloud(moving);

		try {
			pbr.align();
			Eigen::Matrix<float, 4, 4> homTransformMatrix = pbr.getHomogeneousMatrix();
			float fre = pbr.getFiducialRegistrationError();

			// Print registration matrix and fiducial registration error to the user
			std::cout << std::endl << "The real homogeneous matrix is:" << std::endl << realHomTransform << std::endl << std::endl;
			std::cout << "The homogeneous registration matrix is:" << std::endl << homTransformMatrix << std::endl << std::endl;
			std::cout << "The fiducial registration error is: " << fre << " pixels." << std::endl << std::endl;
		} 
		catch(const std::exception &e) {
			std::cerr << e.what() << std::endl;	
			std::cerr << "Type of the exception: " << typeid(e).name() << std::endl;
			REQUIRE(std::regex_match(typeid(e).name(), std::regex(".*FixedSetIsEmpty.*")));
			caught = true;
		}	
		REQUIRE(caught);
		std::cout << std::endl;
	}

	SECTION("Testing no data case in the moving set") {
		std::cout << "Testing with no data in the moving set." << std::endl;
		bool caught = false;
		
		// Give the sets to the template
		PointCloud<PointXYZ<float> > fixed;
		PointCloud<PointXYZ<float> > moving;
		fixed.push_back(a);
		fixed.push_back(b);
		fixed.push_back(c);
		fixed.push_back(d);
		pbr.setFixedCloud(fixed);
		pbr.setMovingCloud(moving);

		try {
			pbr.align();
			Eigen::Matrix<float, 4, 4> homTransformMatrix = pbr.getHomogeneousMatrix();
			float fre = pbr.getFiducialRegistrationError();

			// Print registration matrix and fiducial registration error to the user
			std::cout << std::endl << "The real homogeneous matrix is:" << std::endl << realHomTransform << std::endl << std::endl;
			std::cout << "The homogeneous registration matrix is:" << std::endl << homTransformMatrix << std::endl << std::endl;
			std::cout << "The fiducial registration error is: " << fre << " pixels." << std::endl << std::endl;
		} 
		catch(const std::exception &e) {
			std::cerr << e.what() << std::endl;	
			std::cerr << "Type of the exception: " << typeid(e).name() << std::endl;
			REQUIRE(std::regex_match(typeid(e).name(), std::regex(".*MovingSetIsEmpty.*")));
			caught = true;
		}	
		REQUIRE(caught);
		std::cout << std::endl;
	}
	
	SECTION("Noisy case, alogithm fails") {
		std::cout << "Testing a highly noisy case, algorithm fails." << std::endl;
		bool caught = false;
		PointBasedRegistration<PointXYZ<float>, PointXYZ<float>, float> pbr;
		
		// Give the sets to the template
		PointCloud<PointXYZ<float> > fixed;
		PointCloud<PointXYZ<float> > moving;
		
		// Point A (fixed)
		a[0] = 1.0;
		a[1] = 2.0;
		a[2] = 3.0;

		// Point B (fixed)
		b[0] = 4.0;
		b[1] = 5.0;
		b[2] = 6.0;

		// Point C (fixed)
		c[0] = 7.0;
		c[1] = 8.0;
		c[2] = 9.0;

		// Point D (fixed)
		d[0] = 10.0;
		d[1] = 11.0;
		d[2] = 12.0;
		
		// Point A (fixed)
		a_[0] = 13.0;
		a_[1] = 14.0;
		a_[2] = 15.0;

		// Point B (fixed)
		b_[0] = 16.0;
		b_[1] = 17.0;
		b_[2] = 18.0;

		// Point C (fixed)
		c_[0] = 19.0;
		c_[1] = 20.0;
		c_[2] = 21.0;

		// Point D (fixed)
		d_[0] = 22.0;
		d_[1] = 23.0;
		d_[2] = 24.0;

		fixed.push_back(a);
		fixed.push_back(b);
		fixed.push_back(c);
		fixed.push_back(d);
		moving.push_back(a_);
		moving.push_back(b_);
		moving.push_back(c_);
		moving.push_back(d_);
		pbr.setFixedCloud(fixed);
		pbr.setMovingCloud(moving);
		
		try {
			pbr.align();
			Eigen::Matrix<float, 4, 4> homTransformMatrix = pbr.getHomogeneousMatrix();
			// float fre = pbr.getFiducialRegistrationError();

			// Print registration matrix and fiducial registration error to the user
			// std::cout << std::endl << "The real homogeneous matrix is:" << std::endl << realHomTransform << std::endl << std::endl;
			// std::cout << "The homogeneous registration matrix is:" << std::endl << homTransformMatrix << std::endl << std::endl;
			// std::cout << "The fiducial registration error is: " << fre << " pixels." << std::endl << std::endl;
		} 
		catch(const std::exception &e) {
			std::cerr << e.what() << std::endl;	
			std::cerr << "Type of the exception: " << typeid(e).name() << std::endl;
			REQUIRE(std::regex_match(typeid(e).name(), std::regex(".*AlgorithmFailed.*")));
			caught = true;
		}	
		REQUIRE(caught);
		std::cout << std::endl;
	}

}

// Test FileReader template
TEST_CASE("Testing that the class FileReader behaves properly", "[FileReader]") { 
	
	SECTION("Successful reading of a valid file") {
		std::cout << "Testing that the class is able to read a proper file" << std::endl << std::endl;

		// Create a cloud of points
		PointXYZ<float> p;
		PointCloud<PointXYZ<float> > mat;
		p[0] = 1.0;
		p[1] = 2.0;
		p[2] = 3.0;
		mat.push_back(p);
		p[0] = 4.0;
		p[1] = 5.0;
		p[2] = 6.0;
		mat.push_back(p);
		p[0] = 7.0;
		p[1] = 8.0;
		p[2] = 9.0;
		mat.push_back(p);
		p[0] = 10.0;
		p[1] = 11.0;
		p[2] = 12.0;
		mat.push_back(p);

		std::string path(".TestingFileReader.mat");
		// Create a valid file with that cloud of points
		{ // RAII
			std::ofstream ofs(path);
			if (!ofs.is_open())
				throw std::runtime_error("ERROR! Unable to open " + path + " file.");
			ofs << "1.0 2.0 3.0\n";
			ofs << "4.0 5.0 6.0\n";
			ofs << "7.0 8.0 9.0\n";
			ofs << "10.0 11.0 12.0\n";
		}

		// Read it
		PointCloud<PointXYZ<float> > mat2 = FileReader<float>::parseInputFile(path);

		// Check that the two matrices have the same values
		PointXYZ<float> p2;
		for(int i = 0; i < mat.size(); ++i) {
			REQUIRE(mat[i] == mat2[i]);	
		} 
		
		// Delete the test file
		remove(path.c_str());
	}

	SECTION("File does not exist") {
		std::cout << "Trying to read a file that does not exist" << std::endl;
		bool caught = false;
		try {
			FileReader<float>::parseInputFile("AnExampleOfFileThatDoesNotExist.FILE");
		}
		catch (const std::exception &e) {
			std::cerr << e.what() << std::endl << std::endl;	
			caught = true;
		}
		REQUIRE(caught);
	}

	SECTION("File is malformed.") {
		std::cout << "Trying to read a file that is malformed" << std::endl;
		bool caught = false;
		std::string path(".TestingFileReader.mat");
		// Create a valid file with that cloud of points
		{ // RAII
			std::ofstream ofs(path);
			if (!ofs.is_open())
				throw std::runtime_error("ERROR! Unable to open " + path + " file.");
			ofs << "1.0 2.0 3.0\n";
			ofs << "4.0     6.0\n";
			ofs << "7.0 8.0 9.0\n";
			ofs << "10.0 11.0 12.0\n";
		}
		try {
			PointCloud<PointXYZ<float> > mat2 = FileReader<float>::parseInputFile(path);
		}
		catch (const std::exception &e) {
			std::cerr << e.what() << std::endl << std::endl;	
			caught = true;	
		}
		REQUIRE(caught);

		// Delete the test file
		remove(path.c_str());
	}

}

// Test FileWriter template
TEST_CASE("Testing that the class FileWriter behaves properly", "[FileWriter]") { 

	SECTION("Successful writing to file") {
		std::string path("HomMatrixTestFile.FILE");
		Eigen::Matrix4f mat, mat2;
		mat(0, 0) = 1.0;
		mat(0, 1) = 2.0;
		mat(0, 2) = 3.0;
		mat(0, 3) = 4.0;
		mat(1, 0) = 5.0;
		mat(1, 1) = 6.0;
		mat(1, 2) = 7.0;
		mat(1, 3) = 8.0;
		mat(2, 0) = 9.0;
		mat(2, 1) = 10.0;
		mat(2, 2) = 11.0;
		mat(2, 3) = 12.0;
		mat(3, 0) = 13.0;
		mat(3, 1) = 14.0;
		mat(3, 2) = 15.0;
		mat(3, 3) = 16.0;
		FileWriter<float>::saveHomogeneousMatrix(mat, path);
		{ // RAII
			std::ifstream ifs(path);
			if (!ifs.is_open())
				throw std::runtime_error("ERROR! Unable to open " + path + " file.");
			ifs >> mat2(0, 0);
			ifs >> mat2(0, 1);
			ifs >> mat2(0, 2);
			ifs >> mat2(0, 3);
			ifs >> mat2(1, 0);
			ifs >> mat2(1, 1);
			ifs >> mat2(1, 2);
			ifs >> mat2(1, 3);
			ifs >> mat2(2, 0);
			ifs >> mat2(2, 1);
			ifs >> mat2(2, 2);
			ifs >> mat2(2, 3);
			ifs >> mat2(3, 0);
			ifs >> mat2(3, 1);
			ifs >> mat2(3, 2);
			ifs >> mat2(3, 3);
		}
		REQUIRE(mat.isApprox(mat2, 0.00001));	
		remove(path.c_str());
	}

	SECTION("Output file already exists") {
		std::string path("HomMatrixTestFile.FILE");
		Eigen::Matrix4f mat, mat2;
		mat(0, 0) = 1.0;
		mat(0, 1) = 2.0;
		mat(0, 2) = 3.0;
		mat(0, 3) = 4.0;
		mat(1, 0) = 5.0;
		mat(1, 1) = 6.0;
		mat(1, 2) = 7.0;
		mat(1, 3) = 8.0;
		mat(2, 0) = 9.0;
		mat(2, 1) = 10.0;
		mat(2, 2) = 11.0;
		mat(2, 3) = 12.0;
		mat(3, 0) = 13.0;
		mat(3, 1) = 14.0;
		mat(3, 2) = 15.0;
		mat(3, 3) = 16.0;
		FileWriter<float>::saveHomogeneousMatrix(mat, path);
		bool caught = false;
		try {
			FileWriter<float>::saveHomogeneousMatrix(mat, path);
		}
		catch (const std::exception &e) {
			std::cerr << e.what() << std::endl;	
			caught = true;
		}
		REQUIRE(caught);
		remove(path.c_str());
	}

}

// Complete unit test for Surface based registration.
TEST_CASE("Testing that the normal workflow works", "[All]") { 
	std::cout << "Testing the whole system with the files fixed.txt, moving.txt and matrix.4x4" << std::endl;
	std::string fixedPath("fran_cut.txt");
	std::string movingPath("fran_cut_transformed.txt");
	std::string matrix4x4Path("matrixSurface.4x4");
	try {
		// Parse file with fixed 3D points
		PointCloud<PointXYZ<float> > fixed = FileReader<float>::parseInputFile(fixedPath);
	
		// Parse file with moving 3D points
		PointCloud<PointXYZ<float> > moving = FileReader<float>::parseInputFile(movingPath);

		// Register fixed to moving points
		SurfaceBasedRegistration<PointXYZ<float>, PointXYZ<float>, float> sbr;
		sbr.setFixedCloud(fixed);
		sbr.setMovingCloud(moving);
		sbr.align();
		Eigen::Matrix<float, 4, 4> homTransformMatrix = sbr.getHomogeneousMatrix();
		Eigen::Matrix<float, 4, 4> mat; 

		{ // RAII
			std::ifstream ifs(matrix4x4Path);
			if (!ifs.is_open())
				throw std::runtime_error("ERROR! Unable to open " + matrix4x4Path + " file.");
			ifs >> mat(0, 0);
			ifs >> mat(0, 1);
			ifs >> mat(0, 2);
			ifs >> mat(0, 3);
			ifs >> mat(1, 0);
			ifs >> mat(1, 1);
			ifs >> mat(1, 2);
			ifs >> mat(1, 3);
			ifs >> mat(2, 0);
			ifs >> mat(2, 1);
			ifs >> mat(2, 2);
			ifs >> mat(2, 3);
			ifs >> mat(3, 0);
			ifs >> mat(3, 1);
			ifs >> mat(3, 2);
			ifs >> mat(3, 3);
		}
		REQUIRE(homTransformMatrix.isApprox(mat, 0.15));
	} 
	catch (const std::exception &e) {
		std::cerr << e.what() << std::endl;	
	}
}
