/**
 * @brief  First part of the coursework for the module Research Computing with C++ taught at
 *         University College London. The aim is to register two sets of 3D points using
 *         least-squares fitting as explained in Arun et al. (1987).
 * 
 * Task list (the X indicates completion):
 * =======================================
 * 
 * 
 * [X] 1. Create a new program entry point for a “Point Based Registration” program, with a name of
 *        your chosing, with a main() function.
 *
 * [X] 2. Use Boost.Program_options to provide command line options. The program should provide the 
 *        options to read two files containing point data and write the resulting matrix to a third
 *        file. If you can’t use Boost, you may write this part yourself using standard C++, but be
 *        aware that marks will be deducted if the command line parsing is not robust. 
 * 
 * [X] 3. Write code to read a list of points from a file. The file format should be plain text, 
 *        reading successive x, y, z coordinates for each point. Read: 
 *        http://www.tomdalling.com/blog/software-design/resource-acquisition-is-initialisation-raii-explained/
 *        Use RAII to make sure that file handles are always closed. Write unit tests using the
 *        Catch framework to test your file access.  
 *
 * [X] 4. Take a fork of https://github.com/UCL-RITS/research-computing-with-cpp-demo.git and create 
 *        a new class/function to implement a Point Based Registration according to Arun’s paper, 
 *        using Eigen. The method should output to console the Fiducial Registration Error (FRE), 
 *        which is the Root Mean Squared (RMS) distance between corresponding points once aligned. 
 *        The final output of the registration should be a 4x4 matrix. Justify your choice of design
 *        of class/function, the input and output parameters. 
 *
 * [X] 5. Create unit tests for the Point Based Registration class/function using the Catch 
 *        framework taught in lecture 1. Consider the basic successful registration cases, and cases 
 *        with no data, not enough data, or unequal number of points. Errors should be raised using 
 *        your own custom exception type, so implement your own exception class. Make sure your 
 *        registration class/function documents the use-cases, and/or throws meaningful errors as
 *        appropriate. Generate your own test data.
 * 
 * [X] 6. Connect your components together, so that your main() program can parse the command line 
 *        arguments, read the points from file, do the registration and output the matrix to file.
 *        Test data can be found in folder:
 *        research-computing-with-cpp-demo/Testing/PointBasedRegistrationData 
 *        
 * [X] 7. Create a unit test that reads files fixed.txt and moving.txt as input, calculate a 4x4 
 *        transformation matrix, and compares the output matrix to matrix.4x4 to 3 decimal places.
 *        The unit test must pass for full marks. Paste your final matrix into the report.
 *
 * [ ] 8. In medical imaging, point-based registration would normally have a relatively small number 
 *        of points. Discuss how your program would currently work with point sets over 100,000 
 *        points. How efficient would it be? What might you change to make your solution scale up? 
 *        You are not required to make these changes. 
 *
 * NOTE: the order of these tasks is different from the one provided in the instructions.
 *
 * @author Luis Carlos Garcia-Peraza Herrera (luis.herrera.14@ucl.ac.uk).
 * @date   10 Mar 2014.
 */

// #include <iostream>
// #include <Eigen/Dense>
// #include <boost/lexical_cast.hpp>
// #include <itkImage.h>

#include <iostream>
#include <cstdlib>
#include <vector>
#include <Eigen/Dense>
// #include <boost/exception/exception.hpp>
// #include <boost/exception/diagnostic_information.hpp>

// My includes
#include "commandlineoptions.h"
#include "pointxyz.h"
#include "pointcloud.h"
#include "pointbasedregistration.h"
#include "filereader.h"
#include "filewriter.h"
#include "surfacebasedregistration.h"

int main(int argc, char** argv) {
	
	// Read command line options
	if (!CommandLineOptions::getInstance().processCmdLineOptions(argc, argv))
		return EXIT_FAILURE;
	
	try {
		// Parse file with fixed 3D points
		PointCloud<PointXYZ<double> > fixed = FileReader<double>::parseInputFile(
			CommandLineOptions::getInstance().getFixedPointsFilePath()
		);
	
		// Parse file with moving 3D points
		PointCloud<PointXYZ<double> > moving = FileReader<double>::parseInputFile(
			CommandLineOptions::getInstance().getMovingPointsFilePath()
		);

		// Register fixed to moving points
		SurfaceBasedRegistration<PointXYZ<double>, PointXYZ<double>, double> sbr;
		sbr.setFixedCloud(fixed);
		sbr.setMovingCloud(moving);
		sbr.align();
		Eigen::Matrix<double, 4, 4> homTransformMatrix = sbr.getHomogeneousMatrix();
		double fre = sbr.getFiducialRegistrationError();

		// Print registration matrix and fiducial registration error to the user
		std::cout << std::endl << "The homogeneous registration matrix is:" << std::endl << homTransformMatrix << std::endl << std::endl;
		std::cout << "The fiducial registration error is: " << fre << " pixels." << std::endl << std::endl;

		// Write output matrix to file
		FileWriter<double>::saveHomogeneousMatrix(homTransformMatrix, CommandLineOptions::getInstance().getOutputFilePath());
	} 
	catch (const std::exception &e) {
		std::cerr << e.what() << std::endl;	
	}
	
	return EXIT_SUCCESS;
}
