/**
 * @class CommandLineOptions represents the class that parses and stores the command line options
 *        provided by the user to the program.
 * @author Luis Carlos Garcia-Peraza Herrera (luis.herrera.14@ucl.ac.uk).
 * @date   14 Mar 2015. 
 */

#include <iostream>
#include <string>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <exception>

// My includes
#include "commandlineoptions.h"

CommandLineOptions::CommandLineOptions() {

}

CommandLineOptions& CommandLineOptions::getInstance() {
	static CommandLineOptions instance; // Guaranteed to be destroyed and instantiated on first use.
	return instance;
}

std::string CommandLineOptions::getFixedPointsFilePath() const {
	return m_fixedPath;
}

std::string CommandLineOptions::getMovingPointsFilePath() const {
	return m_movingPath;
}

std::string CommandLineOptions::getOutputFilePath() const {
	return m_outputPath;
}

/**
 * @brief Reads and prints the command line arguments provided by the user. 
 * @param[in]  argc Number of command line arguments including the program.
 * @param[in]  argv Array of strings containing the parameters provided.
 * @returns true if the mandatory parameters are provided and they are correct (i.e. input files exist). 
 */
bool CommandLineOptions::processCmdLineOptions(int argc, char **argv) {
	bool retval = false;

	// Declare the supported options.
	boost::program_options::options_description desc("\nThis program registers two sets of points"
	                                                 " using the algorithm described in Arun et al."
																	 " (1987).\nThe parameters --fixed, --moving and "
																	 "--output are mandatory.\n\nOptions"
	                                                );
	desc.add_options()
		("help", "Prints this help message.")
		("fixed", boost::program_options::value<std::string>(), "Filename of the set of fixed points.")
		("moving", boost::program_options::value<std::string>(), "Filename of the set of moving points.")
		("output", boost::program_options::value<std::string>(), "Filename where the output homogeneous "
		                                                    "matrix will be saved.")
	;

	boost::program_options::variables_map vm;
	try {
		boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
		boost::program_options::notify(vm);    
	}
	catch(std::exception const &e) {
		std::cerr << "\nERROR! " << e.what() << std::endl << desc << std::endl;
		return false;
	}
	
	// Print help if the user asks for it
	if (vm.count("help")) {
		std::cout << desc << std::endl;
		retval = true;
	}
	
	// Check and print all mandatory options
	if (vm.count("fixed") && vm.count("moving") && vm.count("output")) {
		m_fixedPath = vm["fixed"].as<std::string>();
		m_movingPath = vm["moving"].as<std::string>();
		m_outputPath = vm["output"].as<std::string>();
		retval = true;
		/*	
		if (boost::filesystem::exists(m_fixedPath) &&
		    boost::filesystem::exists(m_movingPath)) {
			
			std::cout << "\nInput files:\n";
			std::cout << "   - First (fixed) set of points: " << m_fixedPath  << std::endl;
			std::cout << "   - Second (moving) set of points: " << m_movingPath << std::endl;
			std::cout << "\nOutput file:\n";
			std::cout << "   - Homogeneous matrix: " << m_outputPath << std::endl;
			retval = true;
		}
		else {
			std::cerr << "\nERROR! At least one of the files provided does not exist.\n" << desc << std::endl;
		}
		*/
	}
	else {
		std::cerr << "\nERROR! Mandatory arguments not provided.\n" << desc << std::endl;
	}

	return retval;
}
