/**
 * @class CommandLineOptions represents the class that parses and stores the command line options
 *        provided by the user to the program.
 * @author Luis Carlos Garcia-Peraza Herrera (luis.herrera.14@ucl.ac.uk).
 * @date   14 Mar 2015. 
 */

#ifndef __COMMAND_LINE_OPTIONS_H__
#define __COMMAND_LINE_OPTIONS_H__

#include <string>

class CommandLineOptions {

public:
	// Singleton: only one command line
	static CommandLineOptions& getInstance();

	// Getters and setters
	std::string getFixedPointsFilePath() const;
	std::string getMovingPointsFilePath() const;
	std::string getOutputFilePath() const;
	bool processCmdLineOptions(int argc, char **argv);
	
private:
	CommandLineOptions();
	CommandLineOptions(CommandLineOptions const&) = delete;
	void operator = (CommandLineOptions const&) = delete;

	std::string m_fixedPath;
	std::string m_movingPath;
	std::string m_outputPath;
};

#endif
