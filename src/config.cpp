#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <typeinfo>

// Boost
#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>

// Eigen
#include <Eigen/Dense>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/foreach.hpp>


using namespace std;



class ConfigHandler {

  public:

	ConfigHandler();
    void updateOptions();

    // parsing
    template<typename TYPE> bool parseMatrixf(const std::string& s, TYPE &matrix);

	// getters
	void printOptions();
	void save();


	// parse and return option
	template<typename TYPE> bool getOption(std::string opt_name, TYPE &var);
	template<typename TYPE> bool getOptionMatrix(std::string opt_name, TYPE &matrix);
	template<typename TYPE> void updateUption(std::string opt_name, TYPE value);

	private:

		void loadConfigFile();

		boost::property_tree::ptree options;


};

ConfigHandler::ConfigHandler(){
	loadConfigFile();
}


// parse and return option
template<typename TYPE> bool ConfigHandler::getOption(std::string opt_name, TYPE &var){
	var = options.get<TYPE>(opt_name);
}

template<typename TYPE> bool ConfigHandler::getOptionMatrix(std::string opt_name, TYPE &matrix){

	// get option as string, check if special signs included
	std::string stringvar = options.get<std::string>(opt_name);


	int cols = 0, rows = 0;

	// stored property does not represent a matrix
	std::size_t found = stringvar.find(",");
	if (found==std::string::npos){
		std::cout << "The selected configuration property does not represent a matrix" << std::endl;
		return false;
	}

	std::vector< std::string > buff;
	std::string delimiters(";");
	boost::split(buff, stringvar, boost::is_any_of(delimiters));

	// split rows
	rows = buff.size();

	delimiters = ",;";
	boost::split(buff, stringvar, boost::is_any_of(delimiters));

	cols = (int)buff.size()/rows;

    // populate matrix
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
        	// convert to float and save in matrix
        	matrix(i,j) = std::stod(buff[ cols*i+j ]);


    return true;

}

void ConfigHandler::save()
{

	write_ini( "config.ini", options );

}

template<typename TYPE> void ConfigHandler::updateUption(std::string opt_name, TYPE value){


	options.put(opt_name, value);
	save();

}


void ConfigHandler::loadConfigFile(){

    read_ini("config.ini", options);

}

void ConfigHandler::printOptions(){

    for (auto& section : options)
    {
        std::cout << '[' << section.first << "]\n";
        for (auto& key : section.second){
            std::cout << key.first << "=" << key.second.get_value<std::string>() << "\n";

        }
    }

}


int main(int argc, char** argv)
{

	ConfigHandler conf;

	std::string mystring;
	int myint;
	Eigen::Matrix4f myMat;
	conf.printOptions();

	conf.getOptionMatrix("camera_parameters.extrinsics", myMat);


	std::cout << myMat << std::endl;


	//conf.updateUption("settings",1337);


	conf.save();

  return 0;
}
