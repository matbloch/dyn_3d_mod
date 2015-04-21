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


#include <sstream>

using namespace std;


template <typename T> string toString(const T& t) {
   ostringstream os;
   os<<t;
   return os.str();
}

class ConfigHandler {

	std::string config_path;

  public:

		ConfigHandler(std::string path);

		void printOptions();	// print the options

		// parse and return option
		template<typename TYPE> bool getOption(std::string opt_name, TYPE &var);
		template<typename TYPE> bool getOptionMatrix(std::string opt_name, TYPE &matrix);
		template<typename TYPE> bool updateOption(std::string opt_name, TYPE value);
		template<typename MTYPE> bool updateOptionMatrix(std::string opt_name, MTYPE matrix);

	private:

		boost::property_tree::ptree options;

		void loadConfigFile();
		void save();	// save current pt back to .ini file

	    // parsing
	    template<typename TYPE> bool parseMatrixf(const std::string& s, TYPE &matrix);

};

ConfigHandler::ConfigHandler(std::string path){
	config_path = path;
	loadConfigFile();
}

template<typename TYPE> bool ConfigHandler::getOption(std::string opt_name, TYPE &var){

	try{
		var = options.get<TYPE>(opt_name);
		return true;
	}catch(boost::property_tree::ptree_error &e){
        cout << "Property does not exist: " << e.what() << endl;
	}

	return false;

}

template<typename TYPE> bool ConfigHandler::getOptionMatrix(std::string opt_name, TYPE &matrix){

	std::string stringvar;
	try{
		// get option as string, check if special signs included
		stringvar = options.get<std::string>(opt_name);
	}catch(boost::property_tree::ptree_error &e){
        cout << "Property does not exist: " << e.what() << endl;
        return false;
	}

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

	if(matrix.rows() != rows || matrix.cols() != cols){
		std::cout << "Loading matrix from config file failed. Matrices do not have the same dimensions" << std::endl;
		return false;
	}

    // populate matrix
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
        	// convert to float and save in matrix
        	matrix(i,j) = std::stod(buff[ cols*i+j ]);

    return true;

}

void ConfigHandler::save()
{

	write_ini(config_path, options );

}

template<typename TYPE> bool ConfigHandler::updateOption(std::string opt_name, TYPE value){

	// check if option exists
	try{
		options.get<string>(opt_name);
		options.put(opt_name, value);
		save();
		return true;
	}catch(boost::property_tree::ptree_error &e){
        cout << "Property does not exist: " << e.what() << endl;
	}

	return false;

}

template<typename MTYPE> bool ConfigHandler::updateOptionMatrix(std::string opt_name, MTYPE matrix){

	// check if option exists
	try{
		options.get<string>(opt_name);

		std::string matcat;

		for(int i=0;i<matrix.rows();i++){
			for(int j=0;j<matrix.cols();j++){

				// add value
				matcat.append(toString(matrix(i,j)));

			    if(j==matrix.cols()-1){
			    	if(i!=matrix.rows()-1){
			    		matcat.append(";");
			    	}
				}else{
					matcat.append(",");
				}

			}
		}

		options.put(opt_name, matcat);
		save();
		return true;

	}catch(boost::property_tree::ptree_error &e){
        cout << "Property does not exist: " << e.what() << endl;
	}

	return false;

}

void ConfigHandler::loadConfigFile(){

    try
    {
    	read_ini(config_path, options);
    }catch(boost::property_tree::ptree_error &e){
        cout << e.what() << endl;
    }

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
