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
    Eigen::MatrixXf parseMatrixf(const std::string& s);

	// getters
	void printOptions();
	void save();
	bool populate_tree(boost::program_options::variables_map& vm, boost::property_tree::ptree& tree);

	// parse and return option
	template<typename TYPE> bool getOption(std::string opt_name, TYPE &var){


		/*
		 * TODO: handle custom parsing (matrices)
		 */

		// check if matrix
		if ( boost::is_same<TYPE, Eigen::Matrix4f>::value){
			//var = options[opt_name].as< int >();
			std::cout << "BOOJAH, matrix!" << std::endl;
			return true;
		}


		try{
			// else parse and return
			var = options[opt_name].as< TYPE >();
		}catch(std::exception const&  ex)
		{
			std::cout << "Error parsing config file. Setting variable type \"" << typeid(var).name() << "\" does not match the defined property type." << std::endl;
		}

		return true;

	}

	private:

		void loadConfigFile();

		boost::program_options::variables_map options;


};

ConfigHandler::ConfigHandler(){
	loadConfigFile();
}


bool ConfigHandler::populate_tree(boost::program_options::variables_map& vm, boost::property_tree::ptree& tree)
{
    boost::program_options::notify(vm);
    boost::property_tree::ptree& root = tree.add("root", "");
    boost::property_tree::ptree empty_root = root;
    BOOST_FOREACH(boost::program_options::variables_map::value_type& i, vm) {
        boost::any value = i.second.value();
        try {
            root.add(i.first, boost::any_cast<std::string>(value));
        }
        catch (const boost::bad_any_cast& e) {
            root.add(i.first, boost::any_cast<int>(value));
        }
    }
    return empty_root != root; // tree not empty
}


void ConfigHandler::save()
{

	boost::property_tree::ptree tree;

	const bool filled = populate_tree(options, tree);
	write_ini( "config2.ini", tree );

}


void ConfigHandler::loadConfigFile(){

	namespace po = boost::program_options;

	// Setup options
	po::options_description desc("Options");

	// store in variable map
	desc.add_options()
	("plugins.name", po::value< std::vector< std::string > >()->multitoken(),"plugin names" )
	("camera_parameters.extrinsics", po::value< std::string >(),"Camera Extrinsics" )	// store matrix as string
	("some_cat.string_prop", po::value< std::string >(),"tests" )
	("settings.type", po::value< int >(),"settings_type" );

	// Load setting file.
	std::ifstream settings_file( "config.ini");
	po::store( po::parse_config_file( settings_file , desc ), options );
	settings_file.close();
	po::notify( options );

}

void ConfigHandler::printOptions(){

	/*

	//---------------------------------------- Print settings.

	typedef std::vector< std::string >::iterator iterator;
	for ( multiple_values::iterator iterator = plugin_names.begin(),
								  end = plugin_names.end();
	iterator < end;
	++iterator )
	{
	std::cout << "plugin.name: " << *iterator << std::endl;
	}

	*/

}

/* ========================================== *\
 * 		PARSING
\* ========================================== */

Eigen::MatrixXf ConfigHandler::parseMatrixf(const std::string& s) {

	int cols = 0, rows = 0;

	std::vector< std::string > buff;
	std::string delimiters(";");
	boost::split(buff, s, boost::is_any_of(delimiters));

	// split rows
	rows = buff.size();

	delimiters = ",;";
	boost::split(buff, s, boost::is_any_of(delimiters));

	cols = (int)buff.size()/rows;

    // populate matrix
	Eigen::MatrixXf result(rows,cols);
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
        	// convert to float and save in matrix
            result(i,j) = std::stod(buff[ cols*i+j ]);

    return result;

}


int main()
{

	ConfigHandler conf;

	std::string mystring;
	int myint;
	Eigen::Matrix4f myMat;

	//conf.getOption("settings.type", mystring);
	conf.save();


  return 0;
}
