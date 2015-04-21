/*
 * script.cpp
 *
 *  Created on: 16 mrt. 2015
 *      Author: RGrandia
 */

#include <Eigen/Dense>
#include "../lib/config/config_handler.h"

int main(int argc, char** argv)
{

	ConfigHandler conf;

	std::string mystring;
	int myint;
	Eigen::Matrix3f myMat;
	conf.printOptions();

	conf.getOptionMatrix("camera_parameters.extrinsics", myMat);
	std::cout << myMat << std::endl;

	conf.updateOption("settings.type",4);

  return 0;
}


