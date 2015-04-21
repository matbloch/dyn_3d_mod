/*
 * script.cpp
 *
 *  Created on: 16 mrt. 2015
 *      Author: RGrandia
 */

#include <Eigen/Dense>
#include "definitions.h"
#include "config/config_handler.h"

	ConfigHandler conf;

int main(int argc, char** argv)
{

	float approx_leaf_size;
	float normal_est_search_radius;
	float icp_max_correlation_dist;
	float shot_search_radius;

	conf.printOptions();

	conf.getOption("extrinsics_calibration.approx_leaf_size",approx_leaf_size);
	conf.getOption("extrinsics_calibration.normal_est_search_radius",normal_est_search_radius);
	conf.getOption("extrinsics_calibration.icp_max_correlation_dist",icp_max_correlation_dist);
	conf.getOption("extrinsics_calibration.shot_search_radius",shot_search_radius);

  return 0;
}


