#include <iostream>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <atomic>

// Eigen
#include <Eigen/Dense>

// OpenCV includes
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

// our own libraries
#include <config/config_handler.h>
#include <cv/filters.h>
#include "cv/voxel_grid.h"
#include "misc/colio.h"
#include "definitions.h"

// time space tree library
#include "tree/TStree.hpp"


// IGL includes
#include <igl/readOFF.h>
#include <igl/viewer/Viewer.h>
#include <igl/marching_cubes.h>
#include "igl/MCTables.hh"

// grid status
static int MAX_DIM = 8;
static int GRID_SIZE = (int)pow(2,MAX_DIM);
static float spacing_in_m = 0.008;
float max_v[3] = {(float)(GRID_SIZE-1), (float)(GRID_SIZE-1), (float)(GRID_SIZE-1)};
float min_v[3] = {0, 0, 0};

// object handlers
ConfigHandler conf;
ImageFilters filters;
voxelGrid grid1;
voxelGrid grid2;
igl::Viewer viewer;

// globals
cv::Mat FilledVoxels1;
cv::Mat FilledVoxels2;
cv::Mat FusedVoxels;

// Time Space Tree
TStree tstree((GRID_SIZE-1)*spacing_in_m, MAX_DIM);
int t = 0;

//intermediate result: grid points, at which the imlicit function will be evaluated, #G x3
Eigen::MatrixXd grid_points;
//intermediate result: implicit function values at the grid points, #G x1
Eigen::VectorXd grid_values;
Eigen::VectorXd grid_values2(GRID_SIZE*GRID_SIZE*GRID_SIZE);
//intermediate result: grid point colors, for display, #G x3
Eigen::MatrixXd grid_colors;
//intermediate result: grid lines, for display, #L x6 (each row contains
//starting and ending point of line segment)
Eigen::MatrixXd grid_lines;
//output: vertex array, #V x3
Eigen::MatrixXd V;
//output: face array, #F x3
Eigen::MatrixXi F;
//output: face normals of the reconstructed mesh, #F x3
Eigen::MatrixXd FN;
// Pointer to the tweak bar
TwBar *mybar;

using namespace cv;

// Visualization Functions
void TW_CALL ResetGridCB(void *clientData);
void createGrid();
void getLines();
bool callback_init(igl::Viewer& viewer);
bool callback_key_down(igl::Viewer& viewer, unsigned char key, int modifiers);
void visualize_mesh();

// Grid initialization Functions
void getCameraPoses(cv::Mat R1, cv::Mat tVec1, cv::Mat R2, cv::Mat tVec2);
cv::Mat getIntrinsics();
void InitializeVoxelGrids();

// Fuse function
void FuseVoxels(cv::Mat Filled1,cv::Mat Filled2,cv::Mat Fused);

int main(int argc, char** argv)
{
	cv::Mat image1;
	cv::Mat image2;
	cv::Mat filtered1;
	cv::Mat filtered2;

	cv::FileStorage storage("test/ml_recording2_img1.yml", cv::FileStorage::READ);
	storage["img"] >> image1;
	storage.release();
	cv::FileStorage storage2("test/ml_recording2_img2.yml", cv::FileStorage::READ);
	storage2["img"] >> image2;
	storage2.release();


	filters.bilateral(image1, filtered1);
	filters.bilateral(image2, filtered2);

	filtered1 *= 0.001;
	filtered2 *= 0.001;

	/* ========================================== *\
	 * 		1. Setup voxel struture
	\* ========================================== */

	InitializeVoxelGrids();

	std::cout<<"Voxels initialized"<<std::endl;

	/* ========================================== *\
	 * 		2. Voxel grid
	\* ========================================== */

	grid1.fillVoxels(filtered1, FilledVoxels1);
	grid2.fillVoxels(filtered2, FilledVoxels2);

	std::cout<<"Voxels Filled"<<std::endl;

	/* ========================================== *\
	 * 		3. Fusion
	\* ========================================== */

	FuseVoxels(FilledVoxels1,FilledVoxels2,FusedVoxels);

	std::cout<<"Voxels Fused"<<std::endl;

	/* ========================================== *\
	 * 		4. Octree integration
	\* ========================================== */

	for(int i=0; i<3; i++){
		max_v[i] = (grid1.units).at(GRID_SIZE-1);
		min_v[i] = (grid1.units).at(0);
	}
	tstree.insert(FusedVoxels, t);
	t++;
	//tstree.print_timespacetree();

	std::cout<<"Tree insert done"<<std::endl;

	viewer.callback_key_down = callback_key_down;
	viewer.callback_init = callback_init;
	callback_key_down(viewer, '1', 0);
	viewer.launch();

	std::cout<<"Press 2 to visualize"<<std::endl;

	cv::waitKey(0);

}

// -------------------
// External functions
// -------------------


//Initialize tweakbar
bool callback_init(igl::Viewer& viewer)
{

	mybar = TwNewBar("MarchingCubesDemo");
	// change default tweak bar size and color
	TwDefine(" MarchingCubesDemo size='200 250' color='76 76 127' position='230 16' fontresizable=true");
	// add parameter for tweaking
	TwAddVarRW(mybar, "resolution", TW_TYPE_UINT32, &GRID_SIZE," min=2 step=1 ");
	// add button with callback function
	TwAddButton(mybar,"ResetGrid", ResetGridCB, &viewer, " ");
	TwAddSeparator(mybar, "", "");
	return false;
}

void TW_CALL ResetGridCB(void *clientData)
{
	//  retrieve the viewer from the void*
	igl::Viewer *viewer = static_cast<igl::Viewer *>(clientData);
	//  recreate the grid
	createGrid();
	//  switch the view to show the grid
	callback_key_down(*viewer, '3', 0);
}

//Creates a grid_points array for the simple sphere example. The points are
//stacked up into a single matrix, ordered first in the x, then in the y
//and then in the z direction.
//Replace this with your own function for creating the grid, if need be
void createGrid()
{
	grid_points.resize(0,3);
	//grid_values.resize(0,1);
	grid_colors.resize(0,3);
	grid_lines.resize(0,6);
	V.resize(0,3);
	F.resize(0,3);
	FN.resize(0,3);


	//diagonal
	Eigen::RowVector3d bb_max(max_v[0], max_v[1], max_v[2]);
	Eigen::RowVector3d bb_min(min_v[0], min_v[1], min_v[2]);
	Eigen::RowVector3d  diag = bb_max - bb_min;


	//steps in x,y,z direction
	double dx = diag[0] / (double)(GRID_SIZE-1); Eigen::RowVector3d vx(dx,0.,0.);
	double dy = diag[1] / (double)(GRID_SIZE-1); Eigen::RowVector3d vy(0.,dy,0.);
	double dz = diag[2] / (double)(GRID_SIZE-1); Eigen::RowVector3d vz(0.,0.,dz);
	//3d positions of the grid points -- see slides or marching_cubes.h for ordering
	grid_points.resize(GRID_SIZE*GRID_SIZE*GRID_SIZE,3);
	//iterate through the grid
	for (unsigned int x=0; x<GRID_SIZE; ++x)
		for (unsigned int y=0; y<GRID_SIZE; ++y)
			for (unsigned int z=0; z<GRID_SIZE; ++z)
			{
				//linear index of the point at (x,y,z)
				int index = x + y*GRID_SIZE + z*GRID_SIZE*GRID_SIZE;
				//3D point at (x,y,z)
				grid_points.row(index) = bb_min + x*vx +y*vy +z*vz;
				//grid_points.row(index) = x*vx +y*vy +z*vz;
			}
}



// Code to display the grid lines given a grid structure of the given form.
// Assumes grid_points have been correctly assigned
// Replace with your own code for displaying lines if need be.
void getLines()
{
	int nnodes = grid_points.rows();
	grid_lines.resize(3*nnodes,6);
	int numLines = 0;
	for(int x=0;x<GRID_SIZE;x++)
		for(int y=0;y<GRID_SIZE;y++)
			for(int z=0;z<GRID_SIZE;z++)
			{
				int index = x + y*GRID_SIZE + z*GRID_SIZE*GRID_SIZE;
				if(x<GRID_SIZE-1)
				{
					int index1 = (x+1) + y*GRID_SIZE + z*GRID_SIZE*GRID_SIZE;
					grid_lines.row(numLines++) << grid_points.row(index),grid_points.row(index1);
				}
				if(y<GRID_SIZE-1)
				{
					int index1 = x + (y+1)*GRID_SIZE + z*GRID_SIZE*GRID_SIZE;
					grid_lines.row(numLines++) << grid_points.row(index),grid_points.row(index1);
				}
				if(z<GRID_SIZE-1)
				{
					int index1 = x + y*GRID_SIZE + (z+1)*GRID_SIZE*GRID_SIZE;
					grid_lines.row(numLines++) << grid_points.row(index),grid_points.row(index1);
				}
			}
	grid_lines.conservativeResize(numLines,Eigen::NoChange);
}

bool callback_key_down(igl::Viewer& viewer, unsigned char key, int modifiers)
{
	if (key == '1')
	{
		//show imported points
		viewer.data.clear();
		//viewer.core.align_camera_center(P);
		viewer.core.point_size = 11;
		//viewer.data.add_points(P,Eigen::RowVector3d(0,0,0));
	}
	if (key == '2'){
		grid_values = tstree.read(0);
		createGrid();

		visualize_mesh();
		cout << "read finish" << endl;
	}
	if (key == '3'){
		tstree.update(&grid_values);
		visualize_mesh();
		cout << "update finish" << endl;
	}
	if (key == '4')
	{
		//show grid points with colored nodes and connected with lines
		viewer.data.clear();
		// make grid
		createGrid();
		// get grid lines
		getLines();
		// Code for coloring and displaying the grid points and lines
		// Assumes that grid_values and grid_points have been correctly assigned.
		grid_colors.setZero(grid_points.rows(),3);
		// build up color map
		for(int i=0;i<grid_points.rows();i++)
		{
			double value = grid_values(i);
			//cout << value << endl;
			//
			if(value<0){
				grid_colors(i,1) = 1;
			}
			else{
				grid_colors(i,0) = 1;
			}
		}
		//draw lines and points
		viewer.core.point_size = 8;
		viewer.data.add_points( grid_points, grid_colors);
		viewer.data.add_edges ( grid_lines.block(0,0,grid_lines.rows(),3), grid_lines.block(0,3,grid_lines.rows(),3), Eigen::RowVector3d(0.8,0.8,0.8)   );
		/*** end: sphere example***/
	}


	return true;
}

void visualize_mesh(){
	// show reconstructed mesh
	viewer.data.clear();

	// Code for computing the mesh (V,F) from grid_points and grid_values
	if (grid_points.rows()==0 || grid_values.rows() == 0)
	{
		cerr<<"Not enough data for Marching Cubes !"<<endl;
		return;
	}
	// run Marching Cubes
	igl::marching_cubes(grid_values,grid_points, GRID_SIZE, GRID_SIZE, GRID_SIZE, V, F);
	if (V.rows()==0)
	{
		cerr<<"Marching Cubes failed!"<<endl;
		return;
	}
	igl::per_face_normals(V,F,FN);
	viewer.data.set_mesh(V, F);
	viewer.core.show_lines = true;
	viewer.core.show_faces = true;
	viewer.data.set_normals(FN);

}

void getCameraPoses(cv::Mat R1, cv::Mat tVec1, cv::Mat R2, cv::Mat tVec2){
	// Set up the voxel grid position for camera 1
//	cv::Mat rVec(3, 1, cv::DataType<float>::type); // Rotation vector
//	// Rotation vector in Rodrigues angles
//	rVec.at<float>(0) = 0;
//	rVec.at<float>(1) = 0;
//	rVec.at<float>(2) = 0;
//	Rodrigues(rVec, R1);

	R1 = Mat::zeros(3,3,CV_32F);
//		R1.at<float>(0,0) = 1;
//		R1.at<float>(1,1) = -1;
//		R1.at<float>(2,2) = -1;
//	R1.at<float>(0,1) = 1;
//	R1.at<float>(1,2) = -1;
//	R1.at<float>(2,0) = -1;
	R1.at<float>(0,0) = 0.8434;
	R1.at<float>(0,2) = 0.5373;
	R1.at<float>(1,1) = 1;
	R1.at<float>(2,0) = -0.5373;
	R1.at<float>(2,2) = 0.8434;

	// Translation vector in Camera frame
	tVec1.at<float>(0) = 0;
	tVec1.at<float>(1) = 0;
	tVec1.at<float>(2) = 2.5;

	// get extrinsics
	Eigen::Matrix4f extrinsics;
	conf.getOptionMatrix("camera_parameters.extrinsics", extrinsics);
	cv::Mat extrinsics_mat(4,4, CV_32F);
	cv::eigen2cv(extrinsics, extrinsics_mat);
	cv::Mat R_ext = extrinsics_mat(cv::Range(0,3), cv::Range(0,3));
	cv::Mat t_ext = extrinsics_mat(cv::Range(0,3), cv::Range(3,4));

	// Set up the voxel grid objects for both cameras
	R2 = R_ext*R1;
	tVec2 = t_ext + R_ext*tVec1;

	std::cout << "Rext: " << R_ext << std::endl;
	std::cout << "text: " << t_ext << std::endl;

//	R2 = R_ext.t()*R1;
//	tVec2 = R_ext.t()*(tVec1-t_ext);
}

cv::Mat getIntrinsics(){
	cv::Mat intrinsicMat(3, 3, CV_32F); // intrinsic matrix
	Eigen::Matrix3f intrinsics_eig;
	conf.getOptionMatrix("camera_parameters.intrinsics", intrinsics_eig);
	cv::eigen2cv(intrinsics_eig, intrinsicMat);
	return intrinsicMat;

}

void InitializeVoxelGrids(){
	// Configure rotations and translations
	cv::Mat R1(3, 3, CV_32F);    // Rotation vector
	cv::Mat tVec1(3, 1, CV_32F); // Translation vector in camera frame
	cv::Mat R2(3, 3, CV_32F);    // Rotation vector
	cv::Mat tVec2(3, 1, CV_32F); // Translation vector in camera frame
	getCameraPoses(R1,tVec1,R2,tVec2);


	std::cout << "R1: " << R1 << std::endl;
	std::cout << "R2: " << R2 << std::endl;
	std::cout << "t1: " << tVec1 << std::endl;
	std::cout << "t2: " << tVec2 << std::endl;



	// get intrinsics
	cv::Mat intrinsicMat = getIntrinsics();

	grid1.setParameters(GRID_SIZE, spacing_in_m, intrinsicMat, R1, tVec1);
	grid2.setParameters(GRID_SIZE, spacing_in_m, intrinsicMat, R2, tVec2);

	// Setup voxel structure to load the TSDF in
	int sz[3] = {GRID_SIZE,GRID_SIZE,GRID_SIZE};
	FilledVoxels1 = Mat(3,sz, CV_32FC1, Scalar::all(0));
	FilledVoxels2 = Mat(3,sz, CV_32FC1, Scalar::all(0));
	FusedVoxels = Mat(3,sz, CV_32FC1, Scalar::all(0));
}

void FuseVoxels(cv::Mat Filled1,cv::Mat Filled2,cv::Mat Fused){
	for (int i=0; i<GRID_SIZE; i++)
			for (int j=0; j<GRID_SIZE; j++)
				for (int k=0; k<GRID_SIZE; k++){
					if (isnan(Filled1.at<float>(i,j,k))){
						if (isnan(Filled2.at<float>(i,j,k))){
							Fused.at<float>(i,j,k) = 1.0;
						}
						else{
							Fused.at<float>(i,j,k) = Filled2.at<float>(i,j,k);
						}
					}
					else{
						if (isnan(Filled2.at<float>(i,j,k))){
							Fused.at<float>(i,j,k) = Filled1.at<float>(i,j,k);
						}
						else{
							Fused.at<float>(i,j,k) = 0.5*(Filled1.at<float>(i,j,k)+Filled2.at<float>(i,j,k));
						}
					}
				}
}
