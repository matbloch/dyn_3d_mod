#include "TStree.hpp"
#include <igl/readOFF.h>
#include <igl/viewer/Viewer.h>
/*** insert any libigl headers here ***/
#include <igl/marching_cubes.h>
#include "igl/MCTables.hh"
#include <math.h>
#include "voxel_grid.h"

using namespace std;
using namespace cv;

typedef double ScalarType;
typedef unsigned IndexType;

static int MAX_DIM = 7;
int resolution = pow(2, MAX_DIM);

//intermediate result: grid points, at which the imlicit function will be evaluated, #G x3
//Eigen::MatrixXf grid_points(,3);
Eigen::Matrix<ScalarType, Eigen::Dynamic, 3> grid_points(resolution*resolution*resolution,3);
//intermediate result: implicit function values at the grid points, #G x1
//Eigen::VectorXf grid_values;
Eigen::Matrix<ScalarType, Eigen::Dynamic, 1> grid_values(resolution*resolution*resolution,1);
//intermediate result: grid point colors, for display, #G x3
Eigen::MatrixXd grid_colors;
//intermediate result: grid lines, for display, #L x6 (each row contains
//starting and ending point of line segment)
Eigen::MatrixXd grid_lines;
//output: vertex array, #V x3
//Eigen::MatrixXf V;
Eigen::Matrix<float, Eigen::Dynamic, 3>  V;
//output: face array, #F x3
//Eigen::MatrixXi F;
Eigen::Matrix<IndexType, Eigen::Dynamic, 3> F;
//output: face normals of the reconstructed mesh, #F x3
Eigen::MatrixXd FN;

// Pointer to the tweak bar
TwBar *mybar;

static int REF = 20;


static int GRID_SIZE = (int)pow(2,MAX_DIM);
static float spacing_in_m = 0.02;

float max_v[3] = {(float)(GRID_SIZE-1), (float)(GRID_SIZE-1), (float)(GRID_SIZE-1)};
float min_v[3] = {0, 0, 0};

TStree tstree((GRID_SIZE-1)*spacing_in_m, MAX_DIM);

//struct timeval tv;
//struct timezone tz;
double ibefore, iafter;


// Functions
void TW_CALL ResetGridCB(void *clientData);
void createGrid();
void getLines();
bool callback_init(igl::Viewer& viewer);
bool callback_key_down(igl::Viewer& viewer, unsigned char key, int modifiers);

void getCameraParameters(cv::Mat intrinsicMat);
void getCameraPose(cv::Mat R, cv::Mat tVec);
/* ========================================== *\
 * 		         Camera settings
\* ========================================== */

void getCameraParameters(cv::Mat intrinsicMat)
{
	intrinsicMat.at<float>(0, 0) = 589.3667;  // 640/2/tand(57/2)
	intrinsicMat.at<float>(1, 0) = 0;
	intrinsicMat.at<float>(2, 0) = 0;

	intrinsicMat.at<float>(0, 1) = 0;
	intrinsicMat.at<float>(1, 1) = 609.2755;  // 480/2/tand(43/2)
	intrinsicMat.at<float>(2, 1) = 0;

	intrinsicMat.at<float>(0, 2) = 319.5;  // 640/2
	intrinsicMat.at<float>(1, 2) = 239.5;  // 240/2
	intrinsicMat.at<float>(2, 2) = 1;
}

void getCameraPose(cv::Mat R, cv::Mat tVec)
{
	cv::Mat rVec(3, 1, cv::DataType<float>::type); // Rotation vector
	// Rotation vector in Rodrigues angles
	rVec.at<float>(0) = 0;
	rVec.at<float>(1) = 0;
	rVec.at<float>(2) = 0;
	Rodrigues(rVec, R);

	// Translation vector in Camera frame
	tVec.at<float>(0) = 0;
	tVec.at<float>(1) = 0;
	tVec.at<float>(2) = -5;
}

//Initialize tweakbar
bool callback_init(igl::Viewer& viewer)
{
//  int resolution = pow(2, MAX_DIM);

  mybar = TwNewBar("MarchingCubesDemo");
  // change default tweak bar size and color
  TwDefine(" MarchingCubesDemo size='200 250' color='76 76 127' position='230 16' fontresizable=true");
  // add parameter for tweaking
  TwAddVarRW(mybar, "resolution", TW_TYPE_UINT32, &resolution," min=2 step=1 ");
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

//  int resolution = pow(2, MAX_DIM);

  //diagonal
  Eigen::RowVector3d bb_max(max_v[0], max_v[1], max_v[2]);
  Eigen::RowVector3d bb_min(min_v[0], min_v[1], min_v[2]);
  Eigen::RowVector3d  diag = bb_max - bb_min;


  //steps in x,y,z direction
  double dx = diag[0] / (double)(resolution-1); Eigen::RowVector3d vx(dx,0.,0.);
  double dy = diag[1] / (double)(resolution-1); Eigen::RowVector3d vy(0.,dy,0.);
  double dz = diag[2] / (double)(resolution-1); Eigen::RowVector3d vz(0.,0.,dz);
  //3d positions of the grid points -- see slides or marching_cubes.h for ordering
  grid_points.resize(resolution*resolution*resolution,3);
  //iterate through the grid
  for (unsigned int x=0; x<resolution; ++x)
    for (unsigned int y=0; y<resolution; ++y)
      for (unsigned int z=0; z<resolution; ++z)
      {
        //linear index of the point at (x,y,z)
        int index = x + y*resolution + z*resolution*resolution;
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

//  int resolution = pow(2, MAX_DIM);

  int nnodes = grid_points.rows();
  grid_lines.resize(3*nnodes,6);
  int numLines = 0;
  for(int x=0;x<resolution;x++)
    for(int y=0;y<resolution;y++)
      for(int z=0;z<resolution;z++)
      {
        int index = x + y*resolution + z*resolution*resolution;
        if(x<resolution-1)
        {
          int index1 = (x+1) + y*resolution + z*resolution*resolution;
          grid_lines.row(numLines++) << grid_points.row(index),grid_points.row(index1);
        }
        if(y<resolution-1)
        {
          int index1 = x + (y+1)*resolution + z*resolution*resolution;
          grid_lines.row(numLines++) << grid_points.row(index),grid_points.row(index1);
        }
        if(z<resolution-1)
        {
          int index1 = x + y*resolution + (z+1)*resolution*resolution;
          grid_lines.row(numLines++) << grid_points.row(index),grid_points.row(index1);
        }
      }
  grid_lines.conservativeResize(numLines,Eigen::NoChange);
}

bool callback_key_down(igl::Viewer& viewer, unsigned char key, int modifiers)
{
  if (key == '0')
  {
    voxelGrid grid;

    // Define intrinsic camera parameters
    cv::Mat intrinsicMat(3, 3, CV_32F); // intrinsic matrix
    getCameraParameters(intrinsicMat);

    // Define Camera orientation and position
    cv::Mat R(3, 3, CV_32F);
    cv::Mat tVec(3, 1, CV_32F); // Translation vector in camera frame
    getCameraPose(R,tVec);

    // Set Voxel parameters
    int sz[3] = {GRID_SIZE,GRID_SIZE,GRID_SIZE};
    grid.setParameters(GRID_SIZE, spacing_in_m, intrinsicMat, R, tVec);

    int num = 5;
    for(int time=0; time<num; time++){
      cv::Mat kinectimage(480, 640, CV_32F, Scalar(5.1-time/50.0));
      Mat FilledVoxels(3,sz, CV_32FC1, Scalar::all(0));

      grid.fillVoxels(kinectimage, FilledVoxels);



      for(int i=0; i<GRID_SIZE; i++){
        for(int j=0; j<GRID_SIZE; j++){
          for(int k=0; k<GRID_SIZE; k++){
            if(isnan(FilledVoxels.at<float>(i,j,k))){
              FilledVoxels.at<float>(i,j,k) = 1;
              //cout << "nan ";
            }
          }
        }
      }

      for(int i=0; i<3; i++){
        max_v[i] = (grid.units).at(GRID_SIZE-1);
        min_v[i] = (grid.units).at(0);
      }

      tstree.insert(FilledVoxels, time);
    }

    cout << "insert finish" << endl;
    //tstree.print_timespacetree();

  }
  if (key == '1')
  {
    //show imported points
    viewer.data.clear();
    //viewer.core.align_camera_center(P);
    viewer.core.point_size = 11;
    //viewer.data.add_points(P,Eigen::RowVector3d(0,0,0));
  }

  if (key == '2')
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
      else if(value>0){
        grid_colors(i,0) = 1;
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

  if (key == '3')
  {
    // show reconstructed mesh
    viewer.data.clear();
    //std::cout << grid_points << std::endl;
    std::cout << "point" << std::endl;
    //std::cout << grid_values << std::endl;
//    int resolution = pow(2, MAX_DIM);
    // Code for computing the mesh (V,F) from grid_points and grid_values
    if (grid_points.rows()==0 || grid_values.rows() == 0)
    {
      cerr<<"Not enough data for Marching Cubes !"<<endl;
      return true;
    }
    // run Marching Cubes

  Eigen::Matrix<float, Eigen::Dynamic, 3> points = grid_points.cast<float>();
  Eigen::Matrix<float, Eigen::Dynamic, 1> values = grid_values.cast<float>();
//  Eigen::Matrix<ScalarType, Eigen::Dynamic, 3> vertices;
//

//  Eigen::Matrix<IndexType, Eigen::Dynamic, 3> faces;
    igl::marching_cubes(values,points, resolution, resolution, resolution, V, F);
//    igl::marching_cubes(grid_values.cast<float>(),grid_points.cast<float>(), resolution, resolution, resolution, V, F);
    if (V.rows()==0)
    {
      cerr<<"Marching Cubes failed!"<<endl;
      return true;
    }
//    igl::per_face_normals(V,F,FN);
    Eigen::MatrixXd Vertex = V.cast<double>();
    Eigen::MatrixXi Faces = F.cast<int>();
    viewer.data.set_mesh(Vertex, Faces);
    viewer.core.show_lines = true;
    viewer.core.show_faces = true;
//    viewer.data.set_normals(FN);
  }
  if (key == '4'){
    grid_values = tstree.read(0);
    //std::cout << grid_values << std::endl;
    std::cout << grid_values.size() << std::endl;
    createGrid();
    callback_key_down(viewer, '3', 0);
    cout << "read finish" << endl;
  }
  if (key == '5'){
    tstree.update(&grid_values);
    callback_key_down(viewer, '3', 0);
    cout << "update finish" << endl;
  }

  return true;
}

int main(int, char **)
{
  //voxel width

  igl::Viewer viewer;
  viewer.callback_key_down = callback_key_down;
  viewer.callback_init = callback_init;
  callback_key_down(viewer, '1', 0);
  viewer.launch();
}
