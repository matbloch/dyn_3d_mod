#include <iostream>
#include <cmath>
#include "tree.hh"
#include "tree_util.hh"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;

#define EMP 100

class TStree{

  public:
    //TStree(float _max_v[3], float _min_v[3], int _max_d){}

    TStree(float _width, float _max_v[3], float _min_v[3], int _max_d) : MAX_DIM(_max_d), WIDTH(_width),  GRID_SIZE(pow(2,_max_d)){

      init_tstree();
      for(int i=0; i<3; i++){
        MAX_V[i] = _max_v[i];
        MIN_V[i] = _min_v[i];
      }
      //MAX_DIM = _max_d;
      //WIDTH = _width;
      //GRID_SIZE = pow(2,MAX_DIM);
    }
    class ValTime{

      public:
        ValTime(){}
        ValTime(int _val, char _time){
          val = _val;
          time = _time;
        }
        float val;
        char time;
        friend ostream& operator<<(ostream& os, const ValTime& it);
    };

    void insert(cv::Mat voxel_values, char time);
    void insert(float*** p, char time);
    Eigen::VectorXd read(char time);
    void print_timespacetree();

  private:
    tree<vector<ValTime> > tstree;
    tree<vector<ValTime> >::sibling_iterator main_root;
    float MAX_V[3];
    float MIN_V[3];
    const int MAX_DIM;
    const int WIDTH;
    const int GRID_SIZE;

    void init_tstree();
    tree<float> init_tree(int max_d);
    void make_trees(tree<float>* tr, tree<float>::sibling_iterator current, float width, float c_x, float c_y, float c_z, float x, float y, float z, int d, int max_d, float t);
    tree<float> make_octree(float voxel_width, float max_x, float min_x, float max_y, float min_y, float max_z, float min_z, int max_d, float ***t);
    void merge_trees(tree<vector<ValTime> >* main_tr, tree<vector<ValTime> >::sibling_iterator main_cur,tree<float>* new_tr, tree<float>::sibling_iterator new_cur, int time);
    void read_tree(tree<vector<ValTime> >* tr, tree<vector<ValTime> >::sibling_iterator current, vector<int>* index, int t, float* p);
    void read_tree(tree<vector<ValTime> >* tr, tree<vector<ValTime> >::sibling_iterator current, vector<int>* index, int t, Eigen::VectorXd* p);
    bool all_same(tree<float> tr, tree<float>::sibling_iterator current, int t);
    bool has_value(tree<vector<ValTime> >::sibling_iterator main_cur);
    bool has_value(tree<float>::sibling_iterator current);
    void insertToVoxelPos(vector<int> index, int val, float* p);
    void insertToVoxelPos(vector<int> index, int val, Eigen::VectorXd* p);
    void convertTreeIndextoVoxelPos(vector<ValTime> t_tr, vector<int> index, int t, float* p);
    void convertTreeIndextoVoxelPos(vector<ValTime> t_tr, vector<int> index, int t, Eigen::VectorXd* p);
};

ostream& operator<<(ostream& os, const TStree::ValTime& it)
{
  os << '(' << it.val << ',' << (int)(it.time) << ')';
  return os;
}

void TStree::insert(cv::Mat voxel_values, char time){

  float*** p;
  p = (float ***)malloc(sizeof(float **) * GRID_SIZE);
  for (int i=0;i<GRID_SIZE;i++) {
    p[i] = (float **)malloc(sizeof(float *) * GRID_SIZE);
    for (int j=0;j<GRID_SIZE;j++) {
      p[i][j] = (float *)malloc(sizeof(float) * GRID_SIZE);
    }
  }

  for(int x=0; x<GRID_SIZE; x++){
    for(int y=0; y<GRID_SIZE; y++){
      for(int z=0; z<GRID_SIZE; z++){
        p[x][y][z] = voxel_values.at<float>(x,y,z);
      }
    }
  }

  //main_root = tstree.begin();
  tree<float> tr = make_octree(WIDTH, MAX_V[0], MIN_V[0], MAX_V[1], MIN_V[1], MAX_V[2], MIN_V[2], MAX_DIM, p);
  tree<float>::sibling_iterator root;
  root = tr.begin();
  merge_trees(&tstree, main_root, &tr, root, 0);
}

void TStree::insert(float*** p, char time){


  //main_root = tstree.begin();
  tree<float> tr = make_octree(WIDTH, MAX_V[0], MIN_V[0], MAX_V[1], MIN_V[1], MAX_V[2], MIN_V[2], MAX_DIM, p);
  tree<float>::sibling_iterator root;
  root = tr.begin();
  merge_trees(&tstree, main_root, &tr, root, time);
}

Eigen::VectorXd TStree::read(char time){

  tree<vector<ValTime> >::sibling_iterator head = tstree.begin();
  vector<int> index;
  index.clear();
  //int signed_distance[(int)pow(2,MAX_DIM*3)];
  Eigen::VectorXd signed_distance((int)pow(2,MAX_DIM*3));
  read_tree(&tstree, head, &index, time, &signed_distance);

  return signed_distance;
}


/*
 * Print all the leaf nodes of time-space tree
 *
 * tree<vector<ValTime> > tr: time-space tree
 */
void TStree::print_timespacetree(){

  tree<vector<ValTime> >::leaf_iterator leaf_it;
  int node_num = 0;
  for(leaf_it = tstree.begin_leaf(); leaf_it != tstree.end_leaf(); leaf_it++){
    for(int i=0; i<(*leaf_it).size(); i++){
      cout << "(" << node_num << ","  << i << ") " << (*leaf_it)[i].val << "," << (*leaf_it)[i].time << endl;
    }
    node_num++;
  }
}

/*
 * Initialize time-space tree
 *
 */
void TStree::init_tstree(){

  vector<ValTime> initvec;
  ValTime* initit = new ValTime(EMP, EMP);
  initvec.push_back(*initit);
  main_root = tstree.begin();
  main_root = tstree.insert(main_root, initvec); 
}

/*
 * Check if all neighbors of the node are same
 *
 * tree<float> tr: octree
 * tree<float>::sibling_iterator current: node of the octree
 * int t: signed distance value
 */
bool TStree::all_same(tree<float> tr, tree<float>::sibling_iterator current, int t){

  bool ans = true;
  tree<float>::sibling_iterator neighbor;
  neighbor = (tr.parent(current)).begin();
  do{
    if(t != *neighbor && current != neighbor){
      ans = false;
      break;
    }
  } while((tr.parent(neighbor)).end() != ++neighbor);
  return ans;
}

/*
 * Build octree
 *
 * tree<float>* tr: octree
 * tree<float>* tr: root of octree
 * float width: half of the voxel width
 * float c_x: x value of the center coordinate of the voxel
 * float c_y: y value of the center coordinate of the voxel
 * float c_z: z value of the center coordinate of the voxel
 * float x: x value of the coordinate of the point
 * float y: y value of the coordinate of the point
 * float z: z value of the coordinate of the point
 * int d: current dimension of the octree
 * int max_d: maximum dimension of the octree
 * float t: signed distance value
 */
void TStree::make_trees(tree<float>* tr, tree<float>::sibling_iterator current, float width, float c_x, float c_y, float c_z, float x, float y, float z, int d, int max_d, float t){

  if(d < max_d){
    tree<float>::sibling_iterator next;
    next = (*tr).begin(current);
    if((c_x >= x && c_y >= y) && c_z >= z){
      make_trees(tr, next, width/2, c_x - width/2, c_y - width/2, c_z - width/2, x, y, z, ++d, max_d, t);
    }
    else if((c_x >= x && c_y >= y) && c_z < z){
      for(int i=0; i<1; i++)
        next++;
      make_trees(tr, next, width/2, c_x - width/2, c_y - width/2, c_z + width/2, x, y, z, ++d, max_d, t);
    }
    else if((c_x >= x && c_y < y) && c_z >= z){
      for(int i=0; i<2; i++)
        next++;
      make_trees(tr, next, width/2, c_x - width/2, c_y + width/2, c_z - width/2, x, y, z, ++d, max_d, t);
    }
    else if((c_x < x && c_y >= y) && c_z >= z){
      for(int i=0; i<3; i++)
        next++;
      make_trees(tr, next, width/2, c_x + width/2, c_y - width/2, c_z - width/2, x, y, z, ++d, max_d, t);
    }
    else if((c_x >= x && c_y < y) && c_z < z){
      for(int i=0; i<4; i++)
        next++;
      make_trees(tr, next, width/2, c_x - width/2, c_y + width/2, c_z + width/2, x, y, z, ++d, max_d, t);
    }
    else if((c_x < x && c_y >= y) && c_z < z){
      for(int i=0; i<5; i++)
        next++;
      make_trees(tr, next, width/2, c_x + width/2, c_y - width/2, c_z + width/2, x, y, z, ++d, max_d, t);
    }
    else if((c_x < x && c_y < y) && c_z >= z){
      for(int i=0; i<6; i++)
        next++;
      make_trees(tr, next, width/2, c_x + width/2, c_y + width/2, c_z - width/2, x, y, z, ++d, max_d, t);
    }
    else if((c_x < x && c_y < y) && c_z < z){
      for(int i=0; i<7; i++)
        next++;
      make_trees(tr, next, width/2, c_x + width/2, c_y + width/2, c_z + width/2, x, y, z, ++d, max_d, t);
    }
    else{
    }
  }
  else if(max_d == 0){
    *current = t;
  }
  else{
    if(all_same((*tr), current, t)){
      tree<float>::sibling_iterator parent;
      parent = (*tr).parent(current);
      (*tr).erase_children(parent);
      make_trees(tr, parent, 0, 0, 0, 0, 0, 0, 0, --d, --max_d, t);
    }
    else{
      *current = t;
    }
  }
}

tree<float> TStree::make_octree(float voxel_width, float max_x, float min_x, float max_y, float min_y, float max_z, float min_z, int max_d, float*** t){

  tree<float> tr = init_tree(max_d);
  tree<float>::sibling_iterator root;
  root = tr.begin();
  for(int x=0; x<GRID_SIZE; x++){
    for(int y=0; y<GRID_SIZE; y++){
      for(int z=0; z<GRID_SIZE; z++){
        make_trees(&tr, root, voxel_width/2.0, (max_x-min_x)/2.0, (max_y-min_y)/2.0, (max_z-min_z)/2.0, (float)x, (float)y, (float)z, 0, max_d, t[x][y][z]); 
      }
    }
  }
  return tr;

}
/*
 *
 * Initialize octree
 *
 * int max_d: maximum dimension of octree
 */
tree<float> TStree::init_tree(int max_d){

  tree<float> tr;
  tree<float>::sibling_iterator head;
  int num = 0;
  int dim = 0;
  head = tr.begin();
  head = tr.insert(head, EMP); 
  while(true){
    if(dim < max_d){
      if(head!=tr.end(head)){
        for(int i=0; i<8; i++){
          tr.append_child(head, EMP);
        }
        head = tr.begin(head);
        ++dim;
      }
      else{
        head = tr.parent(--head);
        --dim;
        ++head;
        if(head == tr.end()){
          break;
        }
      }
    }
    else{
      head = tr.parent(head);
      --dim;
      ++head;
    }
  }
  //kptree::print_tree_bracketed(tr, std::cout);
  return tr;
}

/*
 * Judge if the node of time-space has a value
 *
 * tree<vector<ValTime> >::sibling_iterator main_cur: node iterator of time-space tree
 */

bool TStree::has_value(tree<vector<ValTime> >::sibling_iterator main_cur){
  if((*main_cur).size() != 0){
    //cout << (*main_cur)[0];
    return true;
  }
  else
    return false;
}

/*
 * Judge if the node of signed distance octree has a value
 *
 * tree<vector<ValTime> >::sibling_iterator current: node iterator of octree
 */

bool TStree::has_value(tree<float>::sibling_iterator current){
  if(*current != EMP)
    return true;
  else
    return false;
}

/*
 * Merge new signed distance octree to time-space tree
 *
 * tree<vector<ValTime> >* main_tr: existing time-space tree
 * tree<vector<ValTime> >::sibling_iterator main_cur: root iterator of time-space tree
 * tree<float>* new_tr: new signed distance octree
 * tree<float>::sibling_iterator root iterator of new octree
 * int time: time of new octree
 */


void TStree::merge_trees(tree<vector<ValTime> >* main_tr, tree<vector<ValTime> >::sibling_iterator main_cur,tree<float>* new_tr, tree<float>::sibling_iterator new_cur, int time){
  if(has_value(new_cur)){
    cout << "aaa" << endl;
    if(has_value(main_cur)){
      cout << "bbb" << endl;
      ValTime* it = new ValTime(*new_cur, time);
      if(time == 0){
        (*main_cur).clear();
        (*main_cur).push_back(*it);
      }
      else if((*it).val != (*main_cur)[(*main_cur).size()-1].val){
        (*main_cur).push_back(*it);
      }
      else{
        cout << (*it).val << "," << (*main_cur)[(*main_cur).size()-1].val << endl;
      }

    }
    else{
      cout << "eee" << endl;
      tree<vector<ValTime> >::sibling_iterator child_main = (*main_tr).begin(main_cur);
      do{
        merge_trees(main_tr, child_main, new_tr, new_cur, time);
      } while(++child_main != (*main_tr).end(main_cur));
    }
  }
  else{
    cout << "ccc" << endl;
    if(has_value(main_cur)){
      cout << "ddd" << endl;
      for(int i=0; i<8; i++){
        (*main_tr).append_child(main_cur, *main_cur);
      }
      (*main_cur).clear();
    }
    tree<float>::sibling_iterator child_new = (*new_tr).begin(new_cur);
    tree<vector<ValTime> >::sibling_iterator child_main = (*main_tr).begin(main_cur);
    do{
      merge_trees(main_tr, child_main, new_tr, child_new, time);
    } while((*main_tr).end(main_cur) != ++child_main && (*new_tr).end(new_cur) != ++child_new);
  }
}

/*
 * Insert value to signed distance array 
 *
 * vector<int>* index: tree hierarchy index vector
 * int val: signed distance value
 * float *p: array of signed distances of the grid (return)
 */

void TStree::insertToVoxelPos(vector<int> index, int val, float* p){
  int num = 0;
  /*
     for(int i=0; i<index.size(); i++){
     num += index[i]*pow(2,3*(MAX_DIM-1-i));
     }
     for(int i=0; i<pow(2,3*(MAX_DIM-index.size())); i++){
     p[num+i] = val;
     }
     */
  for(int i=0; i<index.size(); i++){
    switch(index[i]){
      case 0:
        break;
      case 1:
        num += pow(2, MAX_DIM*2) * pow(2,(MAX_DIM-1-i));
        break;
      case 2:
        num += pow(2, MAX_DIM*1) * pow(2,(MAX_DIM-1-i));
        break;
      case 3:
        num += pow(2, MAX_DIM*0) * pow(2,(MAX_DIM-1-i));
        break;
      case 4:
        num += pow(2, MAX_DIM*1) * pow(2,(MAX_DIM-1-i)) + pow(2, MAX_DIM*2) * pow(2,(MAX_DIM-1-i));
        break;
      case 5:
        num += pow(2, MAX_DIM*0) * pow(2,(MAX_DIM-1-i)) + pow(2, MAX_DIM*2) * pow(2,(MAX_DIM-1-i));
        break;
      case 6:
        num += pow(2, MAX_DIM*0) * pow(2,(MAX_DIM-1-i)) + pow(2, MAX_DIM*1) * pow(2,(MAX_DIM-1-i));
        break;
      case 7:
        num += pow(2, MAX_DIM*0) * pow(2,(MAX_DIM-1-i)) + pow(2, MAX_DIM*1) * pow(2,(MAX_DIM-1-i)) + pow(2, MAX_DIM*2) * pow(2,(MAX_DIM-1-i));
        break;
    }
  }
  for (unsigned int x=0; x<pow(2,(MAX_DIM-index.size())); ++x)
    for (unsigned int y=0; y<pow(2,(MAX_DIM-index.size())); ++y)
      for (unsigned int z=0; z<pow(2,(MAX_DIM-index.size())); ++z)
      {
        p[(int)(num + pow(2, MAX_DIM*0) * x + pow(2, MAX_DIM*1) * y + pow(2, MAX_DIM*2) * z)] = val;
      }
}

void TStree::insertToVoxelPos(vector<int> index, int val, Eigen::VectorXd* p){
  int num = 0;
  /*
     for(int i=0; i<index.size(); i++){
     num += index[i]*pow(2,3*(MAX_DIM-1-i));
     }
     for(int i=0; i<pow(2,3*(MAX_DIM-index.size())); i++){
     p[num+i] = val;
     }
     */
  for(int i=0; i<index.size(); i++){
    switch(index[i]){
      case 0:
        break;
      case 1:
        num += pow(2, MAX_DIM*2) * pow(2,(MAX_DIM-1-i));
        break;
      case 2:
        num += pow(2, MAX_DIM*1) * pow(2,(MAX_DIM-1-i));
        break;
      case 3:
        num += pow(2, MAX_DIM*0) * pow(2,(MAX_DIM-1-i));
        break;
      case 4:
        num += pow(2, MAX_DIM*1) * pow(2,(MAX_DIM-1-i)) + pow(2, MAX_DIM*2) * pow(2,(MAX_DIM-1-i));
        break;
      case 5:
        num += pow(2, MAX_DIM*0) * pow(2,(MAX_DIM-1-i)) + pow(2, MAX_DIM*2) * pow(2,(MAX_DIM-1-i));
        break;
      case 6:
        num += pow(2, MAX_DIM*0) * pow(2,(MAX_DIM-1-i)) + pow(2, MAX_DIM*1) * pow(2,(MAX_DIM-1-i));
        break;
      case 7:
        num += pow(2, MAX_DIM*0) * pow(2,(MAX_DIM-1-i)) + pow(2, MAX_DIM*1) * pow(2,(MAX_DIM-1-i)) + pow(2, MAX_DIM*2) * pow(2,(MAX_DIM-1-i));
        break;
    }
  }
  for (unsigned int x=0; x<pow(2,(MAX_DIM-index.size())); ++x)
    for (unsigned int y=0; y<pow(2,(MAX_DIM-index.size())); ++y)
      for (unsigned int z=0; z<pow(2,(MAX_DIM-index.size())); ++z)
      {
        cout << x << "," << y << "," << z << endl;
        (*p)[(int)(num + pow(2, MAX_DIM*0) * x + pow(2, MAX_DIM*1) * y + pow(2, MAX_DIM*2) * z)] = val;
      }

}

/*
 * Convert tree index to voxel position 
 *
 * tree<vector<ValTime> >* t_tr: time tree
 * vector<int>* index: tree hierarchy index vector
 * int t: time frame
 * float *p: array of signed distances of the grid (return)
 */

void TStree::convertTreeIndextoVoxelPos(vector<ValTime> t_tr, vector<int> index, int t, float* p){
  for(int i=0; i<t_tr.size(); i++){
    int tmptime = t_tr[i].time;
    if(tmptime > t){
      insertToVoxelPos(index, t_tr[i-1].val, p);
    }
    else if(i == t_tr.size()-1){
      insertToVoxelPos(index, t_tr[i].val, p);
    }
  }
}

void TStree::convertTreeIndextoVoxelPos(vector<ValTime> t_tr, vector<int> index, int t, Eigen::VectorXd* p){
  for(int i=0; i<t_tr.size(); i++){
    int tmptime = t_tr[i].time;
    if(tmptime > t){
      insertToVoxelPos(index, t_tr[i-1].val, p);
    }
    else if(i == t_tr.size()-1){
      insertToVoxelPos(index, t_tr[i].val, p);
    }
  }
}
/*
 * Read signed distances of the grid at certain time frame from time-space tree
 *
 * tree<vector<ValTime> >* tr: time-space tree
 * tree<vector<ValTime> >::sibling_iterator current: root iterator of the time-space tree
 * vector<int>* index: index vector (to be empty)
 * int t: time frame
 * float *p: array of signed distances of the grid (return)
 */

void TStree::read_tree(tree<vector<ValTime> >* tr, tree<vector<ValTime> >::sibling_iterator current, vector<int>* index, int t, float* p){
  if(has_value(current)){
    convertTreeIndextoVoxelPos(*current, *index, t, p);
  }
  else{
    tree<vector<ValTime> >::sibling_iterator child = (*tr).begin(current);
    int order = 0;
    do{
      (*index).push_back(order);
      read_tree(tr, child, index, t, p);
      order++;
      (*index).pop_back();
    } while((*tr).end(current) != ++child);
  }
}

void TStree::read_tree(tree<vector<ValTime> >* tr, tree<vector<ValTime> >::sibling_iterator current, vector<int>* index, int t, Eigen::VectorXd* p){
  if(has_value(current)){
    convertTreeIndextoVoxelPos(*current, *index, t, p);
  }
  else{
    tree<vector<ValTime> >::sibling_iterator child = (*tr).begin(current);
    int order = 0;
    do{
      (*index).push_back(order);
      read_tree(tr, child, index, t, p);
      order++;
      (*index).pop_back();
    } while((*tr).end(current) != ++child);
  }
}



