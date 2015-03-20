/*
 * script.cpp
 *
 *  Created on: 16 mrt. 2015
 *      Author: RGrandia
 */

#include <algorithm>
#include <string>
#include <iostream>
#include "tree.hh"

using namespace std;

void printtree(const tree<int> &a_tree);
void split_in_four(tree<int> &a_tree,const tree<int>::iterator iter,int a_fill[]);

int main()
   {
	// Create the Tree object as tr
   tree<int> tr;

   // Declare two iterators, one for initializing, one for pointing to the root
   tree<int>::iterator initialize, root, loc, child;

   // Declare node filling
   int fill_it[] = {1,2,3,4};

   // Initialize and insert the root node
   initialize = tr.begin();
   root = tr.insert(initialize,0);
   child = tr.append_child(root);
   loc = child;
   split_in_four(tr, loc, fill_it);
   split_in_four(tr, ++loc, fill_it);

   printtree(tr);
   cout << endl;
   cout << "number of nodes = " << tr.size() << endl;

   return 0;
}

void printtree(const tree<int> &a_tree){
	tree<int>::iterator iter = a_tree.begin();
	tree<int>::iterator iter_end = a_tree.end();
	while(iter!=iter_end){
		// Add a spaces equal to the depth of current node
		for(int i=0; i<a_tree.depth(iter); ++i)
		        cout << " ";
		// Print value at current node
		cout << (*iter) << endl;
		// Move to next node
		++iter;
	}
}

void split_in_four(tree<int> &a_tree,const tree<int>::iterator iter,int a_fill[]){
	for(int i=0; i<4; ++i){
	a_tree.append_child(iter,a_fill[i]);
	}
}






