/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "render/render.h"

#define DO_3D 1 

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}


	// with Double Pointer
	void insertHelper(Node **node, uint depth, std::vector<float> point, int id)
	{
		// Check if empty tree
		if (*node == NULL) {
			*node = new Node(point, id);
		} else {
#ifdef DO_3D
			// x,y,z values for consequtive depths
			uint cd = depth % 3;  
#else
			// check x value for even depth, y value for odd depth
			uint cd = depth % 2;  
#endif
			if (point[cd] < ((*node)->point[cd])) {
				insertHelper(&((*node)->left), depth+1, point, id);
			} else {
				insertHelper(&((*node)->right), depth+1, point, id);
			}
		}
	}

/*
	// with Pointer Reference
	void insertHelper(Node *&node, uint depth, std::vector<float> point, int id)
	{
		// Check if empty tree
		if (node == NULL) {
			node = new Node(point, id);
		} else {
			// check x value for even depth, y value for odd depth
			uint cd = depth % 2;  

			if (point[cd] < (node->point[cd])) {
				insertHelper(node->left, depth+1, point, id);
			} else {
				insertHelper(node->right, depth+1, point, id);
			}
		}
	}
*/

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		// Jari 04/22/2020
		//std::cout << "Inserting point: id = " << id << " (" << point[0] << "," << point[1] << ")" << std::endl; 

		insertHelper(&root, 0, point, id);
		//insertHelper(root, 0, point, id);
	}


#ifdef DO_3D
	void searchHelper(std::vector<float> target, Node *node, int depth, float distanceTol, std::vector<int> &ids)
	{

		if (node != NULL) {

			// Check if target within the box
			if( (node->point[0]>=(target[0]-distanceTol) && node->point[0]<=(target[0]+distanceTol)) &&
			    (node->point[1]>=(target[1]-distanceTol) && node->point[1]<=(target[1]+distanceTol)) &&
				(node->point[2]>=(target[2]-distanceTol) && node->point[2]<=(target[2]+distanceTol)) ) {

				// Cacluate actual distance
				float distance = sqrt(((node->point[0]-target[0])*(node->point[0]-target[0])) + 
				                      ((node->point[1]-target[1])*(node->point[1]-target[1])) +
									  ((node->point[2]-target[2])*(node->point[2]-target[2])) );
				if (distance <= distanceTol) {
					ids.push_back(node->id);
				}

			}
			if ((target[depth%3]-distanceTol) < node->point[depth%3]) {
				searchHelper( target, node->left, depth+1, distanceTol, ids);
			}
			if ((target[depth%3]+distanceTol) > node->point[depth%3]) {
				searchHelper( target, node->right, depth+1, distanceTol, ids);
			}
		}

	}
#else
	void searchHelper(std::vector<float> target, Node *node, int depth, float distanceTol, std::vector<int> &ids)
	{

		if (node != NULL) {

			// Check if target within the box
			if( (node->point[0]>=(target[0]-distanceTol) && node->point[0]<=(target[0]+distanceTol)) &&
			    (node->point[1]>=(target[1]-distanceTol) && node->point[1]<=(target[1]+distanceTol)) ) {

				// Cacluate actual distance
				float distance = sqrt(((node->point[0]-target[0])*(node->point[0]-target[0])) + 
				                      ((node->point[1]-target[1])*(node->point[1]-target[1])) );
				if (distance <= distanceTol) {
					ids.push_back(node->id);
				}

			}
			if ((target[depth%2]-distanceTol) < node->point[depth%2]) {
				searchHelper( target, node->left, depth+1, distanceTol, ids);
			}
			if ((target[depth%2]+distanceTol) > node->point[depth%2]) {
				searchHelper( target, node->right, depth+1, distanceTol, ids);
			}
		}

	}
#endif

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);

		return ids;
	}
	

};




