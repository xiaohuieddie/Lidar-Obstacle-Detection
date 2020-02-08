/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "render/render.h"
#include "processPointClouds.h"

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
    
    void insertHelper(Node **node, uint depth, std::vector<float> point, int id){
      
      if (*node == NULL){
        *node = new Node(point, id);
      }
      else {
        uint cd = depth % 3;
        if (point[cd] < ((*node)->point[cd])){
          insertHelper(&((*node)->left), depth+1, point, id);
        }            
        else{
          insertHelper(&((*node)->right), depth+1, point, id);
        }
          
      }
     
    }
            
        
        

	void insert(pcl::PointXYZ point_pcl, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
      std::vector<float> point;
      point.push_back(point_pcl.x);
      point.push_back(point_pcl.y);
      point.push_back(point_pcl.z);
      insertHelper(&root, 0, point, id);

	}

	// return a list of point ids in the tree that are within distance of target
	
  void searchHelper(pcl::PointXYZ target, Node *node, int depth, float distanceTol, std::vector<int> &ids){
    
    std::vector<float> target_xyz;
    target_xyz.push_back(target.x);
    target_xyz.push_back(target.y);
    target_xyz.push_back(target.z);
    
    if (node != NULL){
      if ((node->point[0] <= (target_xyz[0] + distanceTol) && node->point[0] >= (target_xyz[0] - distanceTol)) && (node->point[1] <= (target_xyz[1] + distanceTol) && node->point[1] >= (target_xyz[1] - distanceTol)) && (node->point[2] <= (target_xyz[3] + distanceTol) && node->point[2] >= (target_xyz[3] - distanceTol)) ){
        float distance = sqrt((node->point[0]-target_xyz[0])*(node->point[0]-target_xyz[0]) + (node->point[1]-target_xyz[1])*(node->point[1]-target_xyz[1]) + (node->point[2]-target_xyz[2])*(node->point[2]-target_xyz[2]));
        if (distance <= distanceTol){
          ids.push_back(node->id);
        }
      }
      if ((target_xyz[depth%3]-distanceTol) < (node->point[depth%3])){
        searchHelper( target, node->left, depth+1, distanceTol, ids);
      }
      if ((target_xyz[depth%3]+distanceTol) > (node->point[depth%3])){
        searchHelper( target, node->right, depth+1, distanceTol, ids);
      }
        
    }
  }
  
  std::vector<int> search(pcl::PointXYZ target, float distanceTol)
	{
		std::vector<int> ids;
        searchHelper(target, root, 0, distanceTol, ids);
      
		return ids;
	}
	

};




