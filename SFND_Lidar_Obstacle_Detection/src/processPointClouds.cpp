// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include "KDtree_3D.h"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
  
  // Create the filtering object
  typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>()); 
  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (filterRes, filterRes, filterRes);
  sor.filter (*cloud_filtered);
  
  //region of interest
  typename pcl::PointCloud<PointT>::Ptr cloud_interest (new pcl::PointCloud<PointT>());
  pcl::CropBox<PointT> cropBox(true);
  cropBox.setMin(minPoint);
  cropBox.setMax(maxPoint);
  cropBox.setInputCloud(cloud_filtered);
  cropBox.filter(*cloud_interest);
  
  //roof
  std::vector<int> indices;
  pcl::CropBox<PointT> roof(true);
  roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
  roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
  roof.setInputCloud(cloud_interest);
  roof.filter(indices);

  //
  typename pcl::PointIndices::Ptr inliners (new pcl::PointIndices); 
    
    for (int index : indices){
        inliners -> indices.push_back(index);
    }
  pcl::ExtractIndices<PointT> extract;
  // Extract the roof
  extract.setInputCloud (cloud_interest);
  extract.setIndices (inliners);
  extract.setNegative (true);
  extract.filter (*cloud_interest);
  
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_interest;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>()); 
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>()); 
    
    for (int index : inliers -> indices){
        planeCloud -> points.push_back(cloud -> points[index]);
    }
    
    typename pcl::ExtractIndices<PointT> extract;
    // Extract the inliers
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}

//Build Segmentation from scratch
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Segment_Scratch(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
  // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
  
	pcl::PointIndices::Ptr inliersResult (new pcl::PointIndices ());
	srand(time(NULL));
	// For max iterations 
    for (int i=1; i<=maxIterations; i++){
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
      
      // Randomly sample subset and fit line
      while(inliers->indices.size() <3){
        inliers->indices.push_back(rand() % (cloud -> points.size()));
      }
      
      float x1, y1, z1, x2, y2, z2, x3, y3, z3;
      auto itr = inliers->indices.begin();
      x1 = cloud->points[*itr].x;
      y1 = cloud->points[*itr].y;
      z1 = cloud->points[*itr].z;
      itr++;
      x2 = cloud->points[*itr].x;
      y2 = cloud->points[*itr].y;
      z2 = cloud->points[*itr].z;
      itr++;
      x3 = cloud->points[*itr].x;
      y3 = cloud->points[*itr].y;
      z3 = cloud->points[*itr].z;
      
      float a = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
      float b = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
      float c = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
      float d = -(a*x1 + b*y1 + c*z1);
      
      //skip the three points in the fitted plane
      for (int index=0; index<cloud->points.size(); index++){
        if (index == (inliers->indices[0] or inliers->indices[1] or inliers->indices[2])){
          continue;
        }
        
        // Measure distance between every point and fitted plane
        PointT point = cloud->points[index];
        float x4 = point.x;
        float y4 = point.y;
        float z4 = point.z;
        float distance = fabs(a*x4 + b*y4 + c*z4 + d) / sqrt(a*a + b*b + c*c);
        
        // If distance is smaller than threshold count it as inlier
        if (distance <= distanceTol){
          inliers->indices.push_back(index);
        }
      }
      
      // Return indicies of inliers from fitted plane with most inliers
      if (inliers->indices.size() > inliersResult->indices.size())
      {
        inliersResult->indices = inliers->indices;
      }
    }
    if (inliersResult->indices.size() == 0){
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;    
    }
  
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliersResult,cloud);
    
  return segResult;
}

//Build function "SegmentPlane" for egmentation using pcl
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());  
    pcl::SACSegmentation<PointT> seg;
  
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);
  
  // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;    
    }
  
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

/*
template<typename PointT>
void ProcessPointClouds<PointT>::Proximity(int indice, const std::vector<std::vector<float>> points, std::vector<int> &cluster_id, std::vector<bool> &Processed, KdTree* tree, float distanceTol){
Processed[indice] = true;
    cluster_id.push_back(indice);
    
    std::vector<int> nearby_points = tree->search(points[indice], distanceTol);
   
    for (int i : nearby_points){
     if (!Processed[i]){
      Proximity(i, points, cluster_id, Processed, tree, distanceTol);
     }
    }

}

//Build Clustering_Scratch Function from scratch
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering_Scratch(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
void Proximity(int indice, const std::vector<std::vector<float>> points, std::vector<int> &cluster_id, std::vector<bool> &Processed, KdTree* tree, float distanceTol){



std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>> points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
    std::vector<bool> Processed_points(points.size(), false);
    int i=0;
    while (i < points.size()){
      if (Processed_points[i]){
        i++;
        continue;
      }
      std::vector<int> cluster_id;
      Proximity(i, points, cluster_id, Processed_points, tree, distanceTol);
      clusters.push_back(cluster_id);
      i++;
    }
  
	return clusters;

}

}
*/
//Build Clustering function with the help with pcl
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

  // Time clustering process
  auto startTime = std::chrono::steady_clock::now();

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

  // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    
  // Creating the KdTree object for the search method of the extraction
  typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud (cloud);
  
  //cluster_indices[0] contains all indices of the first cluster in our point cloud.
  std::vector<pcl::PointIndices> cluster_indices;
  
  //extracted the clusters out of our point cloud and saved the indices in cluster_indices
  typename pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance (clusterTolerance); 
  ec.setMinClusterSize (minSize);
  ec.setMaxClusterSize (maxSize);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);

  //loop each cluster 
  for (pcl::PointIndices getIndices : cluster_indices)
  {
    typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
    //loop each points of each cluster and saved them into cloud_cluster
    for (int index : getIndices.indices)
    {
      cloud_cluster->points.push_back (cloud->points[index]);
    }
    
    //define the size of each cluster
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    
    //save each cluster into 'clusters'
    clusters.push_back (cloud_cluster);
  }
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}