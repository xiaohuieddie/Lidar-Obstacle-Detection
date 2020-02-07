/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud){
  /* 
  //testing with one pcd file
  ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
  */ 
  
  // filtering the point cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.1 , Eigen::Vector4f (-10, -6,-2, 1), Eigen::Vector4f ( 30, 7, 2, 1));
  
  //render the region of interest
  /*
  Box box;
  box.x_min = -10;
  box.y_min = -5;
  box.z_min = -2;
  box.x_max = 30;
  box.y_max = 7;
  box.z_max = 2;
  
  renderBox(viewer, box, 2.0);
  renderPointCloud(viewer,filterCloud,"filterCloud");
  */


  // TODO:: Create point processor
  ProcessPointClouds<pcl::PointXYZI> pointProcessor;
    
  /*
  //using SegmentPlane function with the help of pcl

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessor.SegmentPlane(filterCloud, 100, 0.21);
  */
  
  //using Segment function developed from scratch
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessor.Segment(filterCloud, 100, 0.21);
  
  //render the plane points    
  renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));
    
  //render the obstacle points    
  //renderPointCloud(viewer,segmentCloud.first,"obstacleCloud",Color(1,0,0));
  
  //render cluster of obstacle cloud
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 0.3, 50, 2000);

  int clusterId = 0;
  std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

  for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)  {
    std::cout << "cluster size ";
    pointProcessor.numPoints(cluster);
    //render different cluster
    renderPointCloud(viewer,cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId]);
    ++clusterId;
  
    // render boxes around the cluster
    Box box = pointProcessor.BoundingBox(cluster);
    renderBox(viewer, box, clusterId);
    
  }
  

  
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar *lidar = new Lidar(cars, 0);
    

    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar -> scan();
    //view lidar ray in the point cloud
    /*
    renderRays(viewer, lidar -> position, inputCloud);
    renderPointCloud(viewer, inputCloud, "inputCloud");
    */
  
    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    
    //using SegmentPlane function with the help of pcl
    /*
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, 100, 0.2);
    */
  
    //using Segment function developed from scratch
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.Segment(inputCloud, 100, 0.2);
   
 //render the obstacle cloud points and plane points    
  /*  renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));
    */
  
  //render cluster of obstacle cloud
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);

int clusterId = 0;
std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
{
      std::cout << "cluster size ";
      pointProcessor.numPoints(cluster);
      renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
      ++clusterId;
  
  // render boxes around the cluster
  Box box = pointProcessor.BoundingBox(cluster);
  renderBox(viewer, box, clusterId);
}
  
  
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    //creating point processor, and loading pcl files from folder
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
	pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
  
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    //cityBlock(viewer);
  

    while (!viewer->wasStopped ())
    {
      
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd ((*streamIterator). string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
          streamIterator = stream.begin();
      
        viewer->spinOnce ();
    } 
}