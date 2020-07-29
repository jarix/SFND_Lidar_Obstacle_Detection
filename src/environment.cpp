/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

void printBounds(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) 
{
    float minX = cloud->points[0].x, maxX = cloud->points[0].x;
    float minY = cloud->points[0].y, maxY = cloud->points[0].y;
    float minZ = cloud->points[0].z, maxZ = cloud->points[0].z;
    
    for (auto it = cloud->points.begin(); it != cloud->points.end(); ++it ) {
        if ( (*it).x < minX)  minX = (*it).x;
        if ( (*it).x > maxX)  maxX = (*it).x;
        if ( (*it).y < minY)  minY = (*it).y;
        if ( (*it).y > maxY)  maxY = (*it).y;
        if ( (*it).z < minZ)  minZ = (*it).z;
        if ( (*it).z > maxZ)  maxZ = (*it).z;
    }

    std::cout << "Input Cloud Min ( " << minX << ", " << minY << ", " << minZ << ")" << std::endl;
    std::cout << "Input Cloud Max ( " << maxX << ", " << maxY << ", " << maxZ << ")" << std::endl;
}

void printBoundsEx(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) 
{
    pcl::PointXYZI minPoint, maxPoint;

    pcl::getMinMax3D(*cloud, minPoint, maxPoint);
 
    std::cout << "Input Cloud Min ( " << minPoint.x << ", " << minPoint.y << ", " << minPoint.z << ")" << std::endl;
    std::cout << "Input Cloud Max ( " << maxPoint.x << ", " << maxPoint.y << ", " << maxPoint.z << ")" << std::endl;
}


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


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor
    // Jari
    Lidar *pLidar = new Lidar(cars, 0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = pLidar->scan(); 
  
    //const Vect3 origin( 0, 0, 0);
    Vect3 origin = pLidar->position;
    std::cout << "Origin: " << origin.x << ", " << origin.y << ", " << origin.z << endl;
    
    //void renderRays(pcl::visualization::PCLVisualizer::Ptr& viewer, const Vect3& origin, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    //renderRays(viewer, origin, pointCloud);

    //void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::string name, Color color = Color(1,1,1));
    std::string name = "My pointcloud";
    Color color(1,0,0);
    //renderPointCloud(viewer, inputCloud, name, color);

    // TODO:: Create point processor
    // Instantiate In the stack:
    ProcessPointClouds<pcl::PointXYZ>  pointProcessor;
    // Instatiate in the heap:
    //ProcessPointClouds<pcl::PointXYZ> *pPointProcessor = new ProcessPointClouds<pcl::PointXYZ>();

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, 100, 0.2);

    //renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0));
    //renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = { Color(1,0,0), Color(1,1,0), Color(0,0,1)};
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster: cloudClusters) {

        std::cout << "cluster size ";
        pointProcessor.numPoints(cluster);
        renderPointCloud( viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId%colors.size()]);

        Box box = pointProcessor.BoundingBox(cluster);
        renderBox( viewer, box, clusterId);

        ++clusterId;

    }

}

/*
void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    // Create Point Processor
    ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();

    // Load Point Cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");

    //printBounds(inputCloud);
    printBoundsEx(inputCloud);

    // Filter to downscale
    inputCloud = pointProcessorI->FilterCloud(inputCloud, 0.2 , Eigen::Vector4f( -30, -6, -10, 1), Eigen::Vector4f( 40, 6, 3, 1));

    // Segment to plane and obstacles
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(inputCloud, 100, 0.2);
    std::cout << "Plane Cloud: " << segmentCloud.second->size() << " points" << std::endl;
    printBounds(segmentCloud.second);
    std::cout << "Obstacle Cloud: " << segmentCloud.first->size() << " points" << std::endl;
    printBounds(segmentCloud.first);

    //renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));

    // Cluster obstacles
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 0.5, 20, 800);

    int clusterId = 0;
    std::vector<Color> colors = { Color(1,0,0), Color(1,1,0), Color(0,0,1)};
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster: cloudClusters) {

        std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);
        renderPointCloud( viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId%colors.size()]);

        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox( viewer, box, clusterId);

        ++clusterId;

    }

}
*/

void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer, 
               ProcessPointClouds<pcl::PointXYZI> *pointProcessorI, 
               pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud)
{
    // Create Point Processor
    //ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();

    // Load Point Cloud
    //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");

    //printBounds(inputCloud);
    //printBoundsEx(inputCloud);

    // Filter to downscale
    inputCloud = pointProcessorI->FilterCloud(inputCloud, 0.2 , Eigen::Vector4f( -30, -6, -10, 1), Eigen::Vector4f( 40, 6, 3, 1));

    // Segment to plane and obstacles
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> 
                    segmentCloud = pointProcessorI->SegmentPlane(inputCloud, 100, 0.2);
    //std::cout << "Plane Cloud: " << segmentCloud.second->size() << " points" << std::endl;
    //printBounds(segmentCloud.second);
    //std::cout << "Obstacle Cloud: " << segmentCloud.first->size() << " points" << std::endl;
    //printBounds(segmentCloud.first);

    //renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));

    // Cluster obstacles
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 0.5, 20, 800);
    std::cout << "*** # of Clusters returned from Clustering(): " << cloudClusters.size() << std::endl;
    /*
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr pc : cloudClusters) {
        std::cout << "Cluster Size: " << pc->points.size() << std::endl;
    }
    */
    /*
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc = cloudClusters[0];
    for (pcl::PointXYZI po : *pc ) {
        std::cout << "(" << po.x << "," << po.y << "," << po.z << ") ";
    }
    */

    int clusterId = 0;
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster: cloudClusters) {

        renderPointCloud( viewer, cluster, "obstCloud"+std::to_string(clusterId), Color(1,1,0));

        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox( viewer, box, clusterId);

        ++clusterId;
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
    //CameraAngle setAngle = XY;
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);

    //simpleHighway(viewer);
    //cityBlock(viewer);

    // Create Point Cloud Processor
    ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;


    uint i = 0;
    while (!viewer->wasStopped ())
    {
#if 1
        //if (i++ > 0) continue; 
        
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if (streamIterator == stream.end()) {
            streamIterator = stream.begin();
        }
#endif
        viewer->spinOnce ();
    }
}