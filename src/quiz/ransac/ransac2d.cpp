/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData2D()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

//pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
pcl::PointCloud<pcl::PointXYZI>::Ptr CreateData3D()
{
	//ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	ProcessPointClouds<pcl::PointXYZI> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	srand(time(NULL));

	// TODO: Fill in this function
	// Jari 04/17/2020
	std::cout << "Entering Ransac Line (2D)..." << std::endl;

	//std::cout << "Cloud size, width = " << cloud->width << ", height = " << cloud->height << std::endl;
	std::cout << "Cloud size = " << cloud->points.size() << std::endl;

	std::cout << "Point 0: " << cloud->points[0] << std::endl;

	std::unordered_set<int> inliersResult;

	// For max iterations 
	for (int iterations = 0; iterations < maxIterations; iterations++) {

		// Pick 2 pointsin random
		std::unordered_set<int> inliers;
		while(inliers.size() < 2) {
	
			int point = rand()%(cloud->points.size());
			//std::cout << "p1 = " << p1 << ", p2 = " << p2 << std::endl;
			inliers.insert(point);
		}

		float x1, y1, x2, y2;

		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;

		float a = (y1-y2);
		float b = (x2-x1);
		float c = ((x1*y2)-(x2*y1));

		// Measure distance to the line for all the other points
		for(int index = 0; index < cloud->points.size(); index++) {

			// Check if one of the random line reference points
			if (inliers.count(index) > 0) {
				continue;
			}

			pcl::PointXYZ point = cloud->points[index];
			//std::cout << "Point = " << point << std::endl;
			float x3 = point.x;
			float y3 = point.y;

			// Calcluate points distance to th line
			float d = fabs(a*x3 + b*y3 + c)/sqrt(a*a + b*b);

			// Check if close enough to be an inlier
			if (d <= distanceTol) {
				inliers.insert(index);
			}

		}

		if (inliers.size() > inliersResult.size()) {
			inliersResult = inliers;
		}

	}
	
	return inliersResult;

}

std::unordered_set<int> Ransac3D(typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceTol)
{
	srand(time(NULL));

	// TODO: Fill in this function
	// Jari 04/19/2020
	std::cout << "Entering Ransac Plane (3D)..." << std::endl;

	//std::cout << "Cloud size, width = " << cloud->width << ", height = " << cloud->height << std::endl;
	std::cout << "Cloud size = " << cloud->points.size() << std::endl;

	std::cout << "Point 0: " << cloud->points[0] << std::endl;

	std::unordered_set<int> inliersResult;

	// For max iterations 
	for (int iterations = 0; iterations < maxIterations; iterations++) {

		// Pick 3 points in random
		std::unordered_set<int> inliers;
		while(inliers.size() < 3) {
	
			int point = rand()%(cloud->points.size());
			//std::cout << "p1 = " << p1 << ", p2 = " << p2 << std::endl;
			inliers.insert(point);
		}

		float x1, y1, z1, x2, y2, z2, x3, y3, z3;

		// Iterate over points
		auto itr = inliers.begin();
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

		std::vector<int> v1(3), v2(3);

		// Vector1 from point1 to point2
		v1[0] = x2-x1;
		v1[1] = y2-y1;
		v1[2] = z2-z1;
		// Vector1 from point1 to point3
		v2[0] = x3-x1;
		v2[1] = y3-y1;
		v2[2] = z3-z1;
	
		// Find normal vector with cross product of v1 x v2 = <i,j,k>
		int i = ((y2-y1)*(z3-z1)) - ((z2-z1)*(y3-y1));
		int j = ((z2-z1)*(x3-x1)) - ((x2-x1)*(z3-z1));
		int k = ((x2-x1)*(y3-y1)) - ((y2-y1)*(x3-x1));

		float A = i;
		float B = j;
		float C = k;
		float D = -((i*x1) + (j*y1) + (k*z1));

		// Measure distance to the plane for all the other points
		for(int index = 0; index < cloud->points.size(); index++) {

			// Check if one of the random line reference points
			if (inliers.count(index) > 0) {
				continue;
			}

			pcl::PointXYZI point = cloud->points[index];
			//std::cout << "Point = " << point << std::endl;
			float x = point.x;
			float y = point.y;
			float z = point.z;

			// Calcluate points distance to the plane
			float d = fabs(A*x + B*y + C*z + D)/sqrt(A*A + B*B + C*C);

			// Check if close enough to be an inlier
			if (d <= distanceTol) {
				inliers.insert(index);
			}

		}

		if (inliers.size() > inliersResult.size()) {
			inliersResult = inliers;
		}		
		
	}
	
	return inliersResult;

}


int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	#if 0
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData2D();
	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac2D(cloud, 10, 0.2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	#else
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = CreateData3D();
	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac3D(cloud, 10, 0.3);

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZI>());
	#endif

	for(int index = 0; index < cloud->points.size(); index++)
	{
		//pcl::PointXYZ point = cloud->points[index];
		pcl::PointXYZI point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
