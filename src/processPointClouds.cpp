// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


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
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, 
                    float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    // Jari 04/23/2020

    // Creata the voxel grid object
    pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
    //std::cout << typeid(vg).name() << std::endl;

    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloudFiltered);

    // Region of Interest
    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));    
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for (int point : indices) {
        inliers->indices.push_back(point);
    }    

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

#if 0
    return cloudFiltered;
#else
    return cloudRegion;
#endif
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
  // Jari 04/17/2020

    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>());

    // Copy inliers into the planeCloud
    for (int index : inliers->indices) {
        planeCloud->points.push_back(cloud->points[index]);
    }

    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter( *obstCloud);

    std::cout << "PointCloud representing the plane: " << planeCloud->width * planeCloud->height << " data points." << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    // std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, cloud);
  
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, 
                                         int maxIterations, float distanceThreshold)
{
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult;

    std::cout << "Entering SegmentPlane() ..." << std::endl;
    std::cout << "Cloud Size: " << cloud->size() << " points" << std::endl;
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.
    // Jari 04/17/2020

#if 0
/*  
    // PCL Implementation
    // ==================
    pcl::SACSegmentation<PointT> seg;
	pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};

    // Configure Segmentator
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
    
    // Segment into inliers
    seg.setInputCloud(cloud);
    seg.segment (*inliers, *coefficients);
    if (!inliers->indices.size()) {
        std::cout << "*** FAIL: inliers size = " << inliers->indices.size() << std::endl;
    } else {
        std::cout << "*** OK: inliers size = " << inliers->indices.size() << std::endl;
    }

    segResult = SeparateClouds(inliers,cloud);
 */
 #else

    // My Ransac Implementation
    //=========================

	std::unordered_set<int> inliersResult;

	// For max iterations 
	for (int iterations = 0; iterations < maxIterations; iterations++) {

		// Pick 3 points in random (3 points can determine a plane)
		std::unordered_set<int> inliers;
		while(inliers.size() < 3) {
			int point = rand()%(cloud->points.size());
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
		// Vector2 from point1 to point3
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

			// Check if one of the random plane reference points
			if (inliers.count(index) > 0) {
				continue;
			}

		    PointT point = cloud->points[index];
			float x = point.x;
			float y = point.y;
			float z = point.z;

			// Calcluate points distance to the plane
			float d = fabs(A*x + B*y + C*z + D)/sqrt(A*A + B*B + C*C);

			// Check if close enough to be an inlier
			if (d <= distanceThreshold) {
				inliers.insert(index);
			}

		}

		if (inliers.size() > inliersResult.size()) {
			inliersResult = inliers;
		}		
		
	}  // for (iterations)
	
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::cout << "InliersResult size: " << inliersResult.size() << std::endl;
    std::cout << "Cloud Points Size: " << cloud->points.size() << " points" << std::endl;

    startTime = std::chrono::steady_clock::now();
 
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>());
    segResult.first = obstCloud;
    segResult.second = planeCloud;
  
    // Construct pair of Onstacle and Plane Point Clouds to be returned
	for(int index = 0; index < cloud->points.size(); index++)
	{
		//pcl::PointXYZ point = cloud->points[index];
		PointT point = cloud->points[index];
		if(inliersResult.count(index)) {
            // Plane Cloud
			segResult.second->push_back(point);
        } else {
            // Obstacle Cloud
            segResult.first->push_back(point);
        }
	}
    endTime = std::chrono::steady_clock::now();
    elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Building Plane and Obstavcle Points Clouds took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::cout << "Plane Cloud size: " << segResult.second->size() << std::endl;
    std::cout << "Obstacle Cloud size: " << segResult.first->size() << std::endl;
 
 #endif  

    return segResult;
}


template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int indice, const std::vector<std::vector<float>> points, 
             std::vector<int> &cluster, std::vector<bool> &processed, 
             KdTree *tree, float distanceTol)
{
	processed[indice] = true;
	cluster.push_back(indice);

	std::vector<int> nearest = tree->search(points[indice], distanceTol);

	for (int id : nearest) {
		if (!processed[id]) {
			//std::cout << "Calling clusterHelper, id = " << indice << std::endl;
			clusterHelper(id, points, cluster, processed, tree, distanceTol);
		}
	}
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> 
ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, 
                           float clusterTolerance, int minSize, int maxSize)
{

    std::cout << "Entering Clustering() ..." << std::endl;
    std::cout << "Obstable Cloud Size: " << cloud->size() << " points" << std::endl;

    // Return vector of Clusters
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
 
#if 0
/*
    // PCL Implementation
    //===================

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Jari 04/22/2020
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    for( pcl::PointIndices getIndices: clusterIndices) {

        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        for(int index : getIndices.indices) {
            cloudCluster->points.push_back(cloud->points[index]);
        }

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
    }
*/
#else
    // My Clustering Implementation
    //=============================

	KdTree* tree = new KdTree;
    std::vector<std::vector<float>> points;
    /*
	std::vector<std::vector<float>> points = { {-6.2,7,4}, {-6.3,8.4,3}, {-5.2,7.1, 4.9}, {-5.7,6.3, 3.5}, 
	                                           {7.2,6.1,-4.5}, {8.0,5.3,-3.5}, {7.2,7.1,-2}, 
											   {0.2,-7.1,0}, {1.7,-6.9,1.5}, {-1.2,-7.2,2.5}, {2.2,-8.9,0.7} };
    */ 
    std::vector<float> point(3);
 
    // Insert Cloud points into KD-tree
    for (int i=0; i < cloud->size(); i++) {
        point[0] = cloud->points[i].x;
        point[1] = cloud->points[i].y;
        point[2] = cloud->points[i].z;
        points.push_back(point);
    	tree->insert(point,i);
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Building KD-tree took " << elapsedTime.count() << " milliseconds" << std::endl;
    startTime = std::chrono::steady_clock::now();

    /*
    for (int i=0; i < points.size(); i++) {
        tree->insert(points[i],i);
    }
    */

    /*
  	std::cout << "Test Search: " << std::endl;
  	std::vector<int> nearby = tree->search({-4.0,3.0,-0.5},3.0);
  	for(int index : nearby) {
        std::cout << index << ",";
    }
  	std::cout << std::endl;
    */

	std::vector<std::vector<int>> indexClusters;
	std::vector<bool> processed(cloud->size(), false);

	int i = 0;
	while (i < points.size())
	{
		if (processed[i]) {
			i++;
			continue;
		}

		std::vector<int> indexCluster;
		clusterHelper(i, points, indexCluster, processed, tree, clusterTolerance);
		indexClusters.push_back(indexCluster);
		i++;
	}

    endTime = std::chrono::steady_clock::now();
    elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Eucliding Clustering took " << elapsedTime.count() << " milliseconds" << std::endl;
    startTime = std::chrono::steady_clock::now();

    std::cout << "# of Clusters found: " << indexClusters.size() << std::endl;

    // Construct vector of cluster point clouds to be returned
  	for (std::vector<int> cluster : indexClusters)
  	{
        if ( (cluster.size() >= minSize) && (cluster.size() <= maxSize) ) {
  		    typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
            for (int idx : cluster) {
                PointT *point = new PointT;
                point->x = points[idx][0];
                point->y = points[idx][1];
                point->z = points[idx][2];              
                clusterCloud->points.push_back(*point);
            }
 		    clusters.push_back(clusterCloud);
		} 
  	} 
    
#endif

    endTime = std::chrono::steady_clock::now();
    elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Building Point Cloud took " << elapsedTime.count() << " milliseconds" << std::endl;

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