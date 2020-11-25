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
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point processPointCloudduction and region based filtering

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr planeCloud = std::make_shared<typename pcl::PointCloud<PointT>>();
    typename pcl::PointCloud<PointT>::Ptr obstacleCloud = std::make_shared<typename pcl::PointCloud<PointT>>();
    typename pcl::ExtractIndices<PointT> extract(true);
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.filter(*planeCloud);
    auto indices_rem = extract.getRemovedIndices();
    extract.setIndices(indices_rem);
    extract.filter(*obstacleCloud);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(planeCloud, obstacleCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Segments out ground plane
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	std::shared_ptr<pcl::PointIndices> inliers = std::make_shared<pcl::PointIndices>();
    std::cout << "deets" << inliers << std::endl;
    for (pcl::index_t i = 0; i < cloud->points.size(); ++i){
        auto point = cloud->points[i];
        if (std::abs(point.z) <= distanceThreshold) {
            // std::cout << cloud->points[i] << std::endl;
            inliers->indices.push_back(i);
        }
    }
        
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Returned a vector of pointclouds, and I guess each pointcloud is a cluster
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    // while points, cluster = points.pop
    std::set<uint> remainingIdx;
    for (int i = 0; i < cloud->points.size(); i++) remainingIdx.insert(i);
    while (remainingIdx.size() != 0){
        std::cout << "Points left " << remainingIdx.size() << std::endl;
        typename pcl::PointCloud<PointT>::Ptr new_cluster = std::make_shared<typename pcl::PointCloud<PointT>>();
        uint centroidIdx = *(remainingIdx.begin());
        std::set<uint> usedIdx;
        usedIdx.insert(centroidIdx);
        PointT& centroid = cloud->at(centroidIdx);
        new_cluster->push_back(centroid);
        for (uint candidateIdx: remainingIdx) {
            PointT& candidatePoint = cloud->at(candidateIdx);
            float distance = std::sqrt(
                std::pow(centroid.x-candidatePoint.x, 2) +
                std::pow(centroid.y-candidatePoint.y, 2) +
                std::pow(centroid.z-candidatePoint.z, 2)
                );
            if (distance < clusterTolerance){
                usedIdx.insert(candidateIdx);
                new_cluster->push_back(candidatePoint);
                if (new_cluster->size() > maxSize) break;
            }
        }
        if (new_cluster->size() < minSize) continue;
        std::set<uint> temp;
        std::set_difference(remainingIdx.begin(), remainingIdx.end(),
        usedIdx.begin(), usedIdx.end(),
        std::inserter(temp, temp.begin()));
        remainingIdx = std::move(temp);
        clusters.push_back(new_cluster);
    }
    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

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