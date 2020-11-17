/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include <algorithm>
#include "../../processPointClouds.h"
#include <Eigen/Core>
#include <Eigen/Geometry> 
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <chrono> 


pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
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

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
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

struct Line {
	float A;
	float B;
	float C;
};

float LineDistance(const Line& line, const pcl::PointXYZ point){
	return std::abs(line.A * point.x + line.B * point.y + line.C)/std::sqrt(line.A*line.A + line.B*line.B);
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	auto start = std::chrono::high_resolution_clock::now(); 
	cout << "Height is: " << cloud->height << "Width is : " << cloud->width << std::endl;
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.01);

	seg.setInputCloud (cloud);
	seg.segment (*inliers, *coefficients);

	auto stop = std::chrono::high_resolution_clock::now(); 
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start) * 0.001; 
	cout << "Theirs took " << duration.count() << endl; 

	// return inliers->indices
	// TODO: Fill in this function

	// For max iterations 
	int cloudLen = cloud->points.size();
	std::unordered_set<int> bestIndices;
	uint bestInlierCount = 0;
	float distance;
	cout << "Height is: " << cloud->height << "Width is : " << cloud->width << std::endl;
	auto eigen_cloud = cloud->getMatrixXfMap().block(0,0, 3, cloud->size());
	Eigen::Hyperplane<float, 3> plane;
	for (int index = 0; index < maxIterations; index++){
		// gen random plane
		auto s1 = std::chrono::high_resolution_clock::now(); 
		auto p1 = eigen_cloud.col(std::rand()%cloudLen);
		auto p2 = eigen_cloud.col(std::rand()%cloudLen);
		auto p3 = eigen_cloud.col(std::rand()%cloudLen);
		plane = std::move(Eigen::Hyperplane<float, 3>::Through(p1, p2 ,p3));
		// Line line = {p1.y - p2.y, p2.x - p1.x, p1.x*p2.y -p2.x * p1.y};
		// Get distance from plane to all points
		std::unordered_set<int> inlier_indices;
		// Find inliers
		for (int idx = 0; idx < cloud->points.size(); idx++){
			if ((distance = plane.absDistance(eigen_cloud.col(idx))) < distanceTol){
				inlier_indices.insert(idx);
			}
			// cout << "Distance was " << distance << endl;
			// cout << "Size is " << inlier_indices.size() << endl;
		}
		if (bestInlierCount < inlier_indices.size()){
			bestInlierCount = inlier_indices.size();
			bestIndices = std::move(inlier_indices);
		}
		cout << "Best ind are " << bestIndices.size() << endl;
		auto s2 = std::chrono::high_resolution_clock::now(); 
		duration = std::chrono::duration_cast<std::chrono::microseconds>(s2 - s1) * 0.001; 
		cout << "Ours took " <<duration.count() << endl; 
	}
	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	
	return bestIndices;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 200, .33);
	cout << "Inliers are " << inliers.size() << endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
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
