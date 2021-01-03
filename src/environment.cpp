/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
    Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
    Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
    Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if (renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene = true;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    std::shared_ptr<Lidar> lidar = std::make_shared<Lidar>(cars, 0);
    // TODO(etragas) Figure out how scan and render work
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud = lidar->scan();
    // renderRays(viewer, lidar->position, pointCloud);
    // renderPointCloud(viewer, pointCloud, std::string("name"), Color(0, 1, 0));
    ProcessPointClouds<pcl::PointXYZ> cloudProcess;
    pcl::PointCloud<pcl::PointXYZ>::Ptr planeCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr segmentCloud;
    std::tie(planeCloud, segmentCloud) = cloudProcess.SegmentPlane(pointCloud, 100, 0.2);
    // renderPointCloud(viewer, planeCloud, std::string("plane"), Color(0, 1, 0));
    // renderPointCloud(viewer, segmentCloud, std::string("obstacle"), Color(1, 0, 0));
    // TODO:: Create lidar sensor

    // TODO:: Create point processor
    auto pointProcessor = ProcessPointClouds<pcl::PointXYZ>();
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud, 5, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId]);
        // Box box = pointProcessor.BoundingBox(cluster);
        // renderBox(viewer,box,clusterId);
        ++clusterId;
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZI> *pointProcessor, const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    // Experiment with the ? values and find what works best
    // What does filterres do?
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessor->FilterCloud(inputCloud, .1, Eigen::Vector4f(-25, -6, -10, 1), Eigen::Vector4f(25, 6, 10, 1));
    std::cout << "cluster size ";
    pcl::PointCloud<pcl::PointXYZI>::Ptr planeCloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr segmentCloud;
    std::tie(planeCloud, segmentCloud) = pointProcessor->SegmentPlane(filterCloud, 100, -1.9);
    renderPointCloud(viewer, planeCloud, std::string("plane"), Color(0, 1, 0));
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud, 2, 20, 500);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1), Color(1, 1, 1), Color(0, 1, 1), Color(1, 0, 1)};
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        pointProcessor->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % 6]);
        Box box = pointProcessor->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        ++clusterId;
    }
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    viewer->setBackgroundColor(0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch (setAngle)
    {
    case XY:
        viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
        break;
    case TopDown:
        viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
        break;
    case Side:
        viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
        break;
    case FPS:
        viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if (setAngle != FPS)
        viewer->addCoordinateSystem(1.0);
}

int main(int argc, char **argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    // cityBlock(viewer, pointProcessorI, inputCloudI);
    while (!viewer->wasStopped())
    {

        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce();
    }
}