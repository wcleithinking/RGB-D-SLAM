#pragma once

/* Header Files */
// C standard libraries
#include <fstream>
#include <vector>
using namespace std;

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL
#include <pcl-1.7/pcl/io/pcd_io.h>
#include <pcl-1.7/pcl/point_types.h>
//#include <pcl-1.7/pcl/point_cloud.h>

// Typedef
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// Camera Parameters
struct CAMERA_INTRINSIC_PARAMETERS
{
	double cx, cy, fx, fy, scale;
};

/* Functions */
// inputs: the cv::Mat(s) correspond to images rgb and depth and the camera parameters
// return: a pointer pointing to a point cloud given by the function
PointCloud::Ptr image2PointCloud( cv::Mat &rgb, cv::Mat &depth, CAMERA_INTRINSIC_PARAMETERS &camera );

// inputs: a 3d point (u,v,d) (in the image coordinate) and the camera parameter
// return: a 3d point (x,y,z) (in the space coordinate)
cv::Point3f point2dTo3d( cv::Point3f &point, CAMERA_INTRINSIC_PARAMETERS &camera );
