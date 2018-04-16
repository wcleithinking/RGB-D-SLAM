#pragma once

/* Header Files */
// C standard libraries
#include <fstream>
#include <vector>
using namespace std;

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
// OpenCV
#include <opencv2/features2d/features2d.hpp>
// #include <opencv2/nonfree/nonfree.hpp> //use this if you want to use SIFT or SURF
#include <opencv2/calib3d/calib3d.hpp>


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

// Frame
struct FRAME
{
	cv::Mat rgb, depth;
	cv::Mat desp;
	vector<cv::KeyPoint> kp;
};

// PnP Result
struct RESULT_OF_PNP
{
	cv::Mat rvec, tvec;
	int inliers;
};


/* Class */
class ParameterReader
{
public:
	ParameterReader( string filename="./parameters.txt" )
	{
		ifstream fin( filename.c_str() );
		if (!fin)
		{
			cerr<<"parameter file does not exist."<<endl;
			return;
		}
		while (!fin.eof())
		{
			string str;
			getline( fin, str );
			if (str[0] == '#')
				continue;
			int pos = str.find("=");
			if (pos == -1)
				continue;
			string key = str.substr( 0, pos );
			string value = str.substr( pos+1, str.length() );
			data[key] = value;

			if (!fin.good())
			break;
		}
	}
	
	string getData( string key )
	{
		map<string, string>::iterator iter = data.find(key);
		if (iter == data.end())
		{
			cerr<<"Parameter name"<<key<<" not be found!"<<endl;
			return string("NOT_FOUND");
		}
		return iter->second;
	}
public:
	map<string,string> data;
};



/* Functions */

// inputs: the cv::Mat(s) correspond to images rgb and depth and the camera parameters
// return: a pointer pointing to a point cloud given by the function
PointCloud::Ptr image2PointCloud( cv::Mat &rgb, cv::Mat &depth, CAMERA_INTRINSIC_PARAMETERS &camera );

// inputs: a 3d point (u,v,d) (in the image coordinate) and the camera parameter
// return: a 3d point (x,y,z) (in the space coordinate)
cv::Point3f point2dTo3d( cv::Point3f &point, CAMERA_INTRINSIC_PARAMETERS &camera );

//
void computeKeyPointsAndDesp( FRAME &frame, string detector, string descriptor );

//
RESULT_OF_PNP estimateMotion( FRAME &frame1, FRAME &frame2, CAMERA_INTRINSIC_PARAMETERS &camera );


