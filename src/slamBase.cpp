#include "slamBase.h"

PointCloud::Ptr image2PointCloud( cv::Mat &rgb, cv::Mat &depth, CAMERA_INTRINSIC_PARAMETERS &camera )
{
	PointCloud::Ptr cloud( new PointCloud );	// cloud is a pointer pointing to an empty point cloud
	for (int m = 0; m < depth.rows; m++)
	{
		for (int n = 0; n < depth.cols; n++)
		{
			// get the depth at (m,n)
			unsigned short d = depth.ptr<unsigned short>(m)[n];
			// check the value of d
			// if d==0, do not add point in the cloud and turn to the next forloop 
			if (d == 0)
				continue;
			// else, add a point p and its attributes
			PointT p;
			// (x,y,z)
			p.z = double(d) / camera.scale;
			p.x = (n - camera.cx)*p.z / camera.fx;
			p.y = (m - camera.cy)*p.z / camera.fy;
			// (r,g,b)
			p.b = rgb.ptr<unsigned short>(m)[n*3];
			p.g = rgb.ptr<unsigned short>(m)[n*3+1];
			p.g = rgb.ptr<unsigned short>(m)[n*3+2];
			// add p to the cloud
			cloud->points.push_back( p );
		}
	}
	// set the attributes of the cloud
	cloud->height = 1;
	cloud->width = cloud->points.size();
	cloud->is_dense = false;
	// return
	return cloud;
}

cv::Point3f point2dTo3d( cv::Point3f &point, CAMERA_INTRINSIC_PARAMETERS &camera)
{
	cv::Point3f p;
	p.z = double (point.z) / camera.scale;
	p.x = ( point.x - camera.cx) * p.z / camera.fx;
	p.y = ( point.y - camera.cy) * p.z / camera.fy;
	return p;
}
