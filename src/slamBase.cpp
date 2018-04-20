#include "../include/slamBase.h"

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

void computeKeyPointsAndDesp( FRAME &frame, string detector, string descriptor )
{
	// declare the detector and descriptor
	cv::Ptr<cv::FeatureDetector> _detector;
	cv::Ptr<cv::DescriptorExtractor> _descriptor;

	// init the detector and descriptor
	//	cv::initModule_nonfree();
	_detector = cv::FeatureDetector::create( detector.c_str() );
	_descriptor = cv::DescriptorExtractor::create( descriptor.c_str() );
	if (!_detector || !_descriptor)
	{
		cerr<<"Unknown detector or discriptor type !"<<detector<<","<<descriptor<<endl;
		return;
	}

	// get the keypoints of images
	_detector->detect( frame.rgb, frame.kp );
	_descriptor->compute( frame.rgb, frame.kp, frame.desp );

	return;

}

RESULT_OF_PNP estimateMotion( FRAME &frame1, FRAME &frame2, CAMERA_INTRINSIC_PARAMETERS &camera)
{
	static ParameterReader pd;

	// match the descriptors desp1 and desp2
	vector< cv::DMatch > matches;
	cv::BFMatcher matcher;
	matcher.match( frame1.desp, frame2.desp, matches );
	cout<<"Find total "<<matches.size()<<" matches."<<endl;

	
	// filter the matches
	vector< cv::DMatch > goodMatches;
	double minDis = 9999;
	double good_match_threshold = atof( pd.getData( "good_match_threshold" ).c_str() );
	for ( size_t i = 0; i < matches.size(); i++ )
	{
		if ( matches[i].distance < minDis )
			minDis = matches[i].distance;
	}	
	//	cout<<"min dis of the matches  = "<<minDis<<endl;
	for (size_t i = 0; i < matches.size(); i++)
	{
		if ( matches[i].distance < good_match_threshold*minDis )
			goodMatches.push_back( matches[i] );
	}
	cout<<"good matches = "<<goodMatches.size()<<endl;
	
	/********************************************************************
		solve the rotation and translation by using cv::solvePnPRansac()
	*********************************************************************/
	// prepare
	vector< cv::Point3f > pts_obj;	// the first frame
	vector< cv::Point2f > pts_img;	// the second frame
	
	
	for ( size_t i = 0; i < goodMatches.size(); i++  )
	{
		// query is the fisrt frame and train is the second frame
		cv::Point2f p = frame1.kp[ goodMatches[i].queryIdx ].pt;
		// x is the column and y is the row
		unsigned short d = frame1.depth.ptr<unsigned short>( int(p.y) )[ int(p.x) ];
		if (d == 0)
			continue;
		// convert the image coordinate to the space coordinate by using point2dTo3d()
		cv::Point3f pt ( p.x, p.y, d );
		cv::Point3f pd = point2dTo3d( pt, camera );
		// get pts_obj
		pts_obj.push_back( pd );
		// get pts_img
		pts_img.push_back( cv::Point2f( frame2.kp[ goodMatches[i].trainIdx ].pt ) );
	}
	
	double camera_matrix_data[3][3] = {
		{camera.fx, 0, camera.cx},
		{0, camera.fy, camera.cy},
		{0, 0, 1}
	};

		
	cout<<"sloving pnp"<<endl;

	// construct the camera matrix
	cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );
	cv::Mat rvec, tvec, inliers;
	
	// solve the PnP
	cv::solvePnPRansac( pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 1.0, 100, inliers );
	
	RESULT_OF_PNP result;
	result.rvec = rvec;
	result.tvec = tvec;
	result.inliers = inliers.rows;

	return result;

}

