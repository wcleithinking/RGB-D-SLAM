#include <iostream>
#include "slamBase.h"
using namespace std;

// OpenCV
#include <opencv2/features2d/features2d.hpp>
// #include <opencv2/nonfree/nonfree.hpp> //use this if you want to use SIFT or SURF
#include <opencv2/calib3d/calib3d.hpp>

int main( int argc, char **argv )
{
	// read images
	cv::Mat rgb1 = cv::imread( "/home/wenchao/RGB-D-SLAM/data/rgb1.png" );
	cv::Mat rgb2 = cv::imread( "/home/wenchao/RGB-D-SLAM/data/rgb2.png" );
	cv::Mat depth1 = cv::imread( "/home/wenchao/RGB-D-SLAM/data/depth1.png", -1 );
	cv::Mat depth2 = cv::imread( "/home/wenchao/RGB-D-SLAM/data/depth2.png", -1 );
	
	// declare the detector and descriptor
	cv::Ptr<cv::FeatureDetector> detector;
	cv::Ptr<cv::DescriptorExtractor> descriptor;
	
	// create the detector and descriptor
	detector = cv::FeatureDetector::create("ORB");
	descriptor = cv::DescriptorExtractor::create("ORB");
	// the nonfree case with SIFT or SURF
	// cv::initModule_nonfree();
	// _detector = cv::FeatureDetector::create( "SIFT" )
	// _descriptor = cv::DescriptorExtractor::create( "SIFT" )

	// decalre the keypoints
	vector< cv::KeyPoint > kp1, kp2;
	// get the keypoints of images
	detector->detect( rgb1, kp1 );
	detector->detect( rgb2, kp2 );
	
	cout<<"Key points of two images: "<<kp1.size()<<","<<kp2.size()<<endl;

	// show the keypoints in the images
	cv::Mat imgShow;
	cv::drawKeypoints( rgb1, kp1, imgShow, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
	cv::imshow( "keypoints", imgShow );
	cv::imwrite( "/home/wenchao/RGB-D-SLAM/data/keypoints.png", imgShow );
	cv::waitKey(0); // pause and wait a press-key
	
	// compute the descriptor
	cv::Mat desp1, desp2;
	descriptor->compute( rgb1, kp1, desp1 );
	descriptor->compute( rgb2, kp2, desp2 );
	
	// match the descriptors desp1 and desp2
	vector< cv::DMatch > matches;
	cv::BFMatcher matcher;
	matcher.match( desp1, desp2, matches );
	cout<<"Find total "<<matches.size()<<" matches."<<endl;

	// show the matches in the images
	cv::Mat imgMatches;
	cv::drawMatches( rgb1, kp1, rgb2, kp2, matches, imgMatches );
	cv::imshow( "matches", imgMatches );
	cv::imwrite( "/home/wenchao/RGB-D-SLAM/data/matches.png", imgMatches );
	cv::waitKey( 0 );
	
	// filter the matches
	vector< cv::DMatch > goodMatches;
	double minDis = 9999;
	for ( size_t i = 0; i < matches.size(); i++ )
	{
		if ( matches[i].distance < minDis )
			minDis = matches[i].distance;
	}	
	cout<<"min dis of the matches  = "<<minDis<<endl;
	for (size_t i = 0; i < matches.size(); i++)
	{
		if ( matches[i].distance < 10*minDis )
			goodMatches.push_back( matches[i] );
	}
	
	// show the good matches in the images
	cout<<"good matches = "<<goodMatches.size()<<endl;
	cv::drawMatches( rgb1, kp1, rgb2, kp2, goodMatches, imgMatches );
	cv::imshow( " good matches", imgMatches );
	cv::imwrite( "/home/wenchao/RGB-D-SLAM/data/good_matches.png", imgMatches );
	cv::waitKey( 0 );
	
	/********************************************************************
		solve the rotation and translation by using cv::solvePnPRansac()
	*********************************************************************/
	// prepare
	vector< cv::Point3f > pts_obj;	// the first frame
	vector< cv::Point2f > pts_img;	// the second frame
	
	CAMERA_INTRINSIC_PARAMETERS C;
	C.cx = 325.5;
	C.cy = 253.5;
	C.fx = 518.0;
	C.fy = 519.0;
	C.scale = 1000.0;
	
	for ( size_t i = 0; i < goodMatches.size(); i++  )
	{
		// query is the fisrt frame and train is the second frame
		cv::Point2f p = kp1[ goodMatches[i].queryIdx ].pt;
		// x is the column and y is the row
		unsigned short d = depth1.ptr<unsigned short>( int(p.y) )[ int(p.x) ];
		if (d == 0)
			continue;
		// convert the image coordinate to the space coordinate by using point2dTo3d()
		cv::Point3f pt ( p.x, p.y, d );
		cv::Point3f pd = point2dTo3d( pt, C );
		// get pts_obj
		pts_obj.push_back( pd );
		// get pts_img
		pts_img.push_back( cv::Point2f( kp2[ goodMatches[i].trainIdx ].pt ) );
	}
	
	double camera_matrix_data[3][3] = {
		{C.fx, 0, C.cx},
		{0, C.fy, C.cy},
		{0, 0, 1}
	};

	// construct the camera matrix
	cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );
	cv::Mat vec_rotation, vec_translation, inliers;
	
	// solve the PnP
	cv::solvePnPRansac( pts_obj, pts_img, cameraMatrix, cv::Mat(), vec_rotation, vec_translation, false, 100, 1.0, 100, inliers );
	
	cout<<"inliers: "<<inliers.rows<<endl;
	cout<<"R = "<<vec_rotation<<endl;
	cout<<"P = "<<vec_translation<<endl;

	// plot the inliers match figure
	vector< cv::DMatch > matchesShow;
	for ( size_t i = 0; i < inliers.rows; i++)
	{
		matchesShow.push_back( goodMatches[ inliers.ptr<int>(i)[0] ] );
	}
	cv::drawMatches( rgb1, kp1, rgb2, kp2, matchesShow, imgMatches );
	cv::imshow( "inlier matches", imgMatches );
	cv::imwrite( "/home/wenchao/RGB-D-SLAM/data/inliers.png", imgMatches );
	cv::waitKey( 0 );

	return 0;

} 
