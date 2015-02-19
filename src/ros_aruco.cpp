/*****************************************************************************************
Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

ROS bits and integration added by Florian Lier flier at techfak dot uni-bielefeld dot de

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
********************************************************************************************/
#include <iostream>
#include <fstream>
#include <sstream>
#include <math.h>
#include <unistd.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "aruco.h"
#include "cvdrawingutils.h"


using namespace cv;
using namespace aruco;
string TheInputVideo;
string TheIntrinsicFile;
float TheMarkerSize=-1;
int ThePyrDownLevel;
MarkerDetector MDetector;
VideoCapture TheVideoCapturer;
vector<Marker> TheMarkers;
Mat TheInputImage,TheInputImageCopy;
CameraParameters TheCameraParameters;
void cvTackBarEvents(int pos,void*);
bool readCameraParameters(string TheIntrinsicFile,CameraParameters &CP,Size size);
// Determines the average time required for detection
pair<double,double> AvrgTime(0,0) ;
double ThresParam1,ThresParam2;
int iThresParam1,iThresParam2;
int waitTime=0;
bool update_images;

void test_recv_img(const sensor_msgs::ImageConstPtr& msg) {
    cout << "received image" << endl;
}

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  Mat img;
  bool is_new;

public:
  ImageConverter()
      : it_(nh_), is_new(false)
  {
    // subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_raw", 1, &ImageConverter::imageCb, this);
  }

  ~ImageConverter()
  {
   printf("Stopped Image Import");
  }

  cv::Mat getCurrentImage() {
         is_new = false;
        return img;
  }

  bool isImageNew() {
      return is_new;
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    img = cv_ptr->image;
    is_new = true;
  }

};

bool readArguments ( int argc,char **argv )
{
    if (argc<2) {
        cerr<< "Invalid number of arguments" <<endl;
        cerr<< "Usage: (in.avi|live|copy) [intrinsics.yml] [size]" <<endl;
        return false;
    }
    TheInputVideo=argv[1];
    if (argc>=3)
        TheIntrinsicFile=argv[2];
    if (argc>=4)
        TheMarkerSize=atof(argv[3]);
    if (argc==3)
        cerr<< "NOTE: You need makersize to see 3d info!" <<endl;
    return true;
}


// XX (mtourne): doesnt' convert nicely to boost type
// static const double COV[] = {0.1, 0, 0, 0, 0, 0,
//                              0, 0.1, 0, 0, 0, 0,
//                              0, 0, 0.1, 0, 0, 0,
//                              0, 0, 0, 99999, 0, 0,  // large covariance on rot x
//                              0, 0, 0, 0, 99999, 0,  // large covariance on rot y
//                              0, 0, 0, 0, 0, 99999};  // large covariance on rot z

int main(int argc,char **argv){
	// Show images, press "SPACE" to diable image
    // rendering to save CPU time
    update_images = true;

	if (readArguments(argc,argv)==false) {
		return 0;
	}

    // XX (read straight from openc)
	// TheVideoCapturer.open(0);
	// // Check video is open
	// if (!TheVideoCapturer.isOpened()) {
	// 	cerr<<"Could not open video"<<endl;
	// 	return 0;
	// }


	// ROS messaging init
	ros::init(argc, argv, "aruco_ros");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);

	// Read first image to get the dimensions
	// TheVideoCapturer>>TheInputImage;
    ImageConverter image_topic;

    while (!image_topic.isImageNew()) {
        ros::spinOnce();
    }
    cout << "Got first image!!" << endl;
    TheInputImage = image_topic.getCurrentImage();

	// Read camera parameters if passed
	if (TheIntrinsicFile!="") {
		TheCameraParameters.readFromXMLFile(TheIntrinsicFile);
		TheCameraParameters.resize(TheInputImage.size());
	}

	// Configure other parameters
	if (ThePyrDownLevel>0)
		MDetector.pyrDown(ThePyrDownLevel);

	// Create gui
	cv::namedWindow("THRESHOLD IMAGE",1);
	cv::namedWindow("INPUT IMAGE",1);

	MDetector.getThresholdParams( ThresParam1,ThresParam2);
	MDetector.setCornerRefinementMethod(MarkerDetector::LINES);

	iThresParam1=ThresParam1;
	iThresParam2=ThresParam2;
	cv::createTrackbar("ThresParam1", "INPUT IMAGE",&iThresParam1, 13, cvTackBarEvents);
	cv::createTrackbar("ThresParam2", "INPUT IMAGE",&iThresParam2, 13, cvTackBarEvents);
	char key=0;
	int index=0;

    // Odometry publisher
	ros::Publisher pub = n.advertise<nav_msgs::Odometry>("/vo", 1000);
    double position[3], orientation[4];
    nav_msgs::Odometry odom;
    odom.header.seq = index;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom_combined";
    odom.child_frame_id = "base_footprint";
    odom.pose.covariance = {0.1, 0, 0, 0, 0, 0,
                            0, 0.1, 0, 0, 0, 0,
                            0, 0, 0.1, 0, 0, 0,
                            0, 0, 0, 99999, 0, 0,  // large covariance on rot x
                            0, 0, 0, 0, 99999, 0,  // large covariance on rot y
                            0, 0, 0, 0, 0, 99999};  // large covariance on rot z


	// Capture until press ESC or until the end of the video
	while ((key != 'x') && (key!=27) && ros::ok()){

        ros::spinOnce();

		key=cv::waitKey(1);

        // If space is hit, don't render the image.
		if (key == ' '){
			update_images = !update_images;
		}

        if (!image_topic.isImageNew()) {
            continue;
        }

        TheInputImage = image_topic.getCurrentImage();
        index++; // Number of images captured
        double tick = (double)getTickCount();// For checking the speed

        // Detection of markers in the image passed
        MDetector.detect(TheInputImage,TheMarkers,TheCameraParameters,TheMarkerSize);

        // Check the speed by calculating the mean speed of all iterations
        AvrgTime.first+=((double)getTickCount()-tick)/getTickFrequency();
        AvrgTime.second++;

        // Show the detection time
        // cout<<"Time detection="<<1000*AvrgTime.first/AvrgTime.second<<" milliseconds"<<endl;

        // Publish the markers
        for (unsigned int i=0;i<TheMarkers.size();i++) {
            cout<<TheMarkers[i]<<endl;
            TheMarkers[i].draw(TheInputImageCopy,Scalar(0,0,255),1);
            TheMarkers[i].OgreGetPoseParameters(position, orientation);
            geometry_msgs::Point p;
            p.x = position[0];
            p.y = position[1];
            p.z = position[2];
            odom.pose.pose.position = p;
            geometry_msgs::Quaternion q;
            q.x = orientation[0];
            q.y = orientation[1];
            q.z = orientation[2];
            q.w = orientation[3];
            odom.pose.pose.orientation = q;
            pub.publish(odom);
        }

        // Copy image
        TheInputImage.copyTo(TheInputImageCopy);
        if (TheCameraParameters.isValid())
            for (unsigned int i=0;i<TheMarkers.size();i++) {
                CvDrawingUtils::draw3dCube(TheInputImageCopy,TheMarkers[i],TheCameraParameters);
                CvDrawingUtils::draw3dAxis(TheInputImageCopy,TheMarkers[i],TheCameraParameters);
            }

        if (update_images) {
            cv::imshow("INPUT IMAGE",TheInputImageCopy);
            cv::imshow("THRESHOLD IMAGE",MDetector.getThresholdedImage());
        }
	}
}

void checkbox_callback(bool value){
	update_images = value;
}

void cvTackBarEvents(int pos,void*)
{
    if (iThresParam1<3) iThresParam1=3;
    if (iThresParam1%2!=1) iThresParam1++;
    if (ThresParam2<1) ThresParam2=1;
    ThresParam1=iThresParam1;
    ThresParam2=iThresParam2;
    MDetector.setThresholdParams(ThresParam1,ThresParam2);

    // Recompute
    MDetector.detect(TheInputImage,TheMarkers,TheCameraParameters);
    TheInputImage.copyTo(TheInputImageCopy);

    for (unsigned int i=0;i<TheMarkers.size();i++) TheMarkers[i].draw(TheInputImageCopy,Scalar(0,0,255),1);

    // Print other rectangles that contains no valid markers
    /* for (unsigned int i=0;i<MDetector.getCandidates().size();i++) {
        aruco::Marker m( MDetector.getCandidates()[i],999);
        m.draw(TheInputImageCopy,cv::Scalar(255,0,0));
    } */

    // Draw a 3d cube in each marker if there is 3d info
    if (TheCameraParameters.isValid())
        for (unsigned int i=0;i<TheMarkers.size();i++)
            CvDrawingUtils::draw3dCube(TheInputImageCopy,TheMarkers[i],TheCameraParameters);

    cv::imshow("INPUT IMAGE",TheInputImageCopy);
    cv::imshow("THRESHOLD IMAGE",MDetector.getThresholdedImage());
}
