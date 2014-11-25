#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "std_msgs/Int16.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "limits.h"
#include "float.h"
#include "time.h"
#include "cv.h"
#include "highgui.h"
#include "svm.h"
#include "BlobResult.h"
#include "Blob.h"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"
#include <sys/stat.h>
#include <vector>
#include <fstream>
#include <string>
using namespace std;
using namespace cv;

namespace enc = sensor_msgs::image_encodings;

string tag = "/home/drone2/Pictures/Dataset/DATASET_";
int coo;

class Grabber{

    ros::NodeHandle nh_;
  	image_transport::ImageTransport it_; 
    image_transport::Subscriber image_sub_;
  
public: Grabber(int a):it_(nh_){
            cout << "Subscribing " << a <<  endl;
    		image_sub_ = it_.subscribe("/ardrone/image_raw", 1, &Grabber::imageCb, this);
  		}

		~Grabber(){
			cvDestroyWindow("Original_image");
		}
		
  void imageCb(const sensor_msgs::ImageConstPtr& msg){
    //cout << "Received image " << endl;
	cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
	Mat image=cv_ptr->image;
    IplImage* iplimage = cvCloneImage(&(IplImage)image); 
    cvShowImage("Original_image", iplimage); 
    //ROS_INFO("New image");
    //initialization
    try{
    	//cout << "Image detected.\n";
        stringstream ss;
        ss << coo;
        string num = ss.str();
        string filename = tag + num + ".jpeg";
        cout << filename << endl;
    	cvSaveImage(filename.c_str(), iplimage); 
		//release memory
        cvReleaseImage(&iplimage);
		image.release();
      }catch (cv_bridge::Exception& e){
	  	ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
      }
     coo++;
  
}
};

int main(int argc, char** argv)
{   
	if (argc > 1) {
    string n(argv[1]);
    tag = tag + n + "_";
    }
	else tag = tag + "DEF_";
    cout << tag << endl;
	coo = 0;
	ros::init(argc, argv, "grabber_node");
	ROS_INFO("Launch grabber");
	cvNamedWindow("Original_image", CV_WINDOW_AUTOSIZE);
	cvStartWindowThread();
    cout << "OK" << endl; 
    Grabber grab(10);
    ROS_INFO("Done and now listen for incoming images");
    ros::spin();
    return 0;
}
