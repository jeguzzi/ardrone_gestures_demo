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
using namespace std;
using namespace cv;

namespace enc = sensor_msgs::image_encodings;

//Define color thresholds
//Yellow
int Hmin = -1;
int Hmax = -1;
int Smin = -1;
int Smax = -1;
int Vmin = -1;
int Vmax = -1;

bool published_start_v = false;
//Green
/*int Hmin_G = 45;
int Hmax_G = 70;
int Smin_G = 70;
int Smax_G = 255;
int Vmin_G = 10;
int Vmax_G = 190;
//Orange
int Hmin_O = 0;
int Hmax_O = 18;
int Smin_O = 160;
int Smax_O = 255;
int Vmin_O = 130;
int Vmax_O = 255;*/

//calibration variables

int x_to_handle;
int y_to_handle;
bool handled = true;
bool send_hsv = false;


void hsv_calib(IplImage* image){
	CvScalar c;
	c = cvGetAt(image, y_to_handle, x_to_handle); //row i, column j
	
	//the following are doubles
	int h = (int)c.val[0]; //H
	int s = (int)c.val[1]; //S
	int v = (int)c.val[2]; //V
	
	if (Hmin == -1){
		Hmin = h;
		Hmax = h;
		Smin = s;
		Smax = s;
		Vmin = v;
		Vmax = v;
	}

	//adjust
	if (h < Hmin){
		Hmin = h;
	}
	else if (h > Hmax){
		Hmax = h;
	}
	
	if (s < Smin){
		Smin = s;
	}
	else if (s > Smax){
		Smax = s;
	}

	if (v < Vmin){
		Vmin = v;
	}
	else if (v > Vmax){
		Vmax = v;
	}
	send_hsv = true;

}

int process_image(cv::Mat inputImage) {
		// Convert from Mat to IplImage+
		IplImage* image = cvCloneImage(&(IplImage)inputImage);
		cvShowImage("Original_image", image); 

		// THRESH HOLDING //
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//// Segment the current image ////
		IplImage *imageHSV					= cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 3);
		IplImage *imageColor				= cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);   // Grayscale output image

		// Convert RGB to HSV format //
		cvCvtColor(image, imageHSV, CV_BGR2HSV);
		
		//handle the calibration event
		if (!handled){
			hsv_calib(imageHSV);
			handled = true;
		}
		
		if (Hmin != -1){
		// Segement Color //
		cvInRangeS(imageHSV, cvScalar(Hmin, Smin, Vmin), cvScalar(Hmax, Smax, Vmax), imageColor); // Color threshold
               
        cvErode(imageColor, imageColor, NULL, 1); 
        cvDilate(imageColor, imageColor, NULL, 2); 
        cvErode(imageColor, imageColor, NULL, 1);

		cvShowImage("Segmented_image", imageColor);
	
		} 
		cvReleaseImage(&image);
		cvReleaseImage(&imageHSV);
		cvReleaseImage(&imageColor);
		return 0; 
}

class Detector{ 
  	ros::NodeHandle nh_;
  	image_transport::ImageTransport it_;
  	image_transport::Subscriber image_sub_;
	//calibration topics
    ros::Subscriber hmin_sub;
    ros::Subscriber hmax_sub;
    ros::Subscriber smin_sub;
    ros::Subscriber smax_sub;
    ros::Subscriber vmin_sub;
    ros::Subscriber vmax_sub;
	
	//coherence with python gui
	ros::Publisher hmin_pub;
    ros::Publisher hmax_pub;
    ros::Publisher smin_pub;
    ros::Publisher smax_pub;
    ros::Publisher vmin_pub;
    ros::Publisher vmax_pub;
	
	ofstream* fs;	
  
public: Detector(ofstream* fs):it_(nh_){

    		image_sub_ = it_.subscribe("/ardrone/image_raw", 1, &Detector::imageCb, this);

			hmin_sub = nh_.subscribe("/ardrone/Hmin", 1, &Detector::receive_hmin, this);
            hmax_sub = nh_.subscribe("/ardrone/Hmax", 1, &Detector::receive_hmax, this);
			smin_sub = nh_.subscribe("/ardrone/Smin", 1, &Detector::receive_smin, this);
			smax_sub = nh_.subscribe("/ardrone/Smax", 1, &Detector::receive_smax, this);
			vmin_sub = nh_.subscribe("/ardrone/Vmin", 1, &Detector::receive_vmin, this);
			vmax_sub = nh_.subscribe("/ardrone/Vmax", 1, &Detector::receive_vmax, this);

			hmin_pub = nh_.advertise<std_msgs::Int16>("/ardrone/Hminc", 1);
			hmax_pub = nh_.advertise<std_msgs::Int16>("/ardrone/Hmaxc", 1);
			smin_pub = nh_.advertise<std_msgs::Int16>("/ardrone/Sminc", 1);
			smax_pub = nh_.advertise<std_msgs::Int16>("/ardrone/Smaxc", 1);
			vmin_pub = nh_.advertise<std_msgs::Int16>("/ardrone/Vminc", 1);
			vmax_pub = nh_.advertise<std_msgs::Int16>("/ardrone/Vmaxc", 1);
			this->fs = fs;
  		}

		~Detector(){
			*fs << Hmin << " " << Hmax << " " << Smin << " " << Smax << " " << Vmin << " " << Vmax;
			(*fs).close();
			cvDestroyWindow( "Segmented_image");
			cvDestroyWindow("Original_image");
		}
		
		void publish_hsv(){
			std_msgs::Int16 v;
        	v.data = Hmin;
			hmin_pub.publish(v);
			v.data = Hmax;
			hmax_pub.publish(v);
			v.data = Smin;
			smin_pub.publish(v);
			v.data = Smax;
			smax_pub.publish(v);
			v.data = Vmin;
			vmin_pub.publish(v);
			v.data = Vmax;
			vmax_pub.publish(v);
			cout << "published HSV " << endl;
		}		
		
		void receive_hmin(const std_msgs::Int16& msg){
			Hmin = msg.data;
		}

		void receive_hmax(const std_msgs::Int16& msg){
			Hmax = msg.data;
		}

		void receive_smin(const std_msgs::Int16& msg){
			Smin = msg.data;
		}
		
		void receive_smax(const std_msgs::Int16& msg){
			Smax = msg.data;
		}

		void receive_vmin(const std_msgs::Int16& msg){
			Vmin = msg.data;
		}	
		
		void receive_vmax(const std_msgs::Int16& msg){
			Vmax = msg.data;
		}					  

  void imageCb(const sensor_msgs::ImageConstPtr& msg){

	if (Hmin != -1 and !published_start_v){
		published_start_v = true;
		publish_hsv();
	}
	cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
	Mat image=cv_ptr->image;
    //ROS_INFO("New image");
    //initialization
    try{
    	//cout << "Image detected.\n";
    	process_image(image);
		//release memory
		image.release();
		if (send_hsv){
			send_hsv = false;
			publish_hsv();
		}

      }catch (cv_bridge::Exception& e){
	  	ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
      }
  
}
};

void mouseHandler(int event, int x, int y, int flags, void* param){
	if (event == CV_EVENT_LBUTTONDOWN){
    	x_to_handle = x;
	    y_to_handle = y;
		//cout << x << endl;
		handled = false;
	}
}

int main(int argc, char** argv)
{   
	char* filename;
	if (argc > 1) filename = argv[1];
	else filename = "def_calib.txt";
	ofstream fs(filename); 
	if(!fs){
        std::cerr<<"Cannot open the output file. Exiting."<<std::endl;
        return 1;
	}
	
	ros::init(argc, argv, "detector_node");
	ROS_INFO("Launch detector");
	cvNamedWindow("Segmented_image", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Original_image", CV_WINDOW_AUTOSIZE);
	setMouseCallback("Original_image" , mouseHandler, 0);
	cvStartWindowThread(); 
    Detector ic(&fs);
    ROS_INFO("Done and now listen for incoming images");
    ros::spin();
    return 0;
}
