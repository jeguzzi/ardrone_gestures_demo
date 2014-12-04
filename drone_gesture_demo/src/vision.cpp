#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Empty.h"
#include "std_msgs/String.h"
#include "ar_track_alvar_msgs_idsia/AlvarMarkers.h"
#include "ar_track_alvar_msgs_idsia/AlvarMarker.h"
#include "ardrone_autonomy/LedAnim.h"
#include "ardrone_autonomy/FlightAnim.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "limits.h"
#include "float.h"
#include "time.h"
#include "cv.h"
#include "highgui.h"
#include "../cvBlobsLib/include/BlobResult.h"
#include "../cvBlobsLib/include/Blob.h"
//#include <cvblob.h>

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "pthread.h"
#include <sys/stat.h>
#include <boost/circular_buffer.hpp>
#include <semaphore.h>
#include <pthread.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <deque>
#include <map>
#include <cmath>
#include <fstream>
#include <gesture_messages/led.h>


#define AREA_TAKEOFF 1500
#define PERC_OK 0.7



#define TIME_CL 1.5

#define TIME_FEEDBACK 2.5

#define MARKER_1 7
#define MARKER_2 16
#define MARKER_3 0
#define MARKER_4 1
#define MARKER_5 2
#define MARKER_6 3
#define MARKER_7 4

using namespace std;
using namespace cv;

int stage;
std::vector<int> vec_aux(6,0);
enum gesturestype{NO_BLOB, BIG_BLOB, TOP_RIGHT_BLOB, TOP_LEFT_BLOB, TOP_TWO_BLOBS, BOTTOM_TWO_BLOBS, BOTTOM_BLOB};
std::deque<gesturestype> gest_outputs (15,NO_BLOB); 
std::map<gesturestype,string> gesturesnames;
	//gesturesnames[TOP_RIGHT_BLOB] = top_right_blob;

struct MarkerImg{
	//if set to -1 means not detected.
	double xc;
	double yc;
    	double dim;
};


typedef boost::circular_buffer<MarkerImg> CircularBufferMarker;
CircularBufferMarker circularbufferMarker(10);

// Global vars
int cur_marker = -1;
CvPoint centroid1, centroid2;

//Define color thresholds
//Yellow
int Hmin = 15;
int Hmax = 170;
int Smin = 50;
int Smax = 255;
int Vmin = 50;
int Vmax = 255;

bool published_start_v = false;
bool send_hsv = false;

MarkerImg get_avg_xy(){
	double x = 0;
	double y = 0;
    	double dim = 0;
	double count = 0;
	 for(CircularBufferMarker::const_iterator it = circularbufferMarker.begin(); it != circularbufferMarker.end(); ++it){
        if ((*it).xc != -1){
			x += (*it).xc;
			y += (*it).yc;
		    dim += (*it).dim;
			count += 1;	
		}
    }
	if (count == 0) {
		MarkerImg m = {-1, -1, -1};
		return m;
	}else{
		MarkerImg m = {x/count, y/count, dim/count};
		return m;
	}
	
}

gesturestype process_image(cv::Mat inputImage, bool takenoff, bool marker_detected, double xc, double yc, double dim) {
		// Convert from Mat to IplImage+
		
		IplImage* image = cvCloneImage(&(IplImage)inputImage); 

		// THRESH HOLDING //
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//// Segment the current image ////
		IplImage *imageHSV = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 3);
		IplImage *imageColor = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);   // Grayscale output image

		// Convert RGB to HSV format //
		cvCvtColor(image, imageHSV, CV_BGR2HSV);
		cvShowImage("HSV image", imageHSV);
		

		// Segement Color //

		//Jerome Change -> hue value are periodic in [0,180] -> filter should be 0 - min AND max - 180 if max-min > 90
		//Alternatively we could use CV_BGR2HSV_FULL
		if(Hmax-Hmin>90){
		    IplImage *imageColorHigh=cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);
		    cvInRangeS(imageHSV, cvScalar(0, Smin, Vmin), cvScalar(Hmin, Smax, Vmax), imageColor);
			//cvShowImage("1", imageColor);

		    cvInRangeS(imageHSV, cvScalar(Hmax, Smin, Vmin), cvScalar(180, Smax, Vmax), imageColorHigh); 
			//cvShowImage("2", imageColorHigh);


		    cvOr(imageColorHigh,imageColor,imageColor);
		    cvReleaseImage(&imageColorHigh);
		  }
		else{
		    cvInRangeS(imageHSV, cvScalar(Hmin, Smin, Vmin), cvScalar(Hmax, Smax, Vmax), imageColor); // Color threshold
		  }

		// Segement Color //
		// cvInRangeS(imageHSV, cvScalar(Hmin, Smin, Vmin), cvScalar(Hmax, Smax, Vmax), imageColor); // Color threshold
               
        	cvErode(imageColor, imageColor, NULL, 1); 
        	cvDilate(imageColor, imageColor, NULL, 2); 
        	cvErode(imageColor, imageColor, NULL, 1);
		cvShowImage("Segmented_image", imageColor); 

		// Color blob //
		CBlobResult blobsColor;
		blobsColor = CBlobResult(imageColor, NULL, 0);
		CBlob ColorBlob1, ColorBlob2;
		//Get two biggest blobs (hopefully the gloves)
		blobsColor.GetNthBlob(CBlobGetArea(), 0, ColorBlob1);

		
		
		// Go on with centroids
		////////////////////////////////////////////////////////////////////////////////////////
		blobsColor.GetNthBlob(CBlobGetArea(), 1, ColorBlob2);
		centroid1.x = (ColorBlob1.MinX() + ColorBlob1.MaxX()) / 2;
		centroid1.y = (ColorBlob1.MinY() + ColorBlob1.MaxY()) / 2;

		centroid2.x = (ColorBlob2.MinX() + ColorBlob2.MaxX()) / 2;
		centroid2.y = (ColorBlob2.MinY() + ColorBlob2.MaxY()) / 2;

		cvReleaseImage(&image);
		cvReleaseImage(&imageHSV);
		cvReleaseImage(&imageColor);
		//free memory
		blobsColor.ClearBlobs();
		//inputImage.release();
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        	double min_area = dim*0.11;
		double max_area = dim*2.5;

		cout << "blob area "<< ColorBlob1.Area() <<endl;
		cout << "blob2 area "<< ColorBlob2.Area() <<endl;
		//Classify the gesture

			if (ColorBlob1.Area() > AREA_TAKEOFF){ //very close! -> takeoff
					return BIG_BLOB;
			}
			if (ColorBlob1.Area() > min_area and ColorBlob1.Area() < max_area){
			    	//at least one blob
				if(!marker_detected){			
					return NO_BLOB;
					cout << "MARKER NOT DETECTED "<<endl;
				}

			    	if (ColorBlob2.Area() > min_area and ColorBlob2.Area() < max_area){
				//two blobs seen
					if (centroid1.y < yc and centroid2.y > yc){
					    // 1 is top
					    // 2 is bottom
					    if (centroid1.x < xc) return TOP_RIGHT_BLOB;
					    return TOP_LEFT_BLOB;
					}
					if (centroid2.y < yc and centroid1.y > yc){
					    // 2 is top
					    // 1 is bottom
					    if (centroid2.x < xc) return TOP_RIGHT_BLOB;
					    return TOP_LEFT_BLOB;
					}
					if(centroid1.y < yc and centroid2.y < yc) return TOP_TWO_BLOBS;
					return BOTTOM_TWO_BLOBS;
				}
		    		else{
				// only one blob
					if (centroid1.y < yc){
					    if (centroid1.x < xc) return TOP_RIGHT_BLOB;
					    return TOP_LEFT_BLOB;
					}
				return BOTTOM_BLOB;
		    		}
			} // close at least one blob
			cout << "NO BLOB IN THE IMAGE "<<endl;
			return NO_BLOB;
		// Close else go on with centroids

}//Close function

class Detector{ 

	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	ros::Subscriber markers_sub;
	ros::Subscriber switch_marker_sub;
	ros::Subscriber land_sub;
	ros::Subscriber takeoff_sub;
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

	// Gestures publisher
	ros::Publisher gesture_message;	
	
    	bool marker_detected;
	bool takenoff;
    	double xc;
    	double yc;
    	double dim;

	// variables to publish the gesture message
	gesturestype vec_aux_max_potition, status = NO_BLOB; 
	std::vector<int>::iterator vec_aux_max;
	std_msgs::String output_label;
	  
public: Detector():it_(nh_){
    

	image_sub_ = it_.subscribe("/ardrone/image_raw", 1, &Detector::imageCb, this);
    	markers_sub = nh_.subscribe("/ar_pose_marker_individual", 1, &Detector::receive_markers, this);
	switch_marker_sub = nh_.subscribe("/ardrone/switch_marker", 1, &Detector::receive_switch_marker, this);
	land_sub = nh_.subscribe("/ardrone/land", 1, &Detector::receive_land, this);
	takeoff_sub = nh_.subscribe("/ardrone/takeoff", 1, &Detector::receive_takeoff, this);			
			
	hmin_sub = nh_.subscribe("/ardrone/Hmin", 1, &Detector::receive_hmin, this);
	hmax_sub = nh_.subscribe("/ardrone/Hmax", 1, &Detector::receive_hmax, this);
	smin_sub = nh_.subscribe("/ardrone/Smin", 1, &Detector::receive_smin, this);
	smax_sub = nh_.subscribe("/ardrone/Smax", 1, &Detector::receive_smax, this);
	vmin_sub = nh_.subscribe("/ardrone/Vmin", 1, &Detector::receive_vmin, this);
	vmax_sub = nh_.subscribe("/ardrone/Vmax", 1, &Detector::receive_vmax, this);


	gesture_message = nh_.advertise<std_msgs::String>("/drone_gestures_demo/vision", 1);
	hmin_pub = nh_.advertise<std_msgs::Int16>("/ardrone/Hminc", 1);
	hmax_pub = nh_.advertise<std_msgs::Int16>("/ardrone/Hmaxc", 1);
	smin_pub = nh_.advertise<std_msgs::Int16>("/ardrone/Sminc", 1);
	smax_pub = nh_.advertise<std_msgs::Int16>("/ardrone/Smaxc", 1);
	vmin_pub = nh_.advertise<std_msgs::Int16>("/ardrone/Vminc", 1);
	vmax_pub = nh_.advertise<std_msgs::Int16>("/ardrone/Vmaxc", 1);

	// Map definition
	gesturesnames[TOP_RIGHT_BLOB]="top_right_blob";
    	gesturesnames[TOP_LEFT_BLOB]="top_left_blob";
    	gesturesnames[TOP_TWO_BLOBS]="top_two_blobs'";
    	gesturesnames[BOTTOM_TWO_BLOBS]="bottom_two_blobs";
    	gesturesnames[BOTTOM_BLOB]="bottom_blob";
        gesturesnames[NO_BLOB]="no_blob";
	gesturesnames[BIG_BLOB] = "big_blob";

	
    	marker_detected = false;
	takenoff = false;

  	}

~Detector(){
	cvDestroyWindow("Segmented_image");
	//sem_destroy(&mutex);
}

	void receive_land(const std_msgs::Empty& msg){
		takenoff = false;
		cur_marker = -1;
		cout << "Received land" << endl;
	}
		
	void receive_takeoff(const std_msgs::Empty& msg){
		takenoff = true;
		cout << "Received take off" << endl;
	}

	void receive_switch_marker(const std_msgs::Int16& msg){
		cur_marker = msg.data;
		cout << "Received marker " << msg.data << endl;
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

	void receive_markers(const ar_track_alvar_msgs_idsia::AlvarMarkers& msg){
		//cout << endl;
		bool marker_present = false;
		for(vector<ar_track_alvar_msgs_idsia::AlvarMarker>::const_iterator it = msg.markers.begin(); it != msg.markers.end(); ++it){
			if ((cur_marker == MARKER_1 and (*it).id == MARKER_1) or (cur_marker == MARKER_2 and (*it).id == MARKER_2) or
                    	(cur_marker == MARKER_3 and (*it).id == MARKER_3) or (cur_marker == MARKER_4 and (*it).id == MARKER_4) or
                    	(cur_marker == MARKER_5 and (*it).id == MARKER_5) or (cur_marker == MARKER_6 and (*it).id == MARKER_6) or
                    	(cur_marker == MARKER_7 and (*it).id == MARKER_7)){
				MarkerImg m = {(*it).x, (*it).y, (*it).dim};
				//circularbufferMarker.push_back(m);
				circularbufferMarker.push_back(m);
				marker_present = true;
				}
			}
		cout << "marker_present " << marker_present <<endl;
		if (!marker_present){
			MarkerImg m = {-1, -1, -1};
			//circularbufferMarker.push_back(m);
			circularbufferMarker.push_back(m);
			}
		//cout << marker_detected << "\n";
		MarkerImg m_res = get_avg_xy();
		if (m_res.xc == - 1){
			marker_detected = false;			
			}
		else{
			marker_detected = true;
			xc = m_res.xc;
			yc = m_res.yc;
               		dim = m_res.dim;
			//cout << "XC: " << xc << endl;
			//cout << "YC: " << yc << endl;
			}
			//cout << marker_detected;
		} 

	void imageCb(const sensor_msgs::ImageConstPtr& msg){
    		try{
			cv_bridge::CvImagePtr cv_ptr;
     			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			Mat image=cv_ptr->image;
			imshow("test window",image);
			waitKey(1);
			call_vision(image);
	    	}
    		catch (cv_bridge::Exception& e){
      			ROS_ERROR("cv_bridge exception: %s", e.what());
      			return;
    		}
	}   

	void call_vision(cv::Mat image){

	if (!published_start_v){
		published_start_v = true;
		publish_hsv();
	}

	gesturestype result;
    	centroid1.y = 0;
	centroid1.x = 0;
    	centroid2.x = 0;
    	centroid2.y = 0;
	
	if (vec_aux[gest_outputs.front()]>0)
		vec_aux[gest_outputs.front()]--;
	result = process_image(image, takenoff, marker_detected, xc, yc, dim);
	gest_outputs.push_back(result);
	gest_outputs.pop_front();

	if (vec_aux[gest_outputs.back()]<15)
		vec_aux[gest_outputs.back()]++;
	// Obtain the maximum value of the vector
	vec_aux_max = std::max_element(vec_aux.begin(), vec_aux.end());
	vec_aux_max_potition=std::distance(vec_aux.begin(), vec_aux_max);
	
	cout << "result " << result <<endl;
	cout << "marker_detected " << marker_detected <<endl;

	if (*vec_aux_max > 7 && status!=vec_aux_max_potition){ 
		
		output_label.data = gesturesnames[vec_aux_max_potition];
		gesture_message.publish(output_label);
		status=vec_aux_max_potition;
	}
		
	image.release();
	if (send_hsv){
		send_hsv = false;
		publish_hsv();
		} 
	}
};


int main(int argc, char** argv)
{ 	
	if (argc > 1){
		ifstream infile(argv[1]);
		if (infile.is_open()){
			infile >> Hmin >> Hmax >> Smin >> Smax >> Vmin >> Vmax;
			cout << Vmax;
			infile.close();
		}
		else cerr << "Calibration file not found. Using default values." << endl;
	}
	else cout << "Using default segmentation values." << endl;

	ros::init(argc, argv, "vision_node");
	ROS_INFO("Launch vision node");
	cvNamedWindow("Segmented_image", CV_WINDOW_AUTOSIZE);
    	Detector ic;

    	ROS_INFO("Done and now listen for incoming images");
    	ros::spin();
    return 0;
}
