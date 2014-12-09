#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/Int16.h"
#include "std_msgs/String.h"
#include <sys/stat.h>
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "limits.h"
#include "float.h"
#include "time.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <deque>
#include <map>
#include <cmath>

#include "ar_track_alvar_msgs_idsia/AlvarMarkers.h"
#include "ar_track_alvar_msgs_idsia/AlvarMarker.h"
#include "cv.h"
#include "highgui.h"
#include "../cvBlobsLib/include/BlobResult.h"
#include "../cvBlobsLib/include/Blob.h"
#include <gesture_messages/led.h>
#include <gesture_messages/vision_gestures.h>
#include <boost/circular_buffer.hpp>


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

enum gesturestype{NO_BLOB, BIG_BLOB, TOP_RIGHT_BLOB, TOP_LEFT_BLOB, TOP_TWO_BLOBS, BOTTOM_TWO_BLOBS, BOTTOM_BLOB};

struct MarkerImg{
	//if set to -1 means not detected.
	double xc;
	double yc;
    	double dim;
};

typedef boost::circular_buffer<MarkerImg> CircularBufferMarker;


class Detector{ 

	//ros::NodeHandle nh_;
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;

	// Subscribers definition
	ros::Subscriber markers_sub;
	ros::Subscriber switch_marker_sub;

	// Gestures publisher
	ros::Publisher gesture_message;	
	
    	bool marker_detected;
    	double xc;
    	double yc;
    	double dim;

	// variables to publish the gesture messages
	gesturestype vec_aux_max_potition, status = NO_BLOB; 
	std::vector<int>::iterator vec_aux_max;
	gesture_messages::vision_gestures output_label;

	int Hmin;
	int Hmax;
	int Smin;
	int Smax;
	int Vmin;
	int Vmax;
	  
	CircularBufferMarker circularbufferMarker;
	int cur_marker;
	CvPoint centroid1, centroid2;

	std::deque<gesturestype> gest_outputs; 
	std::map<gesturestype,string> gesturesnames;
	std::vector<int> vec_aux;


public: Detector():nh_(),it_(nh_),circularbufferMarker(10),gest_outputs (15,NO_BLOB),vec_aux(7,0){
    
	cur_marker=-1;
	//Node subscribers
	image_sub_ = it_.subscribe("image_raw", 1, &Detector::imageCb, this);
    	markers_sub = nh_.subscribe("pose_marker", 1, &Detector::receive_markers, this);
	switch_marker_sub = nh_.subscribe("switch_marker", 1, &Detector::receive_switch_marker, this);

	//Node Publisher
	gesture_message = nh_.advertise<gesture_messages::vision_gestures>("gestures", 1);

	// Map definition
	gesturesnames[NO_BLOB]="no_blob";
	gesturesnames[BIG_BLOB] = "big_blob";
	gesturesnames[TOP_RIGHT_BLOB]="top_right_blob";
    	gesturesnames[TOP_LEFT_BLOB]="top_left_blob";
    	gesturesnames[TOP_TWO_BLOBS]="top_two_blobs";
    	gesturesnames[BOTTOM_TWO_BLOBS]="bottom_two_blobs";
    	gesturesnames[BOTTOM_BLOB]="bottom_blob";
      
	marker_detected = false;
  	}

~Detector(){
	cvDestroyWindow("Segmented_image");
}
	void receive_switch_marker(const std_msgs::Int16& msg){
		cur_marker = msg.data;
		cout << "Received marker " << msg.data << endl;
		}

	void receive_markers(const ar_track_alvar_msgs_idsia::AlvarMarkers& msg){
		//cout << endl;
		bool marker_present = false;
		for(vector<ar_track_alvar_msgs_idsia::AlvarMarker>::const_iterator it = msg.markers.begin(); it != msg.markers.end(); ++it){
            
            	if(cur_marker==it->id && (cur_marker == MARKER_1 || cur_marker == MARKER_2 || cur_marker == MARKER_3 || cur_marker == MARKER_4 
                                        || cur_marker == MARKER_5 || cur_marker == MARKER_6 || cur_marker == MARKER_7)){
   
				MarkerImg m = {(*it).x, (*it).y, (*it).dim};
				circularbufferMarker.push_back(m);
				marker_present = true;
				}
			}
		//cout << "marker_present " << marker_present <<endl;
		if (!marker_present){
			MarkerImg m = {-1, -1, -1};
			//circularbufferMarker.push_back(m);
			circularbufferMarker.push_back(m);
			}
		
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
		//cout << marker_detected << "\n";
		} 

	void imageCb(const sensor_msgs::ImageConstPtr& msg){
    		try{
			cv_bridge::CvImagePtr cv_ptr;
     			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			Mat image=cv_ptr->image;
			//imshow("test window",image);
			//waitKey(1);

			// Update filter paramiters after obtain the image 
			nh_.param("Hmin",Hmin,15);
			nh_.param("Hmax",Hmax,170);
			nh_.param("Smin",Smin,50);
			nh_.param("Smax",Smax,255);
			nh_.param("Vmin",Vmin,50);
			nh_.param("Vmax",Vmax,255);

			call_vision(image);
	    	}
    		catch (cv_bridge::Exception& e){
      			ROS_ERROR("cv_bridge exception: %s", e.what());
      			return;
    		}
	}   

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

gesturestype process_image(cv::Mat inputImage, bool marker_detected, double xc, double yc, double dim) {
		// Convert from Mat to IplImage+
		IplImage* image = cvCloneImage(&(IplImage)inputImage); 

		// THRESH HOLDING //
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//// Segment the current image ////
		IplImage *imageHSV = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 3);
		IplImage *imageColor = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);   // Grayscale output image

		// Convert RGB to HSV format //
		cvCvtColor(image, imageHSV, CV_BGR2HSV);
		// Segement Color //

		//Jerome Change -> hue value are periodic in [0,180] -> filter should be 0 - min &&   max - 180 if max-min > 90
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
		waitKey(1);

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

		//cout << "blob 1 area "<< ColorBlob1.Area() <<endl;
		//cout << "blob 2 area "<< ColorBlob2.Area() <<endl;
		//cout << "min_area "<<min_area<<endl;
		//cout << "max_area "<<max_area<<endl;

		//Classify the gesture
	
			if (ColorBlob1.Area() > AREA_TAKEOFF){ //very close! -> takeoff
					return BIG_BLOB;
			}
			
			if (ColorBlob1.Area() > min_area && ColorBlob1.Area() < max_area){
			    	//at least one blob
				//cout << "IM SEING ATLEAST ON BLOB "<<endl;
				if(!marker_detected){			
					return NO_BLOB;
					cout << "MARKER NOT DETECTED "<<endl;
				}
			    	if (ColorBlob2.Area() > min_area &&   ColorBlob2.Area() < max_area){
				//cout << "IM SEING TWO BLOBS "<<endl;
				//two blobs seen
					if (centroid1.y < yc &&   centroid2.y > yc){
					    // 1 is top
					    // 2 is bottom
					    if (centroid1.x < xc) return TOP_LEFT_BLOB;
					    return TOP_RIGHT_BLOB;
					}
					if (centroid2.y < yc &&   centroid1.y > yc){
					    // 2 is top
					    // 1 is bottom
					    if (centroid2.x < xc) return TOP_LEFT_BLOB;
					    return TOP_RIGHT_BLOB;
					}
					if(centroid1.y < yc &&   centroid2.y < yc) return TOP_TWO_BLOBS;
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
			//cout << "NO BLOB IN THE IMAGE "<<endl;
			return NO_BLOB;
		// Close else go on with centroids

}//Close function

	void call_vision(cv::Mat image){

	gesturestype result;
    	centroid1.y = 0;
	centroid1.x = 0;
    	centroid2.x = 0;
    	centroid2.y = 0;
	
	if (vec_aux[gest_outputs.front()]>0)
		vec_aux[gest_outputs.front()]--;
	result = process_image(image, marker_detected, xc, yc, dim);
	gest_outputs.push_back(result);
	gest_outputs.pop_front();

	if (vec_aux[gest_outputs.back()]<15)
		vec_aux[gest_outputs.back()]++;
	// Obtain the maximum value of the vector
	vec_aux_max = std::max_element(vec_aux.begin(), vec_aux.end());
	vec_aux_max_potition=std::distance(vec_aux.begin(), vec_aux_max);
	
	//cout << "result " << result <<endl;
	//cout << "marker_detected " << marker_detected <<endl;

	if (*vec_aux_max > 7 && status!=vec_aux_max_potition){ 
		
		output_label.gesture = gesturesnames[vec_aux_max_potition];
		gesture_message.publish(output_label);
		status=vec_aux_max_potition;
	}
		
	image.release();
	}
};


int main(int argc, char** argv){ 	

	ros::init(argc, argv, "vision_node");
	ROS_INFO("Launch vision node");
	cvNamedWindow("Segmented_image", CV_WINDOW_AUTOSIZE);
    	Detector ic;

    	ROS_INFO("Done, now listen for incoming images");
    	ros::spin();
    return 0;
}
