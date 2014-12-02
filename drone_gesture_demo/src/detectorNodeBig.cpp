#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Empty.h"
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
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "pthread.h"
#include <sys/stat.h>
#include <boost/circular_buffer.hpp>
#include <semaphore.h>
#include <pthread.h>
#include <vector>
#include <fstream>
#include <gesture_messages/led.h>
using namespace std;
using namespace cv;

namespace enc = sensor_msgs::image_encodings;

#define AREA_TAKEOFF 1500
#define PERC_OK 0.5

//StateMachine stages
#define STAGE_1 1
#define STAGE_1_END 2
#define STAGE_2 3
#define STAGE_2_END 4
#define STAGE_3 5
#define STAGE_3_END 6
#define STAGE_FAIL 7

#define TIME_CL 1.5

#define TIME_FEEDBACK 2.5

//last frames interpretation
#define BUF_SIZE 45
#define NO_GEST 0
#define YOU 1
#define FOLLOW 2
#define LAND 3
#define LEFT 4
#define RIGHT 5
#define TAKEOFF 6
#define KO 10

int stage;
int takeoff_count = 0;

typedef boost::circular_buffer<int> CircularBufferInt; 
CircularBufferInt cb(BUF_SIZE);

struct MarkerImg{
	//if set to -1 means not detected.
	double xc;
	double yc;
    double dim;
};

typedef boost::circular_buffer<MarkerImg> CircularBufferMarker;
CircularBufferMarker cbm(10);

sem_t mutex;

#define MARKER_1 7
#define MARKER_2 16
#define MARKER_3 0
#define MARKER_4 1
#define MARKER_5 2
#define MARKER_6 3
#define MARKER_7 4

int cur_marker = -1;

// Global vars
CvPoint centroid1, centroid2;

//Define color thresholds
//Yellow
int Hmin = 18;
int Hmax = 28;
int Smin = 150;
int Smax = 255;
int Vmin = 10;
int Vmax = 255;

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

bool send_hsv = false;

//These functions check whether there is the mentioned gesture in the buffer or not
bool is_you(const CircularBufferInt* buffer){
    
    int count = 0;
    for(CircularBufferInt::const_iterator it = (*buffer).begin(); it != (*buffer).end(); ++it){
        if (*it == YOU){
			count += 1;		
		}
    }
	
	if (count >= PERC_OK*BUF_SIZE)
        return true;
    
    else return false;
}

bool is_follow(const CircularBufferInt* buffer){

    int count = 0;
    for(CircularBufferInt::const_iterator it = (*buffer).begin(); it != (*buffer).end(); ++it){
        if (*it == FOLLOW){
			count += 1;		
		}
    }
	
	if (count >= PERC_OK*BUF_SIZE)
        return true;
    
    else return false;	
	
}

bool is_land(const CircularBufferInt* buffer){

    int count = 0;
    for(CircularBufferInt::const_iterator it = (*buffer).begin(); it != (*buffer).end(); ++it){
        if (*it == LAND){
			count += 1;		
		}
    }
	
	if (count >= PERC_OK*BUF_SIZE)
        return true;
    
    else return false;
}

bool is_left(const CircularBufferInt* buffer){

    int count = 0;
    for(CircularBufferInt::const_iterator it = (*buffer).begin(); it != (*buffer).end(); ++it){
        if (*it == LEFT){
			count += 1;		
		}
    }
	
	if (count >= PERC_OK*BUF_SIZE)
        return true;
    
    else return false;
}

bool is_right(const CircularBufferInt* buffer){

    int count = 0;
    for(CircularBufferInt::const_iterator it = (*buffer).begin(); it != (*buffer).end(); ++it){
        if (*it == RIGHT){
			count += 1;		
		}
    }
	
	if (count >= PERC_OK*BUF_SIZE)
        return true;
    
    else return false;
}

MarkerImg get_avg_xy(){
	double x = 0;
	double y = 0;
    double dim = 0;
	double count = 0;
	 for(CircularBufferMarker::const_iterator it = cbm.begin(); it != cbm.end(); ++it){
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

/*void switch_marker(){
	if (cur_marker == MARKER_1) cur_marker = MARKER_2;
	else cur_marker = MARKER_1;
}*/


int process_image(cv::Mat inputImage, bool takenoff, bool marker_detected, double xc, double yc, double dim) {
		// Convert from Mat to IplImage+
		IplImage* image = cvCloneImage(&(IplImage)inputImage); 

		// THRESH HOLDING //
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//// Segment the current image ////
		IplImage *imageHSV					= cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 3);
		IplImage *imageColor				= cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);   // Grayscale output image

		// Convert RGB to HSV format //
		cvCvtColor(image, imageHSV, CV_BGR2HSV);
		

		// Segement Color //

		//Jerome Change -> hue value are periodic in [0,180] -> filter should be 0 - min AND max - 180 if max-min > 90
		//Alternatively we could use CV_BGR2HSV_FULL
		if(Hmax-Hmin>90)
		  {
		    IplImage *imageColorHigh=cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);
		    cvInRangeS(imageHSV, cvScalar(0, Smin, Vmin), cvScalar(Hmin, Smax, Vmax), imageColor);
		    cvInRangeS(imageHSV, cvScalar(Hmax, Smin, Vmin), cvScalar(180, Smax, Vmax), imageColorHigh); 
		    cvOr(imageColorHigh,imageColor,imageColor);
		    cvReleaseImage(&imageColorHigh);
		  }
		else
		  {
		cvInRangeS(imageHSV, cvScalar(Hmin, Smin, Vmin), cvScalar(Hmax, Smax, Vmax), imageColor); // Color threshold
		  }




		// Segement Color //
		//		cvInRangeS(imageHSV, cvScalar(Hmin, Smin, Vmin), cvScalar(Hmax, Smax, Vmax), imageColor); // Color threshold
               
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

		if (!takenoff){
			cvReleaseImage(&image);
			cvReleaseImage(&imageHSV);
			cvReleaseImage(&imageColor);
			//free memory
			blobsColor.ClearBlobs();			
			if (ColorBlob1.Area() > AREA_TAKEOFF){ //very close! -> takeoff
				return TAKEOFF;
			}
			else return NO_GEST;
		} //takeoff = True
		else if(!marker_detected){
			cvReleaseImage(&image);
			cvReleaseImage(&imageHSV);
			cvReleaseImage(&imageColor);
			//free memory
			blobsColor.ClearBlobs();			
			return NO_GEST;
		}
		else{
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
		//Classify the gesture
        if (ColorBlob1.Area() > min_area and ColorBlob1.Area() < max_area){
            
            //at least one blob
            
            if (ColorBlob2.Area() > min_area and ColorBlob2.Area() < max_area){ 
                
                //two blobs seen
                
                if (centroid1.y < yc and centroid2.y > yc)
                {
                    // 1 is top
                    // 2 is bottom
                    if (centroid1.x < xc) return TOP_RIGHT_BLOB;
                    return TOP_LEFT_BLOB;
                }
                if (centroid2.y < yc and centroid1.y > yc)
                {
                    // 2 is top
                    // 1 is bottom
                    if (centroid2.x < xc) return TOP_RIGHT_BLOB;
                    return TOP_LEFT_BLOB;
                }
                
                if(centroid1.y < yc and centroid2.y < yc) return TOP_TWO_BLOBS;
                
                return BOTTOM_TWO_BLOBS;
			}
            else
            {
                // only one blob
                
                if (centroid1.y < yc)
                {
                    if (centroid1.x < xc) return TOP_RIGHT_BLOB;
                    return TOP_LEFT_BLOB;
                }
                return BOTTOM_BLOB;
            }
        }
        return NO_BLOBS
}


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

  	ros::Publisher gesture_pub;
	
	//coherence with python gui
	ros::Publisher hmin_pub;
    ros::Publisher hmax_pub;
    ros::Publisher smin_pub;
    ros::Publisher smax_pub;
    ros::Publisher vmin_pub;
    ros::Publisher vmax_pub;	
	
    bool marker_detected;
	bool takenoff;
    double xc;
    double yc;
    double dim;
  
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

    		gesture_pub = nh_.advertise<std_msgs::Int8>("/ardrone/gest", 1);
			hmin_pub = nh_.advertise<std_msgs::Int16>("/ardrone/Hminc", 1);
			hmax_pub = nh_.advertise<std_msgs::Int16>("/ardrone/Hmaxc", 1);
			smin_pub = nh_.advertise<std_msgs::Int16>("/ardrone/Sminc", 1);
			smax_pub = nh_.advertise<std_msgs::Int16>("/ardrone/Smaxc", 1);
			vmin_pub = nh_.advertise<std_msgs::Int16>("/ardrone/Vminc", 1);
			vmax_pub = nh_.advertise<std_msgs::Int16>("/ardrone/Vmaxc", 1);

    		marker_detected = false;
			takenoff = false;
  		}

		~Detector(){
			cvDestroyWindow("Segmented_image");
			sem_destroy(&mutex);
		}

		void receive_land(const std_msgs::Empty& msg){
			takenoff = false;
			cur_marker = -1;
			stage = STAGE_1;
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
					cbm.push_back(m);
					marker_present = true;
				}
			}
			//cout << marker_present << endl;
			if (!marker_present){
				MarkerImg m = {-1, -1, -1};
				cbm.push_back(m);
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

	if (!published_start_v){
		published_start_v = true;
		publish_hsv();
	}
	int result;
    centroid1.y = 0;
	centroid1.x = 0;
    centroid2.x = 0;
    centroid2.y = 0;
	cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
	Mat image=cv_ptr->image;
    //ROS_INFO("New image");
	
	if (marker_detected){
		takeoff_count = 0;
	}
    //initialization
    try{
    	//cout << "Image detected.\n";
    	result = process_image(image, takenoff, marker_detected, xc, yc, dim);
		if (result == TAKEOFF and !takenoff){ //8 subsequent frames with takeoff mean TAKEOFF !takeoff reduntant
		    takeoff_count += 1;
			cout << "Count takeoff " << takeoff_count << endl;
		    if (takeoff_count == 8){
				std_msgs::Int8 gesture;
        		gesture.data = TAKEOFF;
				cout << "Sending Takeoff" <<endl;
			    gesture_pub.publish(gesture);
			    takeoff_count = 0;
			    }
		}
        //cout << result << endl;
		cb.push_back(result);
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

class StateMachine
{
    ros::NodeHandle nh_;
    ros::Publisher gesture_pub;
    ros::Publisher led_pub;
    ros::Timer timer;
    double start;
	pthread_t thread_sound;

	ros::ServiceClient client_led;
	ros::ServiceClient client_flight;
 
	public: StateMachine(){
				timer = nh_.createTimer(ros::Duration(0.1), &StateMachine::doProcess, this);
				gesture_pub = nh_.advertise<std_msgs::Int8>("/ardrone/gest", 1);
                led_pub = nh_.advertise<gesture_messages::led>("/drone1/led", 1);
				start = -1;
				stage = STAGE_1;
				client_led = nh_.serviceClient<ardrone_autonomy::LedAnim>("/ardrone/setledanimation");
				client_flight = nh_.serviceClient<ardrone_autonomy::FlightAnim>("/ardrone/setflightanimation");
            }
            
            std_msgs::Int8 prepare_message(int id){
				std_msgs::Int8 gesture;
        	    gesture.data = id;
        	    return gesture;
			}

            gesture_messages::led prepare_word(gesture_messages::led& msg, int id){
                msg.command = "off";
                if (id == 1){
                    msg.duration = 0.5;
                    msg.iterations = 1;
                }
                else{
					msg.duration = 0.5;
                    msg.iterations = 3;
				}
                return msg;
			}

			void call_led_anim(int id){
				ardrone_autonomy::LedAnim srv;
				srv.request.type = id;
				srv.request.freq = 4;
				srv.request.duration = 3;
				if (!client_led.call(srv))
					ROS_ERROR("Failed to call service add_two_ints"); 
			}

			void call_flight_anim(int id){
				ardrone_autonomy::FlightAnim srv;
				srv.request.type = id;
				srv.request.duration = 0;
				if (!client_flight.call(srv))
					ROS_ERROR("Failed to call service add_two_ints");
			}

			void doProcess(const ros::TimerEvent& event){
                //make a copy of the current buffer
				if (!cb.full()) return;
				switch(stage){
					case STAGE_1: {
						sem_wait(&mutex); 
                		CircularBufferInt* cb_copy = new CircularBufferInt(cb);
						sem_post(&mutex); 
					    if (is_you(cb_copy)){
						    cout << "Ended stage 1. Going to stage 1 End. \n";
							stage = STAGE_1_END;
						}
						else {
							//the buffer is not holding a YOU.
							cout << "No gesture detected. \n";						
						}
						delete cb_copy;
					    break;}//end stage 1
					
					case STAGE_1_END:{
						cout << "In stage 1 End. \n";
						if (start == -1){
								start = ros::Time::now().toSec();
								cout << "Sending ok feedback. \n";
								gesture_pub.publish(prepare_message(YOU));
                                /*gesture_messages::led msg;
                                led_pub.publish(prepare_word(msg, 1));*/
								//system("/usr/bin/canberra-gtk-play --id='bell'");
								call_led_anim(5);
							}
						else{
							if (ros::Time::now().toSec() - start > TIME_FEEDBACK){
								cout << "Ended stage 1 End. Going to stage 2. \n";
								start = -1;
								stage = STAGE_2;							
								}
							else cout << "In stage 1 End. Waiting... \n";
							}
						break;
					}

					case STAGE_2:{
						cout << "In stage 2. \n";
						if (start == -1){
							start = ros::Time::now().toSec();
						}
						else{
							if (ros::Time::now().toSec() - start > TIME_CL){
								start = -1;
								//consider classified frames
								sem_wait(&mutex); 
                				CircularBufferInt* cb_copy = new CircularBufferInt(cb);
								sem_post(&mutex);
								if (is_land(cb_copy)){
									cout << "Ended stage 2. Landing. \n";
									gesture_pub.publish(prepare_message(LAND));
                                    /*gesture_messages::led msg;
                                    led_pub.publish(prepare_word(msg, 2));*/
							        stage = STAGE_3_END;
									}
								else if (is_follow(cb_copy)){
									cout << "Ended stage 2. Follow. \n";
									gesture_pub.publish(prepare_message(FOLLOW));
                                    /*gesture_messages::led msg;
									led_pub.publish(prepare_word(msg, 1));*/
									stage = STAGE_2_END;
									}
								else{
									//something went bad. Going to Fail stage =(
									cout << "Ended stage 2. Classification failed. \n";
									stage = STAGE_FAIL;
									}
 								delete cb_copy;
								}
							else cout << "In stage 2. Classifying... \n";
							}
						break;
						}
					case STAGE_2_END:{
						cout << "In stage 2 End.\n";
						if (start == -1){
								start = ros::Time::now().toSec();
								cout << "Sending ok feedback. \n";
								//gesture_pub.publish(prepare_message(FEEDBACK_OK));
								//system("/usr/bin/canberra-gtk-play --id='bell'");
								call_led_anim(5);
							}
						else{
							if (ros::Time::now().toSec() - start > TIME_FEEDBACK){
								cout << "Ended stage 2 End. Going to stage 3. \n";
								start = -1;
								stage = STAGE_3;							
								}
							else cout << "In stage 2 End. Waiting... \n";
							}
						break;
						}
					
					case STAGE_3:{
						cout << "In stage 3. \n";
						if (start == -1){
							start = ros::Time::now().toSec();
						}
						else{
							if (ros::Time::now().toSec() - start > TIME_CL){
								start = -1;
								//consider classified frames
								sem_wait(&mutex); 
                				CircularBufferInt* cb_copy = new CircularBufferInt(cb);
								sem_post(&mutex);
								if (is_left(cb_copy)){
									cout << "Ended stage 3. Left. \n";
									gesture_pub.publish(prepare_message(LEFT));
                                    /*gesture_messages::led msg;
									led_pub.publish(prepare_word(msg, 2));*/
									cur_marker = -1;
							        stage = STAGE_3_END;
									}
								else if (is_right(cb_copy)){
									cout << "Ended stage 3. Right. \n";
									gesture_pub.publish(prepare_message(RIGHT));
                                    /*gesture_messages::led msg;
									led_pub.publish(prepare_word(msg, 2));*/
									cur_marker = -1;
									stage = STAGE_3_END;
									}
								else{
									//something went bad. Going to Fail stage =(
									cout << "Ended stage 3. Classification failed. \n";
									stage = STAGE_FAIL;
									}
								delete cb_copy;
 
								}
							else cout << "In stage 3. Classifying... \n";
							}
						break;
						}

					case STAGE_3_END:{
						cout << "In stage 3 End.\n";
						if (start == -1){
								start = ros::Time::now().toSec();
								cout << "Sending ok feedback. \n";
								//gesture_pub.publish(prepare_message(FEEDBACK_OK));
								//system("/usr/bin/canberra-gtk-play --id='bell'");
								call_led_anim(4);
							}
						else{
							if (ros::Time::now().toSec() - start > TIME_FEEDBACK){
								cout << "Ended stage 1 End. Going to stage 1. \n";
								start = -1;
								stage = STAGE_1;							
								}
							else cout << "In stage 3 End. Waiting... \n";
							}
						break;
						}					

					case STAGE_FAIL:{
						cout << "In stage fail. \n";
						if (start == -1){
								start = ros::Time::now().toSec();
								cout << "Sending KO feedback. \n";
								gesture_pub.publish(prepare_message(KO));
                                /*gesture_messages::led msg;
                                led_pub.publish(prepare_word(msg, 1));*/
								 //system("/usr/bin/canberra-gtk-play --id='button-pressed'");
								call_flight_anim(4);
							}
						else{
							if (ros::Time::now().toSec() - start > TIME_FEEDBACK){
								cout << "Ended stage Fail. Going to stage 1. \n";
								start = -1;
								stage = STAGE_1;							
								}
							else cout << "In stage Fail. Waiting... \n";
							}
						break;
						}
				}//end switch
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

	ros::init(argc, argv, "detector_node");
	ROS_INFO("Launch detector");
    sem_init(&mutex, 0, 1);
	cvNamedWindow("Segmented_image", CV_WINDOW_AUTOSIZE);
	cvStartWindowThread();
    Detector ic;
    StateMachine sm;
    ROS_INFO("Done and now listen for incoming images");
    ros::spin();
    return 0;
}
