#include "stdafx.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "limits.h"
#include "float.h"
#include "time.h"
#include "cv.h"
#include "highgui.h"
#include "svm.h"
#include "cvBlobsLib/src/BlobResult.h"
#include "cvBlobsLib/src/Blob.h"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "threads/built/include/pthread.h"
#include <windows.h>

using namespace std;
using namespace cv;


// Set camera properties
///////////////////////////////
//1920×1080 -- full hd 1080p
//1280x720 -- hd 720p
static int framewidth = 1280;
static int frameheight = 720;
static int fps = 30;

// Global vars
CvPoint centroidyellow, centroidgreen, centroidjacket, jacketubound, jacketlbound, jacketright, jacketleft;


//Define color thresholds
//Yellow
int Hmin_Y = 18;
int Hmax_Y = 28;
int Smin_Y = 150;
int Smax_Y = 255;
int Vmin_Y = 10;
int Vmax_Y = 255;
//Green
int Hmin_G = 45;
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
int Vmax_O = 255;



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



int simple3(cv::Mat inputImage) {
		// Gesture action vocabulary (5 classes)
		//////////////////////////////////////////////////////////////////////////////////////
		// 1 - If both gloves are higher than jacket
		// 2 - If both gloves are lower than jacket
		// 3 - If green glove is higher than jacket and yellow glove is lower than jacket
		// 4 - If yellow glove is higher than jacket and green glove is lower than jacket
		// 5 - No gesture detected
		if ((centroidgreen.y < jacketlbound.y) && (centroidyellow.y < jacketlbound.y)) {
			return 1;
		} else if ((centroidgreen.y > jacketlbound.y) && (centroidyellow.y > jacketlbound.y)) {
			return 2;
		} else if ((centroidgreen.y < jacketlbound.y) && (centroidyellow.y > jacketlbound.y)) {
			return 3;
		} else if ((centroidgreen.y > jacketlbound.y) && (centroidyellow.y < jacketlbound.y)) {
			return 4;
		} else {
			return 0;
		}
}


int simple2(cv::Mat inputImage) {
		// if green glove is higher than yellow glove --> there is a gesture
		if (centroidgreen.y < centroidyellow.y) {
			return 1;
		// else no gesture detected
		} else {
			return 0;
		}
}



int simple1(cv::Mat inputImage) {
		// Convert from Mat to IplImage
		IplImage* image = cvCloneImage(&(IplImage)inputImage);

		// THRESH HOLDING //
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//// Segment the current image ////
		IplImage *imageHSV					= cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 3);
		IplImage *imageYellow				= cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);   // Grayscale output image
		IplImage *imageGreen				= cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);   // Grayscale output image
		IplImage *imageOrange				= cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);   // Grayscale output image
		IplImage *imageYellowGreen			= cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);   // Grayscale output image
		IplImage *imageYellowGreenOrange	= cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);   // Grayscale output image

		// Convert RGB to HSV format //
		cvCvtColor(image, imageHSV, CV_BGR2HSV);
		// Segement YELLOW //
		cvInRangeS(imageHSV, cvScalar(Hmin_Y, Smin_Y, Vmin_Y), cvScalar(Hmax_Y, Smax_Y, Vmax_Y), imageYellow); // yellow threshold
		// Segement GREEN //
		cvInRangeS(imageHSV, cvScalar(Hmin_G, Smin_G, Vmin_G), cvScalar(Hmax_G, Smax_G, Vmax_G), imageGreen); // green threshold
		// Segement ORANGE //
		cvInRangeS(imageHSV, cvScalar(Hmin_O, Smin_O, Vmin_O), cvScalar(Hmax_O, Smax_O, Vmax_O), imageOrange); // orange threshold
		// Combine all segmented colors into one image
		cvOr(imageYellow,imageGreen,imageYellowGreen);
		cvOr(imageYellowGreen,imageOrange,imageYellowGreenOrange);

		// GREEN blob //
		CBlobResult blobsGreen;
		blobsGreen = CBlobResult(imageGreen, NULL, 0);
		CBlob biggestGreenBlob;
		blobsGreen.GetNthBlob(CBlobGetArea(), 0, biggestGreenBlob);
		// YELLOW blob //
		CBlobResult blobsYellow;
		blobsYellow = CBlobResult(imageYellow, NULL, 0);
		CBlob biggestYellowBlob;
		blobsYellow.GetNthBlob(CBlobGetArea(), 0, biggestYellowBlob);
		// 1 ORANGE blob //
		CBlobResult blobsOrange;
		blobsOrange = CBlobResult(imageOrange, NULL, 0);
		CBlob biggestOrangeBlob;
		blobsOrange.GetNthBlob(CBlobGetArea(), 0, biggestOrangeBlob);
		// All 3 blobs //
		CBlobResult blobsYellowGreenOrange;
		blobsYellowGreenOrange = CBlobResult(imageYellowGreenOrange, NULL, 0);
		CBlob biggestYellowGreenOrangeBlob;
		blobsYellowGreenOrange.GetNthBlob(CBlobGetArea(), 0, biggestYellowGreenOrangeBlob);

		// Centroids and upper/lower bounds of gloves and jacket
		////////////////////////////////////////////////////////////////////////////////////////
		centroidyellow.x = (biggestYellowBlob.MinX() + biggestYellowBlob.MaxX()) / 2;
		centroidgreen.x = (biggestGreenBlob.MinX() + biggestGreenBlob.MaxX()) / 2;
		centroidjacket.x = (biggestOrangeBlob.MinX() + biggestOrangeBlob.MaxX()) / 2;
		centroidyellow.y = (biggestYellowBlob.MinY() + biggestYellowBlob.MaxY()) / 2;
		centroidgreen.y = (biggestGreenBlob.MinY() + biggestGreenBlob.MaxY()) / 2;
		centroidjacket.y = (biggestOrangeBlob.MinY() + biggestOrangeBlob.MaxY()) / 2;
		//
		jacketubound.x = centroidjacket.x;
		jacketubound.y = biggestOrangeBlob.MinY();
		jacketlbound.x = centroidjacket.x;
		jacketlbound.y = biggestOrangeBlob.MaxY();
		//
		jacketleft.x = biggestOrangeBlob.MinX();
		jacketleft.y = centroidjacket.y;
		jacketright.x = biggestOrangeBlob.MaxX();
		jacketright.y = centroidjacket.y;

		//Show frames on computer sceen
		///////////////////////////////////
		cvShowImage("Simple1", imageYellowGreenOrange);

		//release images
		cvReleaseImage(&image);
		cvReleaseImage(&imageHSV);
		cvReleaseImage(&imageYellow);
		cvReleaseImage(&imageGreen);
		cvReleaseImage(&imageOrange);
		cvReleaseImage(&imageYellowGreen);
		cvReleaseImage(&imageYellowGreenOrange);
		//free memory
		blobsYellow.ClearBlobs();
		blobsGreen.ClearBlobs();
		blobsOrange.ClearBlobs();
		blobsYellowGreenOrange.ClearBlobs();
		//inputImage.release();
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		//returns area of biggestblob (with respect to area) found from all three colors
		return biggestYellowGreenOrangeBlob.Area();
}



int main(int argc, const char** argv)
{
	// Capture image
	CvCapture *capture = cvCaptureFromCAM(1);

	// Set camera properties
	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, framewidth);
	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, frameheight);
	cvSetCaptureProperty(capture, CV_CAP_PROP_FPS, fps);
	printf("Frame Width: %d\n", framewidth);
	printf("Frame Height: %d\n", frameheight);

	//initialization
	int result1, result2, result3;
	centroidgreen.y = 0;
	centroidyellow.y = 0;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Go into main thread
	while (1)
	{
		// Acquire frame from camera as IplImage
		IplImage *frame = cvQueryFrame(capture);
		// Convert from IplImage to Mat
		cv::Mat image=cvarrToMat(frame, true); 

		// Do processing here
		result1 = simple1(image);
		result2 = simple2(image);
		result3 = simple3(image);

		// Print results
		printf("result1: %d, result2: %d, result3: %d\n", result1, result2, result3);

		//release memory
		image.release();

		// Do not release the frame!
		//If ESC key pressed, Key=0x10001B under OpenCV 0.9.7 (Linux version),
		//remove higher bits using AND operator
		if ((cvWaitKey(1) & 255) == 27) break;
	}

	// Release the capture device housekeeping
	cvReleaseCapture(&capture);
	cvDestroyWindow("User Interface");

	return 0;
}



/**
please send the gestureSimple.cpp as we discussed in the chat.  If you
manage to do that before tomorrow morning, it will be fine as we'll
already use drones tomorrow and will be able to test your software
directly.

Since the complete system is now (almost) ready, please deliver it
structured it in a similar way to gestureSimple.cpp!  In particular,
you could make a file gestureComplete.cpp which includes the following
functions.

1) void Init()
that will be called once at the beginning.  you can initialize your
threads here (in case you are still mutli-threaded), the debug
windows, initialize variables, and everything else.  This function
should include NO CAMERA-SPECIFIC CODE.

2) int newFrame(cv::Mat)
that will be called every time a new frame is ready.  The
function should always return 0, except when a full sentence is
detected.  Later, we will use a different return type which includes
all information about the detected sentence, and we will also include
information for feedback.

3) void Cleanup()
function, which will be called before the system stops.

You can test the system by writing your own main() which uses the
webcam.  your main will be structured as follows:

main() {
  initialize_camera()
  Init();
  while(true) {
     cv::Mat frame=get_frame_from_camera()
     int ret=newFrame(frame);
     if(ret==1) printf("Finished a sentence!");
  }
  Cleanup();
}


//////////////////////////////////////////////////////////////////////////////

in GestureSimple.cpp please make:

> 1) one function
>
> int simple1(cv::Mat inputImage)
>
> which takes an image as input and returns a very simple result with
> minimal processing.  This function should include NO CODE about the
> camera.
>
>
> 2) one function
>
> int simple2(cv::Mat inputImage)
>
> which returns 0 if no gesture is detected in the image
> and returns 1 if the specific gesture is detected (you decide the gesture).
> As before, this function should include NO CODE about the camera.
>
>
> 3) one function
>
> int simple2(cv::Mat inputImage)
>
> which returns 0 if no gesture is in the image
> else it returns an integer indicating the gesture ID (you choose 3 or
> 4 different gestures to detect)
>
>
> 4) one main function
>
> this will include all the camera-related code and the infinite loop.
>
>
>
> ====
>
> LATER
>
> Only at a second moment, we will integrate all the work you did (with
> or without the motion detector).  In that case, you should expose the
> following interface.
>
>
> 1) a  void Init()
> function, called at the beginning of the acquistion where you can
> setup whatever you need.
>
> 2) a  int newFrame(cv::Mat)
> function, which is called every time a new frame is available.  The
> function should always return 0, except when a full sentence is
> detected.  Later, we will use a different return type which includes
> all information about the detected sentence, and we will also include
> information for feedback.
>
> 3) a  void Cleanup()
> function, which will be called before the system stops.

**/



