//#include "stdafx.h"
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
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "pthread.h"
//#include <windows.h>

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

// Face detector parameters
CvPoint facecentroid;
char *facedirection = "";
float scalefactor = 1.1f;
char *frontclassifier = "haarcascade_frontalface_default.xml";
char *sideclassifier = "haarcascade_profileface.xml";
CvSize minFeatureSize = cvSize(24,24); // Min size: (20,20)
CvHaarClassifierCascade *frontfaceclassifier, *sidefaceclassifier;
int flags = CV_HAAR_DO_CANNY_PRUNING | CV_HAAR_DO_ROUGH_SEARCH; //| CV_HAAR_FIND_BIGGEST_OBJECT; // | CV_HAAR_DO_ROUGH_SEARCH; CV_HAAR_DO_CANNY_PRUNING
int minneighbours = 0; // Set this filter to 1-3 if you just wish to get averaged bounding box. For minneighbours greater than 0, facescore will not be valid.
int kernelsize = 5; // Gaussian kernel size (square; width = height). This is for preprocessing edges (blurring) so sharp edges can be removed (removal of false positives of detected faces)
double facescore, scorecenter, scoreleft, scoreright;


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



// Perform face detection on the input image, using the given Haar Cascade.
// Returns a rectangle for the detected region in the given image.
CvRect detectFaceInImage(IplImage *inputImg, CvHaarClassifierCascade* cascade, int key)
{
	// How detailed should the search be.
	float search_scale_factor = scalefactor;
	IplImage *detectImg;
	IplImage *greyImg = 0;
	CvMemStorage *storage;
	CvRect rc;
	CvSeq *facerect;
	CvSize size;

	// Declare storage and memory
	storage = cvCreateMemStorage(0);
	cvClearMemStorage(storage);

	// If the image is color, use a greyscale copy of the image.
	detectImg = (IplImage*)inputImg;
	if (inputImg->nChannels > 1) {
		size = cvSize(inputImg->width, inputImg->height);
		greyImg = cvCreateImage(size, IPL_DEPTH_8U, 1);
		cvCvtColor(inputImg, greyImg, CV_BGR2GRAY);
		detectImg = greyImg;	// Use the greyscale image.
	}
	// Gaussian blurring
	cvSmooth(detectImg, detectImg, CV_GAUSSIAN, kernelsize, kernelsize);

	//// Detect all possible face matches in the greyscale image ////
	facerect = cvHaarDetectObjects(detectImg, cascade, storage, search_scale_factor, minneighbours, flags, minFeatureSize);
	//groupRectangles(facerect, 1, 0.2);	

	// Get Facescore confidence
	facescore = facerect->total; // global variable

	// Initialize empty values
	CvRect tempface;
	CvPoint pt1, pt2;
	pt1.x = 0;
	pt1.y = 0;
	pt2.x = 0;
	pt2.y = 0;

	// If a face is detected compute mean locations of all the faces found
	if (facescore > 0) {
		for(int i=0; i < facerect->total; i++) {
			// extract the rectanlges only //
			tempface = *(CvRect*)cvGetSeqElem(facerect, i);

			// Sum coordinates of all found bounding boxes
			pt1.x = pt1.x + tempface.x;
			pt1.y = pt1.y + tempface.y;
			pt2.x = pt2.x + tempface.x + tempface.width;
			pt2.y = pt2.y + tempface.y + tempface.height;
		}
		// Average coordinates to compute mean
		pt1.x = pt1.x / facescore;
		pt1.y = pt1.y / facescore;
		pt2.x = pt2.x / facescore;
		pt2.y = pt2.y / facescore;
		rc = cvRect(pt1.x, pt1.y, pt2.x, pt2.y);

	// Else if no face detected
	} else {
		rc = cvRect(-1,-1,-1,-1);	// Couldn't find the face.
	}

/**
	// draw all the rectangles //
	for(int i = 0; i < facerect->total; i++) {
		// extract the rectanlges only //
		CvRect face_rect = *(CvRect*)cvGetSeqElem(facerect, i);
		CvPoint pt1face, pt2face;
		pt1face.x = face_rect.x;
		pt1face.y = face_rect.y;
		pt2face.x = face_rect.x + face_rect.width;
		pt2face.y = face_rect.y + face_rect.height;
		if ((key == 1) || (key == 2)) { // if front pose
			cvRectangle(inputImg, pt1face, pt2face, CV_RGB(255,0,0), 1, 4, 0); //RED (front pose)
		} else if ((key == 3) || (key == 4)) { // if side pose
			cvRectangle(inputImg, pt1face, pt2face, CV_RGB(0,0,255), 1, 4, 0); //BLUE (side pose)
		}
	}
	//Show frames on computer sceen
	///////////////////////////////////
	cvShowImage("FaceDetector - All Bounding Boxes", inputImg);
**/

	if (greyImg) {
		cvReleaseImage(&greyImg);
	}
	cvReleaseMemStorage(&storage);
	
	return rc;	// Return the biggest face found, or (-1,-1,-1,-1).
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



float *simple4(cv::Mat inputImage, size_t arrSize) {
	// Convert from Mat to IplImage
	IplImage* image = cvCloneImage(&(IplImage)inputImage);

	// Declare global variables
	CvPoint pt1, pt2;
	CvRect ptfront, ptside;
	IplImage *imageflipped;
	double frontfacescore, frontfaceflipscore, sidefacescore, sidefaceflipscore;
	float *fdresult = (float*)malloc(sizeof(int)*arrSize);

	// Create a flipped copy of the image (flip, left and right)
	imageflipped = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 3);
	cvFlip(image,imageflipped,1);


	/// Front faces ///
	CvRect facefront = detectFaceInImage(image, frontfaceclassifier, 1);
	frontfacescore = facescore;
	CvRect facefrontflip = detectFaceInImage(imageflipped, frontfaceclassifier, 2);
	frontfaceflipscore = facescore;

	/// Side faces ///
	CvRect faceside = detectFaceInImage(image, sidefaceclassifier, 3);
	sidefacescore = facescore;
	CvRect facesideflip = detectFaceInImage(imageflipped, sidefaceclassifier, 4);
	sidefaceflipscore = facescore;

	// Compute the three face positions //
	scorecenter = frontfacescore + frontfaceflipscore;
	scoreright = sidefacescore;
	scoreleft = sidefaceflipscore;

	// DRAW FACE BOUNDING BOX
	pt1.x = 0; pt1.y = 0; pt2.x = 0; pt2.y = 0;

	// If only front pose of face is detected
	if ((frontfacescore + frontfaceflipscore) > (sidefacescore + sidefaceflipscore)) {
		ptfront.x = (facefront.x + facefrontflip.x) / 2;
		ptfront.y = (facefront.y + facefrontflip.y) / 2;
		ptfront.width = (facefront.width + facefrontflip.width) / 2;
		ptfront.height = (facefront.height + facefrontflip.height) / 2;
		pt1.x = ptfront.x;
		pt1.y = ptfront.y;
		pt2.x = ptfront.width;
		pt2.y = ptfront.height;
		cvRectangle(image, pt1, pt2, CV_RGB(255,0,0), 1, 8, 0); // RED COLOR BOX

	// If only side pose of face is detected
	} else if ((sidefacescore + sidefaceflipscore) > (frontfacescore + frontfaceflipscore)) {
		ptside.x = (faceside.x + facesideflip.x) / 2;
		ptside.y = (faceside.y + facesideflip.y) / 2;
		ptside.width = (faceside.width + facesideflip.width) / 2;
		ptside.height = (faceside.height + facesideflip.height) / 2;
		pt1.x = ptside.x;
		pt1.y = ptside.y;
		pt2.x = ptside.width;
		pt2.y = ptside.height;
		cvRectangle(image, pt1, pt2, CV_RGB(0,0,255), 1, 8, 0);  // BLUE COLOR BOX
	}

	//Compute centroid of face
	////////////////////////////////
	facecentroid.x = (pt1.x + pt2.x) / 2;
	facecentroid.y = (pt1.y + pt2.y) / 2;
	cvCircle(image, facecentroid, 3, CV_RGB(0,255,0), -1);
	//Show frames on computer sceen
	///////////////////////////////////
	cvShowImage("FaceDetector - Mean Bounding Box", image);


	// This rule-based system only works when minneighbors = 0;
	//////// Orientation of Face /////////
	if ((scorecenter <= 15) && (scoreright <= 5) && (scoreleft <= 5)) {
		facedirection = "NO HUMAN PRESENT";
	} else if ((scorecenter > 50) && (scoreright > 0) && (scoreleft > 0)) {
		facedirection = "IN THE CENTER";
	} else if ((scorecenter < 50) && (scoreleft <= 15)) {
		facedirection = "RIGHT ORIENTED";
	} else if ((scorecenter < 50) && (scoreright <= 15)) {
		facedirection = "LEFT ORIENTED";
	}

	//release image
	cvReleaseImage(&imageflipped);

	// return result
	fdresult[0] = facecentroid.x; //face centroid x
	fdresult[1] = facecentroid.y; //face centroid y
	fdresult[2] = pt2.x; //face width
	fdresult[3] = pt2.y; //face height
	fdresult[4] = scorecenter; //face score center
	fdresult[5] = scoreleft; //face score left
	fdresult[6] = scoreright; //face score right
	return fdresult;
}


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


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


int simple2(cv::Mat inputImage) {
		// if green glove is higher than yellow glove --> there is a gesture
		if (centroidgreen.y < centroidyellow.y) {
			return 1;
		// else no gesture detected
		} else {
			return 0;
		}
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


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


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


int main(int argc, const char** argv)
{
	// Capture image
	CvCapture *capture = cvCaptureFromCAM(0);

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
	// Load the HaarCascade classifier for face detection
	frontfaceclassifier = (CvHaarClassifierCascade*)cvLoad("haarcascade_frontalface_default.xml", 0, 0, 0);
	sidefaceclassifier = (CvHaarClassifierCascade*)cvLoad("haarcascade_profileface.xml", 0, 0, 0);


	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Go into main thread
	while (1)
	{
		// Acquire frame from camera as IplImage
		IplImage *frame = cvQueryFrame(capture);
		if(frame==NULL) continue;
                printf("IMAGE: %p\n",frame);
		// Convert from IplImage to Mat
		//cv::Mat image = cvarrToMat(frame, true); 
                cv::Mat image(frame, true);

		// Do processing here
		result1 = simple1(image);
		result2 = simple2(image);
		result3 = simple3(image);
		float *fdresult = simple4(image, 7); // face detector

		// Print results
		printf("Simple r1: %d, r2: %d, r3: %d\n", result1, result2, result3);
		printf("Face: %d,%d,%d,%d,%d,%d,%d - %s\n", (int)fdresult[0],(int)fdresult[1],(int)fdresult[2],(int)fdresult[3],(int)fdresult[4],(int)fdresult[5],(int)fdresult[6],facedirection);

		//release memory
		image.release();
		free(fdresult);

		// Do not release the frame!
		//If ESC key pressed, Key=0x10001B under OpenCV 0.9.7 (Linux version),
		//remove higher bits using AND operator
		if ((cvWaitKey(1) & 255) == 27) break;
	}

	// Free the Face Detector resources when the program is finished
	cvReleaseHaarClassifierCascade(&frontfaceclassifier);
	cvReleaseHaarClassifierCascade(&sidefaceclassifier);

	// Release the capture device housekeeping
	cvReleaseCapture(&capture);
	cvDestroyWindow("User Interface");

	return 0;
}



/**
Just make a function like simple1, which returns an array of three float
No,  five float
 Sent at 3:45 AM on Saturday
 Alessandro:  Ret[0] = face centroid x (or -1 if there is no face)
Ret[1]=face centroid y
 Sent at 3:47 AM on Saturday
 Alessandro:  Ret[2]=face width
 Sent at 3:48 AM on Saturday
 Alessandro:  Ret[3]=score face left
Ret[34=score face center
Ret[5]=score face right
So, six float
In the function just search faces
**/

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



