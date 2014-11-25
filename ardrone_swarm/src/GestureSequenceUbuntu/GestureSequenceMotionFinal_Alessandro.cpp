#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "time.h"
#include "unistd.h"
#include "svm.h"
#include "Blob.h"
#include "BlobResult.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>

using namespace std;
using namespace cv;

#define NDATA 14
#define NFRAMES 1


// Set camera properties
///////////////////////////////
//1920×1080 -- full hd 1080p
//1280x720 -- hd 720p
static int fps = 30; // brightness, contrast, saturation;
static int framewidth = 1280;
static int frameheight = 720;

///////////////////////////////
//Define time intervals
static long int wait_time_before_start = 8000*1000; //10 secs
static long int time_between_indrobots = 3000*1000; //3 secs
static long int time_between_commands = 6000*1000; //6 secs
static double max_gesture_wait_time = 15; //15 secs
double glovesizeratio = 0.05; //5%
double jacketsizeratio = 0.1; //10%
double motionthreshold = 0.1; //range between [0,0.5]
int minyellowglovearea = 100; //to detect a gesture
int mingreenglovearea = 100;  //to detect a gesture
///////////////////////////////

//Bounding box for segmented image
static int dim = 28;
static int box = 32;

//General global variables
CvCapture* capture;
CvFont font, font2;
CvPoint bodyline;
char timetemp[15];
double previousmotion[NFRAMES][NDATA], currentmotion[NFRAMES][NDATA];
int resultfingercounts, facescore, timeelapsed, individualctr = 0;
int resultclassifier, resultint, resultyou, resultaction, faceheight;
double directionyellow, directiongreen, circlescore, motionscore = 0;
bool recording;
bool firstrun = FALSE;
bool threadstart = FALSE;
bool directionshown = FALSE;
bool quitreason = FALSE;
bool timerrunning = FALSE;
bool stage1 = FALSE;
bool stage2 = FALSE;
char *facedirection = "";
char *gesturemode = "";
char *resultfirst = "";
char *resultsecond = "";
char *currentresult = "";
char resultthird[50];
char resultfourth[50];
IplImage *frame;


//Smallest face size
CvSize minFeatureSize = cvSize(24,24);
CvHaarClassifierCascade *frontfaceclassifier, *sidefaceclassifier;
// Classifiers
char *model1 = "model-fingercounts.txt";
char *model2 = "model-direction.txt";
char *frontclassifier = "haarcascade_frontalface_default.xml";
char *sideclassifier = "haarcascade_profileface.xml";
//haarcascade_frontalface_alt.xml";
//haarcascade_frontalface_alt2.xml";
//haarcascade_frontalface_alt_tree.xml";

//Centroids of gloves and jackets
CvPoint centroidyellow, centroidgreen, centroidjacket;
CvPoint jacketubound, jacketlbound, jacketright, jacketleft;
double distgreen, distyellow, distjacketupper, distjacketcenter, distjacketlower; //distances used for motion detection
int distancegloves, distancejacket; // distance used as features for classifying gestures


//Define color thresholds
//Yellow
int Hmin_Y = 22;
int Hmax_Y = 58;
int Smin_Y = 80;
int Smax_Y = 255;
int Vmin_Y = 10;
int Vmax_Y = 255;
//Green
int Hmin_G = 52;
int Hmax_G = 104;
int Smin_G = 70;
int Smax_G = 255;
int Vmin_G = 10;
int Vmax_G = 190;
//Orange
int Hmin_O = 0;
int Hmax_O = 31;
int Smin_O = 128;
int Smax_O = 255;
int Vmin_O = 87;
int Vmax_O = 255;


//Define feature parameters
double *features;
static const int featuresize = 95;

//Normalization values for all models
static double fmin_fingercounts[featuresize] = {0.301296000000000,0.545524000000000,0.461199000000000,2,3.23833100000000,0.163196000000000,14.6216130000000,2,14,13.3000010000000,4,0.365392000000000,0.454035000000000,67.6156430000000,6.50000000000000,0.250976000000000,0.631971000000000,1.12235500000000,0.0481850000000000,15,64.5703490000000,12.3078240000000,0.263212000000000,23,225.500000000000,270,10,1.00009800000000,1.14704200000000,10,10.7611840000000,18.4264310000000,67.7989900000000,52.1500840000000,0.370370000000000,5.11845500000000,0.0315490000000000,2.35063300000000,-0.0369360000000000,3.54090700000000,0.162436000000000,3994.83070300000,0.000109000000000000,5.17080700000000,0.000237000000000000,6,21,1379.85325600000,1.01825500000000,44232.7500000000,1115587.25000000,2893.66666700000,-32037.7944800000,697085.400000000,0.637964000000000,1115495.85000000,194.500000000000,1.51275000000000,0.0410650000000000,44213.2500000000,696514.550000000,2893.16666700000,194.500000000000,-0.0126630000000000,15.7367430000000,0.145572000000000,784747.200000000,64007.0833300000,784187.383300000,64011.9166700000,779388.650000000,9,50436.6666700000,781715.983300000,3376,0,50475.5833300000,69.4713760000000,-25011.8438100000,3375.50000000000,12.1529410000000,11.5000000000000,2,4,52.7653780000000,-58947.9860900000,2,-30627.5410000000,90.0062640000000,-0.0180540000000000,276.846185000000,1.15566300000000,15.7367430000000,0.241564000000000,1.33555700000000};
static double fmax_fingercounts[featuresize] = {0.760129000000000,0.870370000000000,0.870370000000000,8,4.03239200000000,0.270394000000000,18.7688980000000,24,20,18.3800490000000,29,0.890984000000000,0.930854000000000,130.687575000000,147.500000000000,0.311539000000000,2.72025100000000,2.73679000000000,0.238152000000000,27,92.5569290000000,16.1167350000000,0.883837000000000,29,625,729,27,10.9136090000000,4.13969600000000,33.5410190000000,33.3151280000000,68.7256560000000,150.124890000000,150.045045000000,1.80000000000000,22.5710680000000,0.159033000000000,6.15628300000000,0.0169470000000000,12.7726490000000,0.425417000000000,28925.1670400000,1,13.0467040000000,0.0425500000000000,17,29,29516.4227200000,1.65888200000000,171321.250000000,3603548.30000000,9131.83333300000,28050.5622500000,3571054.35000000,0.990142000000000,3654234.30000000,593,2.01124200000000,0.0717030000000000,169391.333300000,3524485.75000000,9045.50000000000,582.500000000000,0.0298570000000000,27.4778280000000,0.254182000000000,2542443.61700000,172329.250000000,2603008.55000000,174434.500000000,2631415.25000000,29,138456.500000000,2578314.28300000,9285.33333300000,90,135993.750000000,197.310108000000,53488.7014900000,9192.66666700000,18.6603970000000,20,11,26,263.762588000000,24590.6194400000,11,28451.4514600000,269.976379000000,0.00970100000000000,733.097124000000,1.85864400000000,27.2334730000000,0.871807000000000,3.45455600000000};
static double fmin_direction[featuresize] = {0.257283000000000,0.545524000000000,0.461199000000000,2,3.23833100000000,0.163196000000000,13.2797780000000,2,12.5000000000000,13.2500020000000,4,0.267157000000000,0.451161000000000,64.0104700000000,4.50000000000000,0.250976000000000,0.631971000000000,1.12052000000000,0.0293910000000000,10,63.2637920000000,12.3078240000000,0.263212000000000,21,176,189,7,1.00009800000000,1.14704200000000,7,9,18.4264310000000,64.4852810000000,0,0,0,0,0,-0.0530840000000000,0,0,0,0,0,0,0,0,0,0,0,0,0,-35059.4145100000,0,0,0,0,0,0,0,0,0,0,-0.0126630000000000,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-35225.2520900000,0,0,0,0,0,0,-58947.9860900000,0,-30627.5410000000,0,-0.0180540000000000,0,0,0,0,0};
static double fmax_direction[featuresize] = {0.760129000000000,0.929293000000000,0.929293000000000,11,4.03239200000000,0.343822000000000,18.7688980000000,24,20,18.3800490000000,29,0.892443000000000,0.963653000000000,130.687575000000,147.500000000000,0.311539000000000,2.72025100000000,3.74311700000000,0.315157000000000,27,93.0821910000000,17.8244590000000,0.883837000000000,29,625,729,27,10.9136090000000,4.13969600000000,35.2234650000000,35.4241940000000,68.7256560000000,150.124890000000,150.045045000000,2.70000000000000,22.9852810000000,0.233829000000000,6.15628300000000,0.0635310000000000,13.0011900000000,0.425417000000000,28925.1670400000,1,13.0467040000000,0.0883080000000000,17,29,29782.1387500000,1.65888200000000,184714.166700000,4042570.35000000,9526,38386.4131900000,3917015.40000000,0.990142000000000,4070635.80000000,593,2.01124200000000,0.0777320000000000,184295.166700000,3906845.05000000,9507.83333300000,582.500000000000,0.0298570000000000,27.4778280000000,0.275554000000000,2727693.58300000,185913.166700000,2737430.78300000,186856.333300000,2790267.25000000,29,144085.166700000,2780918.53300000,9361,90,143739.791700000,197.310108000000,53488.7014900000,9330.16666700000,18.6603970000000,20,13,26,270.834091000000,24590.6194400000,12,46077.6818100000,269.976379000000,0.00970100000000000,733.097124000000,1.86236300000000,27.2334730000000,0.871807000000000,3.46839500000000};
static int lowerlimit = -3;
static int upperlimit = +3;



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// compute shape features
void computeallfeatures(CBlobResult blobs, CBlob biggestBlob, int classifier)
{

	// Get cvblobslib features
    double area = blobs.GetNumber(0,CBlobGetArea());
    double areaellipseratio = blobs.GetNumber(0,CBlobGetAreaElipseRatio());
    //double axisratio = blobs.GetNumber(0,CBlobGetAxisRatio());
    double breadth = blobs.GetNumber(0,CBlobGetBreadth());
    double compactness = blobs.GetNumber(0,CBlobGetCompactness());
    double diffx = blobs.GetNumber(0,CBlobGetDiffX());
    double diffy = blobs.GetNumber(0,CBlobGetDiffY());
    double elongation = blobs.GetNumber(0,CBlobGetElongation());
    double hullarea = blobs.GetNumber(0,CBlobGetHullArea());
    double hullperimeter = blobs.GetNumber(0,CBlobGetHullPerimeter());
    double length = blobs.GetNumber(0,CBlobGetLength());
    double majoraxislength = blobs.GetNumber(0,CBlobGetMajorAxisLength());
    double maxX = blobs.GetNumber(0,CBlobGetMaxX());
    double maxXatmaxY = blobs.GetNumber(0,CBlobGetMaxXatMaxY());
    double maxY = blobs.GetNumber(0,CBlobGetMaxY());
    double maxYatminX = blobs.GetNumber(0,CBlobGetMaxYatMinX());
    double minoraxislength = blobs.GetNumber(0,CBlobGetMinorAxisLength());
    double minX = blobs.GetNumber(0,CBlobGetMinX());
    double minXatMinY = blobs.GetNumber(0,CBlobGetMinXatMinY());
    double minY = blobs.GetNumber(0,CBlobGetMinY());
    double minYatMaxX = blobs.GetNumber(0,CBlobGetMinYatMaxX());
    double orientation = blobs.GetNumber(0,CBlobGetOrientation());
    double orientationcos = blobs.GetNumber(0,CBlobGetOrientationCos());
    double perimeter = blobs.GetNumber(0,CBlobGetPerimeter());
    double roughness = blobs.GetNumber(0,CBlobGetRoughness());
    double centroidx = blobs.GetNumber(0,CBlobGetXCenter());
    double centroidy = blobs.GetNumber(0,CBlobGetYCenter());
    //double centroidx = ((minX + maxX) / 2);
    //double centroidy = ((minY + maxY) / 2);
    double feret = majoraxislength;
	
/**	/////////// Compute Centroid ///////////
	//CBlobGetXCenter centroidx;
	//CBlobGetYCenter centroidy;
	double mom01 = biggestBlob.Moment(0,1);
	double mom10 = biggestBlob.Moment(1,0);
	double area = biggestBlob.Area();
	//Compute blob centroid
	int centroidx = mom10/area;
	int centroidy = mom01/area;
**/

	// Get contour from biggestBlob
	CvMemStorage* storage = cvCreateMemStorage(0);
	CvSeq* contour = NULL;
	CvMoments moments;
	CvHuMoments humoments;

    // Contour approxiamation to use
    contour = cvApproxPoly(biggestBlob.GetExternalContour()->GetContourPoints(), sizeof(CvContour), storage, CV_POLY_APPROX_DP, perimeter*0.005, 1);
    // Calculate moments
    cvMoments(contour, &moments);
    cvGetHuMoments(&moments, &humoments);

	// Get Contour features
	//double contourarea = area;
	//double contourperimeter = perimeter;
	double contourarea = fabs(cvContourArea((CvSeq*)contour, CV_WHOLE_SEQ,0));
	double contourperimeter = fabs(cvArcLength(contour, CV_WHOLE_SEQ, -1));


    // Get MATLAB shape features
    double matlab_formfactor = (4 * CV_PI * contourarea) / pow(contourperimeter,2); // also known as circularity
    double matlab_roundness = (4 * contourarea) / (CV_PI * pow(majoraxislength,2));
    double matlab_roundness2 = (4 * contourarea) / (CV_PI * sqrt(majoraxislength));
    double matlab_compactness = (sqrt((4 * contourarea) / CV_PI) / majoraxislength);
    //double matlab_compactness2 = ((4 * CV_PI * contourarea) / pow(contourperimeter,2));
    //double matlab_compactness3 = ((contourperimeter * contourperimeter) / contourarea);
    double matlab_compactness4 = (contourperimeter / contourarea);
    //double matlab_Hcompactness = (majoraxislength * matlab_compactness);
    double matlab_Hcompactness2 = (majoraxislength * ((4 * CV_PI * contourarea) / pow(contourperimeter,2)));
    double matlab_aspectratio = (majoraxislength / minoraxislength); // also known as Elongation/Symmetry
    double matlab_apratio = (contourarea / contourperimeter);
    double matlab_equivdiameter = sqrt((4 * contourarea) / CV_PI);
    double matlab_solidity = (contourarea / hullarea);
    //double matlab_perimequivdiamater = (contourarea / CV_PI);
    double matlab_areaequivdiamater = sqrt((4 * CV_PI) / contourarea);
    double matlab_concavity = (hullarea - contourarea);
    double matlab_convexity = (hullperimeter / contourperimeter);
    double matlab_eccentricity = (2 * sqrt(pow(majoraxislength/2,2) - pow(minoraxislength/2,2)) / majoraxislength);
    int boxheight = biggestBlob.GetBoundingBox().height;
    int boxwidth = biggestBlob.GetBoundingBox().width;
    int boundingboxarea = (boxheight * boxwidth);
    double matlab_extent = (contourarea / boundingboxarea);

	//new features (2013)
	//double exterior = blobs.GetNumber(0,CBlobGetExterior());
	//double externhullperimeterratio = blobs.GetNumber(0,CBlobGetExternHullPerimeterRatio());
	//double externperimeter = blobs.GetNumber(0,CBlobGetExternPerimeter());
	//double externperimeterratio = blobs.GetNumber(0,CBlobGetExternPerimeterRatio());
	//double mean = blobs.GetNumber(0,CBlobGetMean());
	//double stddev = blobs.GetNumber(0,CBlobGetStdDev());
	double formfactor2 = (4 * CV_PI * area) / sqrt(perimeter);
	double compactness2 = sqrt((4 / CV_PI) * area) / majoraxislength;
	double effdiameter2 = sqrt(area / CV_PI)*2;
	double circularity2 = (4* CV_PI) * (area/pow(perimeter,2)); //Compactness
	double roundness2 = 4 * (area/(CV_PI*pow(majoraxislength,2)));
/**
	Form factor: 4*pi*area / sqr(perimeter) (circularity)
	Roundness: 4*area/ pi*sqr(major axis)
	Compactness: sqrt((4/pi)*area)/major axis
	Aspect Ratio: major axis/minor axis
	Effective diameter: sqrt(area/pi)*2
	Solidity: area/convex area
	Convexity: convex perimeter/perimeter
**/
    // Get Feret features
    double feretaspectratio = (feret / breadth);
    double boxaspectratio = ((maxX - minX) / (maxY - minY));
    //double feretbox = (feret * breadth);
    double rfactor = (contourperimeter / (feret * CV_PI));
    double convexity = (contourperimeter / perimeter);
    double concavity = fabs(contourarea - area);
    double rectangularity = (contourarea / (feret * breadth));
    //double areaeffectdiameter = (sqrt(contourarea / CV_PI) * 2);
    double ellipseeffectarea = ((CV_PI * feret * breadth) / 4);
    //double modificationratio = (minoraxislength / feret);
    double sphericity = ((minoraxislength / 2) / (majoraxislength / 2));
    double rfactor2 = (hullperimeter / (feret * CV_PI));
    //double modificationratio2 = ((2 * minoraxislength) / feret);

    // Get Moment features
    //double matlab_m00 = biggestBlob.Moment(0,0);
    double matlab_m01 = biggestBlob.Moment(0,1);
    double matlab_m02 = biggestBlob.Moment(0,2);
    double matlab_m03 = biggestBlob.Moment(0,3);
    double matlab_m10 = biggestBlob.Moment(1,0);
    double matlab_m11 = biggestBlob.Moment(1,1);
    double matlab_m12 = biggestBlob.Moment(1,2);
    double matlab_m20 = biggestBlob.Moment(2,0);
    double matlab_m21 = biggestBlob.Moment(2,1);
    double matlab_m30 = biggestBlob.Moment(3,0);
    double inv_sqrt_m00 = moments.inv_sqrt_m00;
    //double spat_m00 = cvGetSpatialMoment(&moments, 0, 1);
    double spat_m01 = cvGetSpatialMoment(&moments, 0, 1);
    double spat_m02 = cvGetSpatialMoment(&moments, 0, 2);
    double spat_m03 = cvGetSpatialMoment(&moments, 0, 3);
    double spat_m10 = cvGetSpatialMoment(&moments, 1, 0);
    double spat_m11 = cvGetSpatialMoment(&moments, 1, 1);
    double spat_m12 = cvGetSpatialMoment(&moments, 1, 2);
    double spat_m20 = cvGetSpatialMoment(&moments, 2, 0);
    double spat_m21 = cvGetSpatialMoment(&moments, 2, 1);
    double spat_m30 = cvGetSpatialMoment(&moments, 3, 0);
    //double cent_m00 = cvGetCentralMoment(&moments, 0, 0);
    double cent_m11 = cvGetCentralMoment(&moments, 1, 1);
    double cent_m20 = cvGetCentralMoment(&moments, 2, 0);
    double cent_m02 = cvGetCentralMoment(&moments, 0, 2);
    double cent_m21 = cvGetCentralMoment(&moments, 2, 1);
    double cent_m12 = cvGetCentralMoment(&moments, 1, 2);
    double cent_m30 = cvGetCentralMoment(&moments, 3, 0);
    double cent_m03 = cvGetCentralMoment(&moments, 0, 3);
    //double normcent_00 = cvGetNormalizedCentralMoment(&moments, 0, 0);
    double normcent_11 = cvGetNormalizedCentralMoment(&moments, 1, 1);
    double normcent_20 = cvGetNormalizedCentralMoment(&moments, 2, 0);
    double normcent_02 = cvGetNormalizedCentralMoment(&moments, 0, 2);
    double normcent_21 = cvGetNormalizedCentralMoment(&moments, 2, 1);
    //double normcent_12 = cvGetNormalizedCentralMoment(&moments, 1, 2);
    double normcent_30 = cvGetNormalizedCentralMoment(&moments, 3, 0);
    double normcent_03 = cvGetNormalizedCentralMoment(&moments, 0, 3);
    double hu1 = humoments.hu1;
    double hu2 = humoments.hu2;
    //double hu3 = humoments.hu3;
    //double hu4 = humoments.hu4;
    //double hu5 = humoments.hu5;
    //double hu6 = humoments.hu6;
    //double hu7 = humoments.hu7;


    // Get Convexity defects
    CvMemStorage* storage2 = cvCreateMemStorage(0);
    CvSeq* seqhull = NULL;
    CvSeq* defects = NULL;
    seqhull = cvConvexHull2(contour, 0, CV_CLOCKWISE, 0);
    defects = cvConvexityDefects(contour, seqhull, storage2);
    double convexitydefects = defects->total;
    double seqhulltotal = seqhull->total;

    // Get Minimum Enclosing Circle
    float radius;
    CvPoint2D32f MEC;
    cvMinEnclosingCircle(contour, &MEC, &radius);
    double centroid1x = MEC.x;
    double centroid1y = MEC.y;
    double circlearea = (CV_PI * radius * radius);
    double extentcircle = (contourarea / circlearea);
    double soliditycircle = (circlearea / hullarea);

    // Get Minimum Area Rectangle
    CvBox2D MAR;
    CvMemStorage* storage3 = cvCreateMemStorage(0);
    MAR = cvMinAreaRect2(contour, storage3);
    double minarearectangle = fabs(MAR.angle);
    double minarearectsizeheight = MAR.size.height;
    double minarearectsizewidth = MAR.size.width;
    double centroid2x = MAR.center.x;
    double centroid2y = MAR.center.y;
    double rectarea = (MAR.size.height * MAR.size.width);
    double extentrect = (contourarea / rectarea);
    double solidityrect = (rectarea / hullarea);

    // Get Bounding Box
    CvRect BB;
    BB = biggestBlob.GetBoundingBox();
    //double bbheight = BB.height;
    //double bbwidth = BB.width;
    //double bbx = BB.x;
    //double bby = BB.y;
    double centroid3x = BB.x + BB.width/2;
    double centroid3y = BB.y + BB.height/2;
    double bbarea = (BB.height * BB.width);
    //double extentbb = (contourarea / bbarea);
    double soliditybb = (bbarea / hullarea);

    // Get Ellipse
    CvBox2D FE;
    FE = biggestBlob.GetEllipse();
    //double ellipseangle = FE.angle;
    //double ellipsesizeheight = FE.size.height;
    //double ellipsesizewidth = FE.size.width;
    double centroid4x = FE.center.x;
    double centroid4y = FE.center.y;
    double ellipsearea = (CV_PI * (FE.size.height/2) * (FE.size.width/2));
    double extentellipse = (contourarea / ellipsearea);
    double solidityellipse = (ellipsearea / hullarea);


	//Allocate memory for features
	features = (double*) malloc(featuresize*sizeof(double));

	//Assign features values to vector
	features[0] = extentcircle;
	features[1] = extentrect;
	features[2] = matlab_extent;
	features[3] = minY;
	features[4] = extentellipse;
	features[5] = hu1;
	features[6] = centroid4y;
	features[7] = minYatMaxX;
	features[8] = centroid1y;
	features[9] = centroid2y;
	features[10] = maxYatminX;
	features[11] = sphericity;
	features[12] = matlab_eccentricity;
	features[13] = contourperimeter;
	features[14] = matlab_concavity;
	features[15] = areaellipseratio;
	features[16] = feretaspectratio;
	features[17] = matlab_aspectratio;
	features[18] = normcent_02;
	features[19] = diffy;
	features[20] = hullperimeter;
	features[21] = majoraxislength;
	features[22] = matlab_formfactor;
	features[23] = maxY;
	features[24] = hullarea;
	features[25] = rectarea;
	features[26] = diffx;
	features[27] = elongation;
	features[28] = compactness;
	features[29] = minarearectsizewidth;
	features[30] = minarearectsizeheight;
	features[31] = length;
	features[32] = perimeter;
	features[33] = ellipsearea;
	features[34] = boxaspectratio;
	features[35] = breadth;
	features[36] = normcent_20;
	features[37] = matlab_apratio;
	features[38] = normcent_03;
	features[39] = matlab_Hcompactness2;
	features[40] = matlab_compactness4;
	features[41] = cent_m02;
	features[42] = orientationcos;
	features[43] = minoraxislength;
	features[44] = hu2;
	features[45] = seqhulltotal;
	features[46] = maxX;
	features[47] = cent_m20;
	features[48] = roughness;
	features[49] = spat_m20;
	features[50] = matlab_m03;
	features[51] = spat_m10;
	features[52] = cent_m03;
	features[53] = spat_m30;
	features[54] = matlab_convexity;
	features[55] = spat_m03;
	features[56] = contourarea;
	features[57] = rfactor2;
	features[58] = inv_sqrt_m00;
	features[59] = matlab_m20;
	features[60] = matlab_m30;
	features[61] = matlab_m10;
	features[62] = area;
	features[63] = normcent_30;
	features[64] = matlab_equivdiameter;
	features[65] = matlab_areaequivdiamater;
	features[66] = matlab_m12;
	features[67] = matlab_m02;
	features[68] = spat_m12;
	features[69] = spat_m02;
	features[70] = spat_m21;
	features[71] = maxXatmaxY;
	features[72] = spat_m11;
	features[73] = matlab_m21;
	features[74] = spat_m01;
	features[75] = minarearectangle;
	features[76] = matlab_m11;
	features[77] = matlab_roundness2;
	features[78] = cent_m30;
	features[79] = matlab_m01;
	features[80] = centroid2x;
	features[81] = centroid1x;
	features[82] = convexitydefects;
	features[83] = minXatMinY;
	features[84] = ellipseeffectarea;
	features[85] = cent_m21;
	features[86] = minX;
	features[87] = cent_m12;
	features[88] = orientation;
	features[89] = normcent_21;
	features[90] = formfactor2;
	features[91] = compactness2;
	features[92] = effdiameter2;
	features[93] = circularity2;
	features[94] = roundness2;


    // Normalize feature values
	int i;
    for (i=0; i<featuresize; i++)
    {
		//perform normalization, based on normalization values for each individual classifier model
		if (classifier == 1) { // fingercounts
			features[i] = lowerlimit + (upperlimit-lowerlimit) * (features[i] - fmin_fingercounts[i]) / (fmax_fingercounts[i] - fmin_fingercounts[i]);
		} else if (classifier == 2) { // direction
			features[i] = lowerlimit + (upperlimit-lowerlimit) * (features[i] - fmin_direction[i]) / (fmax_direction[i] - fmin_direction[i]);
		}

		// if any of the feature values are nan or inf, adjust the values accordingly
		if (cvIsNaN(features[i])) {
			features[i] = lowerlimit;
		}
		if (cvIsInf(features[i])) {
			features[i] = upperlimit;
		}

		// correct normalized feature values if larier than limits
        if (features[i] > upperlimit) {
           features[i] = upperlimit;
        } else if (features[i] < lowerlimit) {
           features[i] = lowerlimit;
        }
    }

    // Release and clear memory
    cvClearMemStorage(storage);
    cvClearMemStorage(storage2);
    cvClearMemStorage(storage3);
    if (contour) {
	cvClearSeq(contour);
    }
    if (seqhull) {
	cvClearSeq(seqhull);
    }
    if (defects) {
	cvClearSeq(defects);
    }
    cvReleaseMemStorage(&storage);
    cvReleaseMemStorage(&storage2);
    cvReleaseMemStorage(&storage3);

    //Clear blobs
    blobs.ClearBlobs();
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// Returns a new image that is a cropped version (rectangular cut-out)
// of the original image.
IplImage* cropImage(const IplImage *img, const CvRect region)
{
    IplImage *imageCropped;
    CvSize size;

    if (img->width <= 0 || img->height <= 0
        || region.width <= 0 || region.height <= 0) {
        //cerr << "ERROR in cropImage(): invalid dimensions." << endl;
        exit(1);
    }

    if (img->depth != IPL_DEPTH_8U) {
        //cerr << "ERROR in cropImage(): image depth is not 8." << endl;
        exit(1);
    }

    // Set the desired region of interest.
    cvSetImageROI((IplImage*)img, region);
    // Copy region of interest into a new iplImage and return it.
    size.width = region.width;
    size.height = region.height;
    imageCropped = cvCreateImage(size, IPL_DEPTH_8U, img->nChannels);
    cvCopy(img, imageCropped);    // Copy just the region.

    return imageCropped;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// Creates a new image copy that is of a desired size. The aspect ratio will
// be kept constant if 'keepAspectRatio' is true, by cropping undesired parts
// so that only pixels of the original image are shown, instead of adding
// extra blank space.
// Remember to free the new image later.
IplImage* resizeImage(const IplImage *origImg, int newWidth,
    int newHeight, bool keepAspectRatio)
{
    IplImage *outImg = 0;
    int origWidth;
    int origHeight;
    if (origImg) {
        origWidth = origImg->width;
        origHeight = origImg->height;
    }
    if (newWidth <= 0 || newHeight <= 0 || origImg == 0
        || origWidth <= 0 || origHeight <= 0) {
        //cerr << "ERROR: Bad desired image size of " << newWidth
        //    << "x" << newHeight << " in resizeImage().\n";
        exit(1);
    }

    if (keepAspectRatio) {
        // Resize the image without changing its aspect ratio,
        // by cropping off the edges and enlarging the middle section.
        CvRect r;
        // input aspect ratio
        float origAspect = (origWidth / (float)origHeight);
        // output aspect ratio
        float newAspect = (newWidth / (float)newHeight);
        // crop width to be origHeight * newAspect
        if (origAspect > newAspect) {
            int tw = (origHeight * newWidth) / newHeight;
            r = cvRect((origWidth - tw)/2, 0, tw, origHeight);
        }
        else {    // crop height to be origWidth / newAspect
            int th = (origWidth * newHeight) / newWidth;
            r = cvRect(0, (origHeight - th)/2, origWidth, th);
        }
        IplImage *croppedImg = cropImage(origImg, r);

        // Call this function again, with the new aspect ratio image.
        // Will do a scaled image resize with the correct aspect ratio.
        outImg = resizeImage(croppedImg, newWidth, newHeight, false);
        cvReleaseImage(&croppedImg);

    }
    else {

        // Scale the image to the new dimensions,
        // even if the aspect ratio will be changed.
        outImg = cvCreateImage(cvSize(newWidth, newHeight),
            origImg->depth, origImg->nChannels);
        if (newWidth > origImg->width && newHeight > origImg->height) {
            // Make the image larger
            cvResetImageROI((IplImage*)origImg);
            // CV_INTER_LINEAR: good at enlarging.
            // CV_INTER_CUBIC: good at enlarging.          
            cvResize(origImg, outImg, CV_INTER_LINEAR);
        }
        else {
            // Make the image smaller
            cvResetImageROI((IplImage*)origImg);
            // CV_INTER_AREA: good at shrinking (decimation) only.
            cvResize(origImg, outImg, CV_INTER_AREA);
        }

    }
    return outImg;
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



void computeblobfeatures(IplImage *image, CBlob blob, int classifier)
{
	// Get bounding box around biggest blob
	CvRect handRect;
	handRect = blob.GetBoundingBox();

	// Copy image ROI
	cvSetImageROI(image,handRect);
	IplImage *imageROI = cvCreateImage(cvSize(handRect.width, handRect.height), IPL_DEPTH_8U, 1);
	cvCopy(image,imageROI,NULL);
	cvResetImageROI(imageROI);

	// Resize image
	int dimWidth = (handRect.width >= handRect.height) ? dim : static_cast<int>(handRect.width * (dim / (float)handRect.height));
	int dimHeight = (handRect.width > handRect.height) ? static_cast<int>(handRect.height * (dim / (float)handRect.width)) : dim;
	IplImage *imageNewSize = resizeImage(imageROI, dimWidth, dimHeight, false);
	IplImage *imageBox = cvCreateImage(cvSize(box,box),IPL_DEPTH_8U, 1);

	//Convert all pixel values to black
	int i,j;
	int width2	 =	imageBox->width;
	int height2	 =	imageBox->height;
	int step2	 =	imageBox->widthStep/sizeof(uchar);
	uchar* data2 =	(uchar *)imageBox->imageData;
	 
	// Fill the image with black pixels
	for (i=0; i<(height2); i++)
	{
		for (j=0; j<(width2); j++)
		{
			data2[i*step2 + j] = 0;
		}
	}

	//Get bounding rectangle box for black image for superimposing
	CvRect handRect2;
	handRect2.x = (box/2) - (dimWidth/2);
	handRect2.y = (box/2) - (dimHeight/2);
	handRect2.height = dimHeight;
	handRect2.width = dimWidth;

	//Get ROI from the blank white image
	cvSetImageROI(imageBox,handRect2);
	//Copy image ROI to big image with black bordered pixels
	cvCopy(imageNewSize,imageBox,NULL);
	cvResetImageROI(imageBox);

	// Get the only blob
	CBlobResult finalblobs;
	finalblobs = CBlobResult(imageBox, NULL, 0);
	CBlob biggestfinalBlob;
	finalblobs.GetNthBlob(CBlobGetArea(), 0, biggestfinalBlob);

	////// Compute features of the blob //////
	computeallfeatures(finalblobs, biggestfinalBlob, classifier);

	//release and free memory
	cvReleaseImage(&image); //not sure
	cvReleaseImage(&imageROI);
	cvReleaseImage(&imageNewSize);
	cvReleaseImage(&imageBox);
	finalblobs.ClearBlobs();
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



int segmentandpredict(int glove, int classifier)
{
	// Convert RGB to HSV format //
	int predicted_label;
	IplImage *imageHSV = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3);
	cvCvtColor(frame, imageHSV, CV_BGR2HSV);

	//// HSV Thresholding ////
	///////////////////////////////////////////////////////////////////////////////////////

	// YELLOW blob //
	if (glove == 1) {
		IplImage *imageYellow = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 1);   // Grayscale output image
		cvInRangeS(imageHSV, cvScalar(Hmin_Y, Smin_Y, Vmin_Y), cvScalar(Hmax_Y, Smax_Y, Vmax_Y), imageYellow); // Yellow threshold
		CBlobResult blobsYellow;
		blobsYellow = CBlobResult(imageYellow, NULL, 0);
		CBlob biggestYellowBlob;
		blobsYellow.GetNthBlob(CBlobGetArea(), 0, biggestYellowBlob);
		biggestYellowBlob.FillBlob(imageYellow, CV_RGB(255,255,255), 0, 0);
		
		//get final segmented blob and compute features
		computeblobfeatures(imageYellow, biggestYellowBlob, classifier);

		//Predict using SVM
		if (classifier == 1) { //FINGERCOUNTS
			predicted_label = svm_do_prediction_passmemory(model1,features);			
		} else if (classifier == 2) { //DIRECTION
			predicted_label = svm_do_prediction_passmemory(model2,features);
		}

		//free memory
		free(features);
		blobsYellow.ClearBlobs();


	// GREEN blob //
	} else if (glove == 2) {
		IplImage *imageGreen = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 1);   // Grayscale output image
		cvInRangeS(imageHSV, cvScalar(Hmin_G, Smin_G, Vmin_G), cvScalar(Hmax_G, Smax_G, Vmax_G), imageGreen); // Green threshold
		CBlobResult blobsGreen;
		blobsGreen = CBlobResult(imageGreen, NULL, 0);
		CBlob biggestGreenBlob;
		blobsGreen.GetNthBlob(CBlobGetArea(), 0, biggestGreenBlob);
		biggestGreenBlob.FillBlob(imageGreen, CV_RGB(255,255,255), 0, 0);

		//get final segmented blob and compute features
		computeblobfeatures(imageGreen, biggestGreenBlob, classifier);
		//Predict using SVM
		predicted_label = svm_do_prediction_passmemory(model1,features);

		//free memory
		free(features);
		blobsGreen.ClearBlobs();
	}

	//release memory
	cvReleaseImage(&imageHSV);

	//return result as a vector of integers (if the second cell is not 0, only then two images were processed)
	return predicted_label;
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



int computepredictionresults(int classifier)
{
	//Initialize
	int gloveg, glovey, total;

	//Segment, compute features and predict
	////////////////////////////////////////////////////////////
	if (classifier == 1) { //FINGERCOUNTS
		glovey = segmentandpredict(1, classifier); // yellow glove
		gloveg = segmentandpredict(2, classifier); // green glove

		//total finger counts
		total = (gloveg - 1) + (glovey - 1);

	} else if (classifier == 2) { //DIRECTION
		total = segmentandpredict(1, classifier); // yellow glove
	}

	//return the total number of fingercounts for both hands
	return total;
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



double computecirclescore()
{
	// Initialize
	double area, majoraxislength, minoraxislength, perimeter, aspectratio, circularity, roundness;

	/// Copy frame ///
	IplImage *image = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3);
	cvCopy(frame,image,NULL);

	// Convert RGB to HSV format //
	IplImage *imageHSV = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 3);
	cvCvtColor(image, imageHSV, CV_BGR2HSV);

	// Segment GREEN Blob //
	IplImage *imageGreen = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);   // Grayscale output image
	cvInRangeS(imageHSV, cvScalar(Hmin_G, Smin_G, Vmin_G), cvScalar(Hmax_G, Smax_G, Vmax_G), imageGreen); // Green threshold
	CBlobResult blobsGreen;
	blobsGreen = CBlobResult(imageGreen, NULL, 0);
	CBlob biggestGreenBlob;
	blobsGreen.GetNthBlob(CBlobGetArea(), 0, biggestGreenBlob);
	blobsGreen.Filter(blobsGreen, B_EXCLUDE, CBlobGetArea(), B_LESS, biggestGreenBlob.Area());

	//Compute blob properties
	area = blobsGreen.GetNumber(0,CBlobGetArea());
	majoraxislength = blobsGreen.GetNumber(0,CBlobGetMajorAxisLength());
	minoraxislength = blobsGreen.GetNumber(0,CBlobGetMinorAxisLength());
	perimeter = blobsGreen.GetNumber(0,CBlobGetPerimeter());
	//Compute features
	aspectratio = (majoraxislength / minoraxislength); // also known as Elongation/Symmetry
	circularity = (4* CV_PI) * (area/pow(perimeter,2)); //Compactness
	roundness = 4 * (area/(CV_PI*pow(majoraxislength,2)));

	//Calculate total circularity score
	circlescore = (aspectratio + circularity + roundness) / 3;

	//release and free memory
	cvReleaseImage(&image);
	cvReleaseImage(&imageGreen);
	cvReleaseImage(&imageHSV);
	blobsGreen.ClearBlobs();

	// return
	return circlescore;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



void *timersecs(void *ptr) {
    //Vars
	time_t start, end;
    time(&start);  /* start the timer */
	int oldtime = 0;
	
	// Initialize time
	sprintf(timetemp, "1 SEC", oldtime);
	timerrunning = TRUE;

    do {
        time(&end);
        timeelapsed = difftime(end, start);

		//Write time elapsed to string
		if (oldtime != timeelapsed) {
			sprintf(timetemp, "%d SECS", timeelapsed+1);
		}
		oldtime = timeelapsed;

    } while((recording == TRUE) && (timeelapsed <= max_gesture_wait_time));  /* run for ten seconds */

	timerrunning = FALSE;
	return NULL;
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



///// ADDRESS:
// allrobots - both gloves higher than the jacket centroid.. both blobs centroids are very close to each other (1 blob)
// somerobots - both gloves higher than the jacket upperbound.. both blobs centroids are very far from each other (2 blobs)
// you - else (last condition in statement if nothing is fulfilled) //if green glove higher than jacket lowerbound, and yellow glove is lower than jacket lowerbound (1 blob)

///// ACTION:
// takeoff - both gloves higher than the jacket upperbound.. both blobs centroids are very close to each other (1 blob)
// search - both gloves higher than the jacket upperbound.. both blobs centroids are very far from each other (2 blobs)
// gotobase - if green glove is higher than jacket lowerbound, and yellow glove is lower than jacket lowerbound (2 blobs)
// follow - distance between both gloves should be greater than 0 and less than a threshold and both gloves should be within upper and lower bounds of the jacket (2 blobs)
// landing - both gloves should be in between upper and lower jacket bounds (1 blob)
//--IF NOTHING, THEN.. ELSE.. the only left option is:
// gotoandwait - else (last condition in statement if nothing is fulfilled)

////// Classifiers to build //////
/// INDIVIDUAL (YOU) - finger pointing towards robot (classifier; to detect to which robot person is pointing)
// Finger counts 0 to 5 (6 classes)

void *gesture_sequence(void *ptr)
{
	// Dummy waiting, just to initialize face detector
	directionshown = TRUE;
	usleep(time_between_indrobots);
	directionshown = FALSE;


	//thread loop
	while (1) {
		threadstart = TRUE;
		stage1 = TRUE;
		recording = FALSE;
		timeelapsed = 0;

		// empty results from previous stage
		resultfirst = "";
		resultsecond = "";
		*resultthird = '\0';
		*resultfourth = '\0';

		//Initial waiting time before starting thread (set to 10 secs)
		gesturemode="PROGRAM STARTED";
		usleep(wait_time_before_start);

		// For the FIRST SET OF GESTURES: Addressing
		/////////////////////////////////////////////////////////////////////////////
		gesturemode="(1) ADDRESS: [<ALL>,<SOME>,<YOU>]";
		usleep(time_between_commands);
		recording = TRUE;


		//motion detector for capturing gesture command
		/////////////////////////////////////////////////////
		//create timer thread
		pthread_t thId1;
		pthread_create(&thId1, NULL, timersecs, NULL);
		//wait untill no motion is detected
		do {
			if ((recording == TRUE) && (motionscore <= motionthreshold)) {
				recording = FALSE;
				quitreason = FALSE;
				timeelapsed = 0;
				goto sequence2;
			}
			if ((recording == TRUE) && (timeelapsed == max_gesture_wait_time)) {
				recording == FALSE;
				quitreason == TRUE;
				timeelapsed = 0;
				goto program_end;
			}
			usleep(1);
		} while (recording == TRUE);


sequence2:
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// determine classified result
		if ((distancegloves == 0) && (centroidyellow.y < jacketubound.y) && (centroidgreen.y < jacketubound.y)) { //ALLROBOTS
			resultfirst="ALL ROBOTS: SELECTED";
		}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//If the result is "somerobots", the human can specify the "number of robots" to select randomly, or select robots located in a "specific area (direction)"
		//classify if the user is presenting a finger count or direction.
		else if ((distancegloves > 0) && (centroidyellow.y < jacketubound.y) && (centroidgreen.y < jacketubound.y)) { //SOMEROBOTS
			gesturemode="SOME ROBOTS: [<DIRECTION|NO. OF ROBOTS>]";
			usleep(time_between_commands);
			recording = TRUE;			

			//motion detector for capturing gesture command
			/////////////////////////////////////////////////////
			//create timer thread
			pthread_t thId21;
			pthread_create(&thId21, NULL, timersecs, NULL);
			//wait untill no motion is detected
			directionshown = TRUE;
			do {
				if ((recording == TRUE) && (motionscore <= motionthreshold)) {
					recording = FALSE;
					quitreason = FALSE;
					directionshown = FALSE;
					timeelapsed = 0;
					goto sequence21;
				}
				if ((recording == TRUE) && (timeelapsed == max_gesture_wait_time-1)) {
					recording == FALSE;
					quitreason == TRUE;
					directionshown = FALSE;
					timeelapsed = 0;
					goto program_end;
				}
				usleep(1);
			} while (recording == TRUE);
		
sequence21:
			//classify if a gesture is a direction or fingercount
			resultclassifier=computepredictionresults(2);
			
			if (resultclassifier == 1) { //DIRECTION
				char somestr1[35];
				sprintf(somestr1, "SOME ROBOTS: BETWEEN %.0f AND %.0f DEG", directionyellow, directiongreen);
				resultfirst=somestr1;
			
			} else if (resultclassifier == 2) { //FINGERCOUNTS
				//if gesture is a fingercount, classify the fingercount
				resultfingercounts=computepredictionresults(1);
				//determine classified result
				char somestr2[35];
				sprintf(somestr2, "SOME: SELECTED %d ROBOT(S)", resultfingercounts);
				resultfirst=somestr2;
			}
		}
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


		//If the result is "you", the human can point towards one or more robots one by one
		//rule 1: if the glove is lower than the jacket then.. select one robot
		//--> no need to go inside the while loop below
		//rule 2: if the glove is higher than the jacket then.. select more than one robot, one by one..
		//--> repeat the while loop N times below; break outside the while loop when the hand goes lower than the jacket; use delay inside the loop
		//
		else if ((distancegloves > 0) && (centroidgreen.y < centroidyellow.y) && (centroidgreen.y < jacketlbound.y) && (centroidyellow.y > jacketlbound.y)) { //(YOU): INDIVIDUAL

			//Score in between [0,1] for the pointing gesture.
			//The robot with the highest score in the swarm, wins and gets selected.
			//Use only the GREEN colored glove to compute the score
			recording = TRUE;	//no actual recording just dummy to display result on screen
			double circlescore = computecirclescore();	

			//First robot selected
			individualctr = 1;
			gesturemode="1 ROBOT SELECTED";
			usleep(time_between_indrobots);
			recording = FALSE;	//no actual recording just dummy to display result on screen


			//continue loop until the hand goes lower than the jacket
			while(1)
			{
				recording = TRUE;
				gesturemode="**CHECKING & SELECTING** ROBOTS...";

				//motion detector for capturing gesture command
				/////////////////////////////////////////////////////
				//create timer thread
				pthread_t thId22;
				pthread_create(&thId22, NULL, timersecs, NULL);
				//wait untill no motion is detected
				do {
					if ((recording == TRUE) && (motionscore <= motionthreshold)) {
						recording = FALSE;
						quitreason = FALSE;
						timeelapsed = 0;
						goto sequence22;
					}
					if ((recording == TRUE) && (timeelapsed == max_gesture_wait_time-1)) {
						recording == FALSE;
						quitreason == TRUE;
						timeelapsed = 0;
						goto program_end;
					}
					usleep(1);
				} while (recording == TRUE);


sequence22:
				//Score in between [0,1] for the pointing gesture.
				//The robot with the highest score in the swarm, wins and gets selected.
				//Use only the GREEN colored glove to compute the score
				circlescore = computecirclescore();

				//Wait and check if the glove is higher or has gone down
				//usleep(1000);

				//break out of the while loop if the hand goes lower than the jacket.
				if ((centroidgreen.y > jacketlbound.y) && (centroidyellow.y > jacketlbound.y)) {
					gesturemode="**SELECTION FINISHED**";
					usleep(time_between_indrobots);
					break;
				} else {
					individualctr++;
					char individualstr[35];
					sprintf(individualstr, "%d INDIVIDUAL ROBOT(S) SELECTED", individualctr);
					gesturemode=individualstr;
					usleep(time_between_indrobots);
				}
			}
			// determine classified result
			// determine number of robots selected with the "you" pointing gesture
			char individualstr2[35];
			sprintf(individualstr2, "SELECTED %d INDIVIDUAL ROBOT(S)", individualctr);
			resultfirst=individualstr2;
			individualctr = 0;


		// ELSE no input detected
		} else {
			resultfirst="NO ROBOT(S) SELECTED";
			goto program_end;
		}
		stage1 = FALSE;
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		

		//// For the SECOND SET OF GESTURES: Action
		///////////////////////////////////////////////////////////////////////////////
		stage2 = TRUE;
		gesturemode="(2) ACTION: [<GOWAI|LND|GOBAS|SEAR|FOL|TKOFF>]";
		usleep(time_between_commands);

		//start capturing images for action
		recording = TRUE;
		gesturemode="(2) **RECORDING** ACTION";

		//motion detector for capturing gesture command
		/////////////////////////////////////////////////////
		//create timer thread
		pthread_t thId3;
		pthread_create(&thId3, NULL, timersecs, NULL);
		//wait untill no motion is detected
		do {
			if ((recording == TRUE) && (motionscore <= motionthreshold)) {
				recording = FALSE;
				quitreason = FALSE;
				timeelapsed = 0;
				goto sequence3;
			}
			if ((recording == TRUE) && (timeelapsed == max_gesture_wait_time-1)) {
				recording == FALSE;
				quitreason == TRUE;
				timeelapsed = 0;
				goto program_end;
			}
			usleep(1);
		} while (recording == TRUE);


sequence3:
		// determine classified result
		if ((distancejacket == 0) && (distancegloves == 0)) {  //&& (centroidgreen.y < faceheight) && (centroidyellow.y < faceheight)) {
			resultsecond="TAKEOFF";		// no subaction required


		} else if ((distancejacket == 1) && (distancegloves == 0)) {  //&& (centroidgreen.y > faceheight) && (centroidyellow.y > faceheight)) { 
			resultsecond="LAND";		// no subaction required


		} else if ((distancegloves > 0) && (centroidgreen.y < jacketubound.y) && (centroidyellow.y < jacketubound.y)) {
			resultsecond="SEARCH";
			///////////////////////////////////////////////////////////////////////////////
			gesturemode="(3) DIRECTION: [<HAND DIRECTION>]";
			usleep(time_between_commands);
			recording = TRUE;

			//motion detector for capturing gesture command
			/////////////////////////////////////////////////////
			//create timer thread
			pthread_t thId31;
			pthread_create(&thId31, NULL, timersecs, NULL);
			//wait untill no motion is detected
			directionshown = TRUE;
			do {
				if ((recording == TRUE) && (motionscore <= motionthreshold)) {
					recording = FALSE;
					quitreason = FALSE;
					directionshown = FALSE;
					timeelapsed = 0;
					goto sequence31;
				}
				if ((recording == TRUE) && (timeelapsed == max_gesture_wait_time-1)) {
					recording == FALSE;
					quitreason == TRUE;
					directionshown = FALSE;
					timeelapsed = 0;
					goto program_end;
				}
				usleep(1);
			} while (recording == TRUE);

sequence31:
			// get direction of hand in 360 degree plain
			sprintf(resultthird, "AT %.0f DEGREES", directionyellow);
			////////////////////////////////////////////////////////

			//get distance gesture (fingercounts)
			gesturemode="(4) DURATION: [<#MINUTES>]";
			usleep(time_between_commands);
			recording = TRUE;

			//motion detector for capturing gesture command
			////////////////////////////////////////////////////////
			//create timer thread
			pthread_t thId311;
			pthread_create(&thId311, NULL, timersecs, NULL);
			//wait untill no motion is detected
			do {
				if ((recording == TRUE) && (motionscore <= motionthreshold)) {
					recording = FALSE;
					quitreason = FALSE;
					timeelapsed = 0;
					goto sequence311;
				}
				if ((recording == TRUE) && (timeelapsed == max_gesture_wait_time-1)) {
					recording == FALSE;
					quitreason == TRUE;
					timeelapsed = 0;
					goto program_end;
				}
				usleep(1);
			} while (recording == TRUE);

sequence311:
			//classify distance in "metres" (based on finger counts)
			resultclassifier=computepredictionresults(1);
			// determine classified result
			sprintf(resultfourth, "FOR %d MINUTES", resultclassifier);


		} else if ((distancegloves > 0) && (centroidgreen.y < jacketlbound.y) && (centroidyellow.y > jacketlbound.y)) {
			resultsecond="GOTO BASE";	// no subaction required


		} else if ((distancegloves > 0) && (centroidgreen.y > jacketubound.y) && (centroidyellow.y > jacketubound.y) && (centroidgreen.y < jacketlbound.y) && (centroidyellow.y < jacketlbound.y)) {
			resultsecond="FOLLOW";
			///////////////////////////////////////////////////////////////////////////////
			gesturemode="(3) DIRECTION: [<PERSON DIRECTION>]";
			usleep(time_between_commands);		
			recording = TRUE;

			//motion detector for capturing gesture command
			/////////////////////////////////////////////////////
			//create timer thread
			pthread_t thId32;
			pthread_create(&thId32, NULL, timersecs, NULL);
			//wait untill no motion is detected
			directionshown = TRUE;
			do {
				if ((recording == TRUE) && (motionscore <= motionthreshold)) {
					recording = FALSE;
					quitreason = FALSE;
					directionshown = FALSE;
					timeelapsed = 0;
					goto sequence32;
				}
				if ((recording == TRUE) && (timeelapsed == max_gesture_wait_time-1)) {
					recording == FALSE;
					quitreason == TRUE;
					directionshown = FALSE;
					timeelapsed = 0;
					goto program_end;
				}
				usleep(1);
			} while (recording == TRUE);

sequence32:
			// get direction of hand in 360 degree plain
			sprintf(resultthird, "PERSON AT %.0f DEGREES", directionyellow);
			////////////////////////////////////////////////////////



		} else if ((distancegloves > 0) && (centroidgreen.y < jacketubound.y) && (centroidyellow.y > jacketubound.y) && (centroidyellow.y < jacketlbound.y)) {
			resultsecond="GOTO AND WAIT";
			///////////////////////////////////////////////////////////////////////////////
			gesturemode="(3) DIRECTION: [<HAND DIRECTION>]";
			usleep(time_between_commands);
			recording = TRUE;

			//motion detector for capturing gesture command
			/////////////////////////////////////////////////////
			//create timer thread
			pthread_t thId33;
			pthread_create(&thId33, NULL, timersecs, NULL);
			//wait untill no motion is detected
			directionshown = TRUE;
			do {
				if ((recording == TRUE) && (motionscore <= motionthreshold)) {
					recording = FALSE;
					quitreason = FALSE;
					directionshown = FALSE;
					timeelapsed = 0;
					goto sequence33;
				}
				if ((recording == TRUE) && (timeelapsed == max_gesture_wait_time-1)) {
					recording == FALSE;
					quitreason == TRUE;
					directionshown = FALSE;
					timeelapsed = 0;
					goto program_end;
				}
				usleep(1);
			} while (recording == TRUE);

sequence33:
			// get direction of hand in 360 degree plain
			sprintf(resultthird, "IN THE DIRECTION OF: %.0f DEGREES", directionyellow);


			//get distance gesture (fingercounts)
			gesturemode="(4) DISTANCE: [<#METRES>]";
			usleep(time_between_commands);
			recording = TRUE;

			//motion detector for capturing gesture command
			////////////////////////////////////////////////////////
			//create timer thread
			pthread_t thId331;
			pthread_create(&thId331, NULL, timersecs, NULL);
			//wait untill no motion is detected
			do {
				if ((recording == TRUE) && (motionscore <= motionthreshold)) {
					recording = FALSE;
					quitreason = FALSE;
					timeelapsed = 0;
					goto sequence331;
				}
				if ((recording == TRUE) && (timeelapsed == max_gesture_wait_time-1)) {
					recording == FALSE;
					quitreason == TRUE;
					timeelapsed = 0;
					goto program_end;
				}
				usleep(1);
			} while (recording == TRUE);

sequence331:
			//classify distance in "metres" (based on finger counts)
			resultclassifier=computepredictionresults(1);
			// determine classified result
			sprintf(resultfourth, "AT %d METRES", resultclassifier);


		// ELSE no input detected
		} else {
			resultsecond="NO ACTION GIVEN";
			goto program_end;
		}
		stage2 = FALSE;
		///////////////////////////////////////////////////////////////////////////////

program_end:
		stage1 = FALSE; stage2 = FALSE;
		//// repeat this entire routine after each round ////
		if (quitreason==TRUE) {
			gesturemode = "NO GESTURE WAS CAPTURED";
			resultfirst = "NO GESTURE CAPTURED";
		} else if (quitreason==FALSE) {
			gesturemode = "PROGRAM ENDED";
		}

		//Wait before restarting program
		recording = FALSE;
		usleep(wait_time_before_start/2);
		threadstart = FALSE;
		usleep(wait_time_before_start/2);
	} // end of while loop

	return NULL;
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



// Perform face detection on the input image, using the given Haar Cascade.
// Returns a rectangle for the detected region in the given image.
CvRect detectFaceInImage(IplImage *inputImg, CvHaarClassifierCascade* cascade, int key)
{
	// Only search for 1 face
	int flags = CV_HAAR_DO_CANNY_PRUNING | CV_HAAR_DO_ROUGH_SEARCH; //| CV_HAAR_FIND_BIGGEST_OBJECT; // | CV_HAAR_DO_ROUGH_SEARCH; CV_HAAR_DO_CANNY_PRUNING
	// How detailed should the search be.
	float search_scale_factor = 1.1f;
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
	cvSmooth(detectImg, detectImg, CV_GAUSSIAN, 11, 11);

	//// Detect all possible face matches in the greyscale image ////
	facerect = cvHaarDetectObjects(detectImg, cascade, storage, search_scale_factor, 0, flags, minFeatureSize);
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
			cvRectangle(displayImage, pt1face, pt2face, CV_RGB(255,0,0), 1, 4, 0); //RED (front pose)
		} else if ((key == 3) || (key == 4)) { // if side pose
			cvRectangle(displayImage, pt1face, pt2face, CV_RGB(0,0,255), 1, 4, 0); //BLUE (side pose)
		}
	}
**/

	if (greyImg) {
		cvReleaseImage(&greyImg);
	}
	cvReleaseMemStorage(&storage);
	
	return rc;	// Return the biggest face found, or (-1,-1,-1,-1).
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void Init() {
	// Capture image
	capture = cvCaptureFromCAM(0);

	// Set variables
	recording = FALSE;
	stage1 = TRUE;
	stage2 = FALSE;

	// Set camera properties
	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, framewidth);
	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, frameheight);
	printf("Frame Width: %d\n", framewidth);
	printf("Frame Height: %d\n", frameheight);

	// Initialize before going into the main thread loop
	cvNamedWindow("User Interface", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Orange Color", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Green Color", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Yellow Color", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Camera Parameters", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Calibration: Thresholded Image", CV_WINDOW_AUTOSIZE);
	cvInitFont(&font,CV_FONT_HERSHEY_PLAIN, 3,3,0,2,CV_AA);
	cvInitFont(&font2,CV_FONT_HERSHEY_PLAIN, 10,10,0,7,CV_AA);

	// Run secondary thread for processing data
	if (firstrun==FALSE) {
		pthread_t thId;
		pthread_create(&thId, NULL, gesture_sequence, NULL);
		firstrun=TRUE;
	}

	// Load the HaarCascade classifier for face detection
	frontfaceclassifier = (CvHaarClassifierCascade*)cvLoad(frontclassifier, 0, 0, 0);
	sidefaceclassifier = (CvHaarClassifierCascade*)cvLoad(sideclassifier, 0, 0, 0);

	//Initialize empty motion data array
	for (int i=0; i<NFRAMES; i++) {
		for (int j=0; j<NDATA; j++) {
			previousmotion[i][j] = 0;
			currentmotion[i][j] = 0;
		}
	}


	// Create trackbars for calibration
	//////////////////////////////////////////////////////////////////////
	cvCreateTrackbar("Hmin (0)","Orange Color",&Hmin_O,180,0);
	cvCreateTrackbar("Hmax (20)","Orange Color",&Hmax_O,180,0);
	cvCreateTrackbar("Smin (0)","Orange Color",&Smin_O,255,0);
	cvCreateTrackbar("Smax (255)","Orange Color",&Smax_O,255,0);
	cvCreateTrackbar("Vmin (0)","Orange Color",&Vmin_O,255,0);
	cvCreateTrackbar("Vmax (255)","Orange Color",&Vmax_O,255,0);
	cvCreateTrackbar("Hmin (35)","Green Color",&Hmin_G,180,0);
	cvCreateTrackbar("Hmax (70)","Green Color",&Hmax_G,180,0);
	cvCreateTrackbar("Smin (0)","Green Color",&Smin_G,255,0);
	cvCreateTrackbar("Smax (255)","Green Color",&Smax_G,255,0);
	cvCreateTrackbar("Vmin (0)","Green Color",&Vmin_G,255,0);
	cvCreateTrackbar("Vmax (255)","Green Color",&Vmax_G,255,0);
	cvCreateTrackbar("Hmin (15)","Yellow Color",&Hmin_Y,180,0);
	cvCreateTrackbar("Hmax (35)","Yellow Color",&Hmax_Y,180,0);
	cvCreateTrackbar("Smin (0)","Yellow Color",&Smin_Y,255,0);
	cvCreateTrackbar("Smax (255)","Yellow Color",&Smax_Y,255,0);
	cvCreateTrackbar("Vmin (0)","Yellow Color",&Vmin_Y,255,0);
	cvCreateTrackbar("Vmax (255)","Yellow Color",&Vmax_Y,255,0);
//	cvCreateTrackbar("FPS (2-30)","Camera Parameters",&fps,30,0);
//	cvCreateTrackbar("Bri (0-255)","Camera Parameters",&brightness,255,0);
//	cvCreateTrackbar("Con (0-255)","Camera Parameters",&contrast,255,0);
//	cvCreateTrackbar("Sat (0-255)","Camera Parameters",&saturation,255,0);
	
	// Use default values from camera
//	brightness = cvGetCaptureProperty(capture, CV_CAP_PROP_BRIGHTNESS);
//	contrast = cvGetCaptureProperty(capture, CV_CAP_PROP_CONTRAST);
//	saturation = cvGetCaptureProperty(capture, CV_CAP_PROP_SATURATION);
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



void DoProcessing() {
	// Initialize variables to run
	CvPoint finalpt1, finalpt2, facecentroid;
	double firstsizeyg, secondsizeyg, firstsizeo, secondsizeo, d1, d2, d3, d4, d5, d6, d7, d8, d9;
	int straight, right, left, frontfacescore, frontfaceflipscore, sidefacescore, sidefaceflipscore;


	// Adjust camera properties if values are changed by user
//	cvSetCaptureProperty(capture, CV_CAP_PROP_FPS, fps-30);
//	cvSetCaptureProperty(capture, CV_CAP_PROP_BRIGHTNESS, brightness);
//	cvSetCaptureProperty(capture, CV_CAP_PROP_CONTRAST, contrast);
//	cvSetCaptureProperty(capture, CV_CAP_PROP_SATURATION, saturation);

	// Make a duplicate copy of last frame to preview on screen
	IplImage *displayImage = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3);
	cvCopy(frame,displayImage,NULL);
	// Create a flipped copy of the image (flip, left and right)
	IplImage *displayImageflip = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3);
	cvFlip(displayImage,displayImageflip,1);


	//superimpose gesture sequence instructions on displayed frames
	if (recording==FALSE) {
		cvCircle(displayImage,cvPoint(28,28),18,CV_RGB(0,255,0),-1);
	} else if (recording==TRUE) {
		cvCircle(displayImage,cvPoint(28,28),18,CV_RGB(255,0,0),-1);
		if (timerrunning==TRUE) {
			cvPutText(displayImage,timetemp,cvPoint(1030,680), &font, CV_RGB(255,0,0));
		}
	}
	// Display current gesture vocabulary
	cvPutText(displayImage,gesturemode,cvPoint(55,45), &font, CV_RGB(255,0,0));

		
	//superimpose results
	if (threadstart==TRUE) {
		cvPutText(displayImage,"GESTURE SEQUENCE:",cvPoint(10,230), &font, CV_RGB(0,255,0));

		if (resultfirst) {
			cvPutText(displayImage,resultfirst,cvPoint(10,270), &font, CV_RGB(255,90,0));

			if (resultsecond) {
				cvPutText(displayImage,resultsecond,cvPoint(10,310), &font, CV_RGB(255,90,0));

				if (resultthird) {
					cvPutText(displayImage,resultthird,cvPoint(10,350), &font, CV_RGB(255,90,0));

					if (resultfourth) {
						cvPutText(displayImage,resultfourth,cvPoint(10,390), &font, CV_RGB(255,90,0));
					}
				}
			}
		}
	}


	/////// Face Detection /////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Perform face detection on the input image, using the given Haar classifier
	if (directionshown == TRUE) {
		/// Front faces ///
		CvRect facefront = detectFaceInImage(displayImage, frontfaceclassifier, 1);
		frontfacescore = facescore;
		CvRect facefrontflip = detectFaceInImage(displayImageflip, frontfaceclassifier, 2);
		frontfaceflipscore = facescore;

		/// Side faces ///
		CvRect faceside = detectFaceInImage(displayImage, sidefaceclassifier, 3);
		sidefacescore = facescore;
		CvRect facesideflip = detectFaceInImage(displayImageflip, sidefaceclassifier, 4);
		sidefaceflipscore = facescore;

		// Compute the three face positions //
		straight = frontfacescore + frontfaceflipscore;
		right = sidefacescore;
		left = sidefaceflipscore;
	
		// DRAW FACE BOUNDING BOX
		finalpt1.x = 0; finalpt1.y = 0; finalpt2.x = 0; finalpt2.y = 0;

		// If only front pose of face is detected
		if ((frontfacescore + frontfaceflipscore) > (sidefacescore + sidefaceflipscore)) {
			finalpt1.x = (facefront.x + facefrontflip.x) / 2;
			finalpt1.y = (facefront.y + facefrontflip.y) / 2;
			finalpt2.x = (facefront.width + facefrontflip.width) / 2;
			finalpt2.y = (facefront.height + facefrontflip.height) / 2;
			if (directionshown == TRUE) {
				cvRectangle(displayImage, finalpt1, finalpt2, CV_RGB(255,0,0), 3, 4, 0);
			}

		// If only side pose of face is detected
		} else if ((sidefacescore + sidefaceflipscore) > (frontfacescore + frontfaceflipscore)) {
			finalpt1.x = (faceside.x + facesideflip.x) / 2;
			finalpt1.y = (faceside.y + facesideflip.y) / 2;
			finalpt2.x = (faceside.width + facesideflip.width) / 2;
			finalpt2.y = (faceside.height + facesideflip.height) / 2;
			if (directionshown == TRUE) {
				cvRectangle(displayImage, finalpt1, finalpt2, CV_RGB(0,0,255), 3, 4, 0);
			}
		}
		//faceheight = finalpt2.y; //lower bound of face bounding box (y-coordinate)

		//Compute centroid of face
		////////////////////////////////
		facecentroid.x = (finalpt1.x + finalpt2.x) / 2;
		facecentroid.y = (finalpt1.y + finalpt2.y) / 2;

		// Draw line from CENTER of FACE down to BODY
		bodyline.x = facecentroid.x;
		bodyline.y = frameheight;
		if (directionshown == TRUE) {
			cvLine(displayImage, facecentroid, bodyline, CV_RGB(0,255,255), 1, 4, 0);
		}

	
		// Compute angle of the YELLOW glove
		////////////////////////////////////////
		directionyellow = atan2(centroidyellow.y - facecentroid.y, centroidyellow.x - facecentroid.x) * 180.0 / CV_PI;
		if (directionyellow > 360) {
			directionyellow = directionyellow - 360;
		} else if (directionyellow < 0) {
			directionyellow = directionyellow + 360;
		}
		// Compute angle of the GREEN glove
		directiongreen = atan2(centroidgreen.y - facecentroid.y, centroidgreen.x - facecentroid.x) * 180.0 / CV_PI;
		if (directiongreen > 360) {
			directiongreen = directiongreen - 360;
		} else if (directiongreen < 0) {
			directiongreen = directiongreen + 360;
		}

		char tempangle[100];
		sprintf(tempangle, "%.0f,%.0f (YEL,GRN) ", directionyellow, directiongreen);
		cvPutText(displayImage,"GESTURE DIRECTION:",cvPoint(10,480), &font, CV_RGB(255,0,0));
		cvPutText(displayImage,tempangle,cvPoint(580,480), &font, CV_RGB(255,90,0));


		// Superimpose facescore on image
		char tempstraight[15];
		char tempright[15];
		char templeft[15];
		sprintf(tempstraight, "%d", straight);
		sprintf(tempright, "%d", right);
		sprintf(templeft, "%d", left);
		//Print facescores
		cvPutText(displayImage,"FACESCORE (CENTER):",cvPoint(10,560), &font, CV_RGB(255,0,255));
		cvPutText(displayImage,tempstraight,cvPoint(580,560), &font, CV_RGB(0,255,0));
		cvPutText(displayImage,"FACESCORE (LEFT):",cvPoint(10,600), &font, CV_RGB(255,0,255));
		cvPutText(displayImage,templeft,cvPoint(580,600), &font, CV_RGB(0,255,0));
		cvPutText(displayImage,"FACESCORE (RIGHT):",cvPoint(10,640), &font, CV_RGB(255,0,255));
		cvPutText(displayImage,tempright,cvPoint(580,640), &font, CV_RGB(0,255,0));
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


		//////// Orientation of Face /////////
		if ((straight <= 15) && (right <= 5) && (left <= 5)) {
			facedirection = "NO HUMAN PRESENT";
		} else if ((straight > 50) && (right > 0) && (left > 0)) {
			facedirection = "IN THE CENTER";
		} else if ((straight < 50) && (left <= 15)) {
			facedirection = "RIGHT ORIENTED";
		} else if ((straight < 50) && (right <= 15)) {
			facedirection = "LEFT ORIENTED";
		}
		cvPutText(displayImage,"FACE DIRECTION:",cvPoint(10,520), &font, CV_RGB(255,0,0));
		cvPutText(displayImage,facedirection,cvPoint(580,520), &font, CV_RGB(255,90,0));
	}
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


	// THRESH HOLDING //
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//// Segment the current image ////
	CvRect rectYellow, rectGreen, rectOrange, rectOrange1, rectOrange2, rectYellowGreen;
	Point ptYellow1, ptYellow2, ptGreen1, ptGreen2, ptOrange11, ptOrange12, ptOrange21, ptOrange22, ptYellowGreen1, ptYellowGreen2, ptOrange1, ptOrange2;
	IplImage *imageHSV					= cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3);
	IplImage *imageYellow				= cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 1);   // Grayscale output image
	IplImage *imageGreen				= cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 1);   // Grayscale output image
	IplImage *imageOrange				= cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 1);   // Grayscale output image
	IplImage *imageYellowGreen			= cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 1);   // Grayscale output image
	IplImage *imageYellowGreenOrange	= cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 1);   // Grayscale output image		

	// Convert RGB to HSV format //
	cvCvtColor(frame, imageHSV, CV_BGR2HSV);
	// Segement YELLOW //
	cvInRangeS(imageHSV, cvScalar(Hmin_Y, Smin_Y, Vmin_Y), cvScalar(Hmax_Y, Smax_Y, Vmax_Y), imageYellow); // yellow threshold
	// Segement GREEN //
	cvInRangeS(imageHSV, cvScalar(Hmin_G, Smin_G, Vmin_G), cvScalar(Hmax_G, Smax_G, Vmax_G), imageGreen); // green threshold
	// Segement ORANGE //
	cvInRangeS(imageHSV, cvScalar(Hmin_O, Smin_O, Vmin_O), cvScalar(Hmax_O, Smax_O, Vmax_O), imageOrange); // orange threshold

	// Combine both gloves //
	cvOr(imageYellow,imageGreen,imageYellowGreen); // Combine yellow and green blob
	// Combine both gloves and jacket //
	cvOr(imageYellowGreen,imageOrange,imageYellowGreenOrange); // Combine yellow and green blob


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
	//Co-ordinates of bounding box joining both blobs of orange jacket
	ptOrange1.x = biggestOrangeBlob.MinX();
	ptOrange1.y = biggestOrangeBlob.MinY();
	ptOrange2.x = biggestOrangeBlob.MaxX();
	ptOrange2.y = biggestOrangeBlob.MaxY();


	// 2 ORANGE blobs //
	CBlobResult blobsOrangeSplit;
	CBlob LargestOrangeBlob, SecondLargestOrangeBlob;
	blobsOrangeSplit = CBlobResult(imageOrange, NULL, 0);
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Get the largest blob
	blobsOrangeSplit.GetNthBlob(CBlobGetArea(), 0, LargestOrangeBlob);
	firstsizeo = LargestOrangeBlob.Area();

	// Get the second largest blob (Filter the largest blob out)
	blobsOrangeSplit.Filter(blobsOrangeSplit, B_EXCLUDE, CBlobGetArea(), B_GREATER_OR_EQUAL, firstsizeo);
	blobsOrangeSplit.GetNthBlob(CBlobGetArea(), 0, SecondLargestOrangeBlob);
	secondsizeo = SecondLargestOrangeBlob.Area();
			
	// Get bounding box around blob
	rectOrange1 = LargestOrangeBlob.GetBoundingBox();
	rectOrange2 = SecondLargestOrangeBlob.GetBoundingBox();
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////


	// 1 YELLOW+GREEN blob //
	CBlobResult blobsYellowGreen;
	CBlob LargestYellowGreenBlob, SecondLargestYellowGreenBlob;
	blobsYellowGreen = CBlobResult(imageYellowGreen, NULL, 0);
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Get the largest blob
	blobsYellowGreen.GetNthBlob(CBlobGetArea(), 0, LargestYellowGreenBlob);
	firstsizeyg = LargestYellowGreenBlob.Area();

	// Get the second largest blob (Filter the largest blob out)
	blobsYellowGreen.Filter(blobsYellowGreen, B_EXCLUDE, CBlobGetArea(), B_GREATER_OR_EQUAL, firstsizeyg);
	blobsYellowGreen.GetNthBlob(CBlobGetArea(), 0, SecondLargestYellowGreenBlob);
	secondsizeyg = SecondLargestYellowGreenBlob.Area();
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////



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


	// Copy all centroids into memory
	currentmotion[0][0]  = centroidyellow.x;
	currentmotion[0][1]  = centroidgreen.x;
	currentmotion[0][2]  = centroidjacket.x;
	currentmotion[0][3]  = centroidyellow.y;
	currentmotion[0][4]  = centroidgreen.y;
	currentmotion[0][5]  = centroidjacket.y;
	currentmotion[0][6]  = jacketubound.x;
	currentmotion[0][7]  = jacketubound.y;
	currentmotion[0][8]  = jacketlbound.x;
	currentmotion[0][9]	 = jacketlbound.y;
	currentmotion[0][10] = jacketleft.x;
	currentmotion[0][11] = jacketleft.y;
	currentmotion[0][12] = jacketright.x;
	currentmotion[0][13] = jacketright.y;


	// Compute bounding boxes //
	// Calculate the bounding rect around the largest blobs
	rectGreen = biggestGreenBlob.GetBoundingBox();
	rectYellow = biggestYellowBlob.GetBoundingBox();
	rectOrange = biggestOrangeBlob.GetBoundingBox();
	rectYellowGreen = LargestYellowGreenBlob.GetBoundingBox();

	//Calculate coordinates of bounding box
	ptGreen1.x = rectGreen.x;
	ptGreen1.y = rectGreen.y;
	ptGreen2.x = rectGreen.x + rectGreen.width;
	ptGreen2.y = rectGreen.y + rectGreen.height;
	ptYellow1.x = rectYellow.x;
	ptYellow1.y = rectYellow.y;
	ptYellow2.x = rectYellow.x + rectYellow.width;
	ptYellow2.y = rectYellow.y + rectYellow.height;
	//
	ptOrange11.x = rectOrange1.x;
	ptOrange11.y = rectOrange1.y;
	ptOrange12.x = rectOrange1.x + rectOrange1.width;
	ptOrange12.y = rectOrange1.y + rectOrange1.height;
	ptOrange21.x = rectOrange2.x;
	ptOrange21.y = rectOrange2.y;
	ptOrange22.x = rectOrange2.x + rectOrange2.width;
	ptOrange22.y = rectOrange2.y + rectOrange2.height;
	//
	ptYellowGreen1.x = rectYellowGreen.x;
	ptYellowGreen1.y = rectYellowGreen.y;
	ptYellowGreen2.x = rectYellowGreen.x + rectYellowGreen.width;
	ptYellowGreen2.y = rectYellowGreen.y + rectYellowGreen.height;


	// Draw the bounding boxes
	// Draw the rectangle over both the gloves if both are combined, else draw two separate boxes for each glove
	if (secondsizeyg < glovesizeratio*firstsizeyg) {
		cvRectangle(displayImage, ptYellowGreen1, ptYellowGreen2, CV_RGB(255,0,255), 1, 4, 0); // 1 Blob
		distancegloves = 0;
	} else {
		cvRectangle(displayImage, ptGreen1, ptGreen2, CV_RGB(0,255,0), 1, 4, 0);  //2 Blobs
		cvRectangle(displayImage, ptYellow1, ptYellow2, CV_RGB(255,255,0), 1, 4, 0);
		distancegloves = 1;
	}
	// Draw the rectangle over jacket, if jacket is split in two pieces (color: light blue), if jacket is single piece (color: orange)
	if (secondsizeo > jacketsizeratio*firstsizeo) {  // 2 Blobs
		cvRectangle(displayImage, ptOrange11, ptOrange12, CV_RGB(0,255,255), 1, 4, 0);
		cvRectangle(displayImage, ptOrange21, ptOrange22, CV_RGB(0,255,255), 1, 4, 0);
		distancejacket = 1;
	} else {
		cvRectangle(displayImage, ptOrange1, ptOrange2, CV_RGB(255,127,0), 1, 4, 0);  //1 Blob
		distancejacket = 0;
	}
	/////////////////////////////////////////////////////////////////////////////////////


	// Print the display score ONLY IF SELECTING INDIVIDUAL ROBOT
	/////////////////////////////////////////////////////////////////
	if ((individualctr > 0) || (directionshown == TRUE)) {
		char tempind[15];
		sprintf(tempind, "%0.2f", circlescore);
		//Show circlescore only in process of selecting an individual robot
		cvPutText(displayImage,"SELECTION SCORE:",cvPoint(10,680), &font, CV_RGB(255,90,0));
		cvPutText(displayImage,tempind,cvPoint(450,680), &font, CV_RGB(0,255,0));
	}


	// Print the current gesture detected by the system
	///////////////////////////////////////////////////////////////////////
	//green, yellow and orange blobs bigger than a size, and biggest orange blob bigger than yellow and green --> only then a gesture is detected, else no.
	if ((biggestYellowBlob.Area() >= minyellowglovearea) && (biggestGreenBlob.Area() >= mingreenglovearea) && (biggestOrangeBlob.Area() > biggestYellowBlob.Area()) && (biggestOrangeBlob.Area() > biggestGreenBlob.Area())) {
		if (stage1 == TRUE) {
			// ADDRESSING (SELECTING)
			if ((distancegloves == 0) && (centroidyellow.y < jacketubound.y) && (centroidgreen.y < jacketubound.y)) {
				currentresult="ALL ROBOTS";
			} else if ((distancegloves > 0) && (centroidyellow.y < jacketubound.y) && (centroidgreen.y < jacketubound.y)) {
				currentresult="SOME ROBOTS";
			} else if ((distancegloves > 0) && (centroidgreen.y < centroidyellow.y) && (centroidgreen.y < jacketlbound.y) && (centroidyellow.y > jacketlbound.y)) {
				currentresult="INDIVIDUAL ROBOTS";
			} else {
				currentresult="NO ROBOTS SELECTED";
			}
		
		// ACTION TO PERFORM
		} else if (stage2 = TRUE) {
			if ((distancejacket == 0) && (distancegloves == 0)) {
				currentresult="TAKEOFF";
			} else if ((distancejacket == 1) && (distancegloves == 0)) {
				currentresult="LAND";			
			} else if ((distancegloves > 0) && (centroidgreen.y < jacketubound.y) && (centroidyellow.y < jacketubound.y)) {
				currentresult="SEARCH";
			} else if ((distancegloves > 0) && (centroidgreen.y < jacketlbound.y) && (centroidyellow.y > jacketlbound.y)) {
				currentresult="GOTO BASE";
			} else if ((distancegloves > 0) && (centroidgreen.y > jacketubound.y) && (centroidyellow.y > jacketubound.y) && (centroidgreen.y < jacketlbound.y) && (centroidyellow.y < jacketlbound.y)) {
				currentresult="FOLLOW";
			} else if ((distancegloves > 0) && (centroidgreen.y < jacketubound.y) && (centroidyellow.y > jacketubound.y) && (centroidyellow.y < jacketlbound.y)) {
				currentresult="GOTO AND WAIT";
			} else {
				currentresult="NO ACTION GIVEN";
			}
		}
	} else {
		currentresult="NO GESTURE DETECTED";
	}
	//Show circlescore only in process of selecting an individual robot
	if (threadstart == TRUE) {
		cvPutText(displayImage,"CURRENT GESTURE:",cvPoint(10,95), &font, CV_RGB(0,255,0));
		cvPutText(displayImage,currentresult,cvPoint(500,95), &font, CV_RGB(255,0,255));
	}


	//// MOTION DETECTOR ////
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//// Compute the distances between the centroids to find the motion of the gloves and jacket ////
	// Go through the memory
	CvPoint centroidyellow2, centroidgreen2, centroidjacket2, jacketubound2, jacketlbound2, jacketright2, jacketleft2;
	centroidyellow2.x = previousmotion[0][0];
	centroidgreen2.x  = previousmotion[0][1];
	centroidjacket2.x = previousmotion[0][2];
	centroidyellow2.y = previousmotion[0][3];
	centroidgreen2.y  = previousmotion[0][4];
	centroidjacket2.y = previousmotion[0][5];
	jacketubound2.x   = previousmotion[0][6];
	jacketubound2.y   = previousmotion[0][7];
	jacketlbound2.x   = previousmotion[0][8];
	jacketlbound2.y   = previousmotion[0][9];
	jacketleft2.x     = previousmotion[0][10];
	jacketleft2.y     = previousmotion[0][11];
	jacketright2.x    = previousmotion[0][12];
	jacketright2.y    = previousmotion[0][13];

	// Compute distances between yellow, green and orange blobs
	d1 = abs(sqrt(abs((centroidgreen.x - jacketubound.x)^2 + (centroidgreen.y - jacketubound.y)^2)) - sqrt(abs((centroidgreen2.x - jacketubound2.x)^2 + (centroidgreen2.y - jacketubound2.y)^2)));
	d2 = abs(sqrt(abs((centroidgreen.x - centroidjacket.x)^2 + (centroidgreen.y - centroidjacket.y)^2)) - sqrt(abs((centroidgreen2.x - centroidjacket2.x)^2 + (centroidgreen2.y - centroidjacket2.y)^2)));
	d3 = abs(sqrt(abs((centroidgreen.x - jacketlbound.x)^2 + (centroidgreen.y - jacketlbound.y)^2)) - sqrt(abs((centroidgreen2.x - jacketlbound2.x)^2 + (centroidgreen2.y - jacketlbound2.y)^2)));
	d4 = abs(sqrt(abs((centroidyellow.x - jacketubound.x)^2 + (centroidyellow.y - jacketubound.y)^2)) - sqrt(abs((centroidyellow2.x - jacketubound2.x)^2 + (centroidyellow2.y - jacketubound2.y)^2)));
	d5 = abs(sqrt(abs((centroidyellow.x - centroidjacket.x)^2 + (centroidyellow.y - centroidjacket.y)^2)) - sqrt(abs((centroidyellow2.x - centroidjacket2.x)^2 + (centroidyellow2.y - centroidjacket2.y)^2)));
	d6 = abs(sqrt(abs((centroidyellow.x - jacketlbound.x)^2 + (centroidyellow.y - jacketlbound.y)^2)) - sqrt(abs((centroidyellow2.x - jacketlbound2.x)^2 + (centroidyellow2.y - jacketlbound2.y)^2)));		
	d7 = abs(sqrt(abs((jacketubound.x - jacketlbound.x)^2 + (jacketubound.y - jacketlbound.y)^2)) - sqrt(abs((jacketubound2.x - jacketlbound2.x)^2 + (jacketubound2.y - jacketlbound2.y)^2)));
	d8 = abs(sqrt(abs((jacketleft.x - jacketright.x)^2 + (jacketleft.y - jacketright.y)^2)) - sqrt(abs((jacketleft.x - jacketright2.x)^2 + (jacketleft2.y - jacketright2.y)^2)));
	d9 = abs(sqrt(abs((centroidyellow.x - centroidgreen.x)^2 + (centroidyellow.y - centroidgreen.y)^2)) - sqrt(abs((centroidyellow2.x - centroidgreen2.x)^2 + (centroidyellow2.y - centroidgreen2.y)^2)));
	motionscore = (d1 + d2 + d3 + d4 + d5 + d6 + d7 + d8 + d9) / 9;

	//copy current values to previous
	previousmotion[0][0] = currentmotion[0][0];
	previousmotion[0][1] = currentmotion[0][1];
	previousmotion[0][2] = currentmotion[0][2];
	previousmotion[0][3] = currentmotion[0][3];
	previousmotion[0][4] = currentmotion[0][4];
	previousmotion[0][5] = currentmotion[0][5];
	previousmotion[0][6] = currentmotion[0][6];
	previousmotion[0][7] = currentmotion[0][7];
	previousmotion[0][8] = currentmotion[0][8];
	previousmotion[0][9] = currentmotion[0][9];
	previousmotion[0][10] = currentmotion[0][10];
	previousmotion[0][11] = currentmotion[0][11];
	previousmotion[0][12] = currentmotion[0][12];
	previousmotion[0][13] = currentmotion[0][13];
	/////////////////////////////////////////////////////////////////////////////////////////////////////


	//Show results for motion detector (difference in pixels of past 3 frames)
	char tempmotion[15];
	if (motionscore <= motionthreshold) {
		sprintf(tempmotion, "NO (%0.2f)", motionscore);
	} else {
		sprintf(tempmotion, "YES (%0.2f)", motionscore);
	}
	if (threadstart == TRUE) {
		cvPutText(displayImage,"MOTION DETECTOR:",cvPoint(10,140), &font, CV_RGB(0,255,0));
		cvPutText(displayImage,tempmotion,cvPoint(500,140), &font, CV_RGB(255,0,255));
		printf("motionscore: %f\n", motionscore);
	}
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


	/// Draw FACE to GLOVE lines ///
	// The first line from face the to one glove
	// The second line from the face to the other glove
	/////////////////////////////////////////////////////////////////////////
	// Draw line between FACE and GLOVE
	if (directionshown == TRUE) {
		cvLine(displayImage, facecentroid, centroidyellow, CV_RGB(255,255,0), 1, 4, 0);
		cvLine(displayImage, facecentroid, centroidgreen, CV_RGB(0,255,0), 1, 4, 0);
	}


	//Show frames on computer sceen
	///////////////////////////////////
	cvShowImage("User Interface", displayImage);

	// Show thresholded image
	IplImage *displayImageCalibration = resizeImage(imageYellowGreenOrange, (int)(framewidth*0.75), (int)(frameheight*0.75), false);
	cvShowImage("Calibration: Thresholded Image", displayImageCalibration);

	//release images
	cvReleaseImage(&displayImage);
	cvReleaseImage(&displayImageflip);
	cvReleaseImage(&displayImageCalibration);
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
	blobsYellowGreen.ClearBlobs();
	blobsOrangeSplit.ClearBlobs();
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void Cleanup() {
	// Free the Face Detector resources when the program is finished
	cvReleaseHaarClassifierCascade(&frontfaceclassifier);
	cvReleaseHaarClassifierCascade(&sidefaceclassifier);

	// Release the capture device housekeeping
	cvReleaseCapture(&capture);
	cvReleaseImage(&frame);
	cvDestroyWindow("User Interface");
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



int main(int argc, const char** argv)
{
	// Initialize variables
	Init();


	//////////////////////////////////////////
	// Go into main thread
	while (1)
	{
		// Acquire frame //
		frame = cvQueryFrame(capture);


		// Do processing here
		DoProcessing();


		// Do not release the frame!
		//If ESC key pressed, Key=0x10001B under OpenCV 0.9.7 (Linux version),
		//remove higher bits using AND operator
		if ((cvWaitKey(1) & 255) == 27) break;
	}
	//////////////////////////////////////////

	// Do cleanup
	Cleanup();

	return 0;
}

