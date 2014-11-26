ardrone gestures demo
=====================

Installation
------------

Install

- [ardrone autonomy](http://wiki.ros.org/ardrone_autonomy)
- [ar\_track\_alvar\_msgs](https://github.com/jeguzzi/ar_track_alvar_msgs.git) (modified version by Jacopo) 
- [ar\_track\_alvar](https://github.com/jeguzzi/ar_track_alvar.git) (modified version by Jacopo)


TODO
----

- port the code in ardrone\_swarm to catkin

    - References to ar\_track\_alvar and ar\_track\_alvar\_msgs should be changed to ar\_track\_alvar\_idsia and ar\_track\_alvar\_msgs\_idsia
    - Copy useful files from ardrone\_swarm to ardrone\_gestures\_demo
    
    - patch (detectorNode*):
        
Code: 

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
    
    
    

- compile and test the demo
- refactor the code
- add new funcionality







