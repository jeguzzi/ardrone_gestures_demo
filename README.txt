This readme assumes that you are familiar with ROS. If not, please follow the tutorials on the web (the packages have been deployed according to rosbuild).

PHYSICAL ITEMS REQUIRED:
-markers (ar_track_alvar type)

-coloured gloves

SOFTWARE REQUIRED:

- ROS environment installed (the demo is tested on vers. fuerte but should work also on other subsequent versions).

- ROS driver for the ardrone (ardrone_autonomy package, included here but also available on the web).

- the custom version of the ar_track_alvar stack (included here).

- ROS parrot's controller, ROS gesture detector and ROS calibration software (all three of them in ardrone_swarm).

- ROS package sound_play (available on the web)


DEMO SETUP:
Install the custom ar_track_alvar stack in your ROS /stack folder (it should be already compiled). Then place ardrone_autonomy and ardrone_swarm in your workspace. You probably need to compile ardrone_autonomy and ardrone_swarm the first time (follow the instruction on the official web page).
MARKER NOTE: the only changed files in ar_track_alvar should be nodes/individualMarkersNokinect.cpp and msg/AlvarMarker.msg
 

Download and install the sound_play ROS package (In fuerte: apt-get install ros-fuerte-audio-common).

The detector node needs in input a simple .txt file containing the Hmin, Hmax, Smin, Smax, Vmin and Vmax values for the color calibration, and expects to find it in the ~/.ros folder (when launched with a .launch file). Some examples are in ardrone_swarm/bin.

To run the demo (with a single parrot), you need to connect to the parrot via wifi and to launch the drone.launch and detector.launch files (here you can specify the name of the color calibration file). drone.launch launches both the demo and the ardrone driver: therefore, make sure that the ardrone_autonomy package is accessible when launching the node. Then start the demo in the GUI that appears to activate the gesture recognition.

To end the demo, kill firstly the detector process (on the terminal) and then the GUI.


COLOR CALIBRATION:
just run the calib node (ardrone_swarm/bin) when the ardrone_autonomy driver is on and provide as input parameter the name of the calibration file you want to write (e.g. ./calib red_gloves_sala_movimento.txt). If no file is specified, the calibration will be written in "def_calib.txt".
You can click the images on the coloured screen to enlarge the Hmin, Hmax, etc. values that the drone will recognize during the demo. When you are done, just CTRL+C the process. If you run the calibration node with a launch file, the output file will be written in /.ros; otherwise, it will be written in the folder where you starded the process.

FEEDBACK SOUND:
to change/add some other vocal feedbacks (e.g. 'now following Eleonora') record a proper .wav file, put it into ardrone_swarm/src/sound folder and modify ardrone_swarm/src/scripts/sound.py to associate the new audio to a tag number.
In order the sound to work, drone.launch launches soundplay_node.py of package sound_play, which is usually not installed by default with ROS (at least for fuerte).

LEDS:
in the current launch files, the led feedback is unactive, as it is not very precise (however, it can be activaded by uncommenting the drone.launch files)

USING TWO PARROTS:
two options:
- run the same two instances of the demo on two laptops (unconnected)

- connect two laptops (be sure that only one roscore is running, and be carefull with the ROS_MASTER_URI variable) and, on the second laptop, run drone_2.launch and detector_2.launch.
  Launching drone.launch will make the corresponding computer behave as a "master" (the nodes for providing the feedback for both the drones run on this same computer).
  Suggested launch sequence: drone.launch (laptop 1) -> drone2.launch (laptop 2) -> detector.launch (laptop 1) -> detector2.launch (laptop 2). Then click start demo on both the GUIs.
  The idsia hp laptop (named drone2, password drone2) has already installed ros and the software.
  
DEMO EXECUTION:
gestures:
->takeoff (a big blob close to the parrot)
->land (a single blob (shaped like 'x' or 'T' for scenic purposes) ABOVE the marker)
->you (a single blob (shaped like pointing to the robot, also for scenic purposes) ABOVE the marker; there can be also other blobs in the image, but UNDER the marker)
->follow (two blobs ABOVE the marker, one at the left and one at the right)
->right, left (a single blob ABOVE the marker, left or right; there can be also other blobs in the image, but UNDER the marker)

examples of phrases:
-takeoff
-you, follow, right (left)
-you, land

Note that when the robot has landed, it can take off again without the need of resetting anything.

HINTS AND TRICKS:
- When the Parrot crashes (physically), you will probably need to manually reset it (use something sharp to reach the reset button placed near the battery van).
- When doing the landing gesture, make sure that the shadow of one glove does not slices the blob in two.
- After one our of flight or so, the Parrot's engines need some rest (the control becomes less precise).
- Make sure that the floor is not reflecting and has enough features (for the embedded velocity estimation).
- The outdoor hull makes the demo work definetively better than with the indoor hull (at your own risk).

MANUAL COMMANDS:
Takeoff is key "Y". Land is key "H" (Use it in case of troubles! Pressing terminate on the gui has the same effect). The complete keyop can be examined by inspecting ardrone_swarm/src/scripts/keyboard_controller.py.

Note that "C" is used to enter/exit from emergency mode, and that "M" is used to recalibrate the drone's gyroscope when placed on a flat surface (use it after a reset, and
possibly after every inizialization).

If you have any questions, send an email at jaco.banfi@gmail.com.

Have fun!
Jacopo
