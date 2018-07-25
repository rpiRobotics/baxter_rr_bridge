Baxter Robot Raconteur Bridge
================

A catkin package to host the ROS topics of the Baxter robot as a Robot Raconteur (RR) service.  This allows for communication through any of the platforms and languages supported in RR, such as Python, C++, C#, MATLAB and Java. 

## July 2018:
### New features added in the latest update:
* Camera host now requires the installation of OpenCV on the workstation. It is for the new function of Aruco Tag Detection. Calling `ARtag_Detection()` will give the IDs of detected tags and transformation matrices from camera frame to tag frame. The transformation matrices are reshaped to a list so they need to be reshape back to 4x4 before being used. If there are more than one tags, the transformations will be appended in the order of IDs. Note that the tag dictionary is `DICT_ARUCO_ORIGINAL` and the default tag size is `8.5 centimeters`. The size can be changed manually by calling function `setMarkerSize($theSizeinMeters)`. Also Camera Intrinsic parameters can be set manually by calling the function `setCameraIntrinsics($intrinsics)`, allowing more accurate results from calibrations to enhance the accuracy of AR tag detections and relative pose extraction.
* Frame to frame transformation listener function using tf package provided by ROS is added to peripheral host. The function `lookUptransforms($frame1, $frame2)` returns quaternion `[x, y, z, w]'` and position `[x, y, z]'` from $frame1 to $frame2.
* Inverse Kinematics solver which uses the IKfast service provided by ROS is added to the joint controller host. Calling function `solveIKfast(position, orientation, $arm)` will return a list of 7 joint angles for the $arm ('left' or 'right') if there is at least one solution. Otherwise it will return an empty list. The position and orientation is the target pose of the end effector in the base frame.
