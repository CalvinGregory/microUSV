# CVSensorSimulator

CVSensorSimulator is a desktop application to simulate a microUSV's onboard sensors using an overhead camera system and [AprilTags](https://github.com/AprilRobotics/apriltag). It requires the OpenCV and Protobuf libraries. The host PC most also be outfitted with a webcam and connected to a wifi network, the same wifi network as any microUSV's being tested. 

The application uses a json config file to which should be modified to reflect a user's setup. 

Config Field | Value
-------------|------
visualize | Enables or disables the application showing a video feed with labelled tag detections.
cameraInfo | Camera calibration parameters
cameraInfo:cameraID | Host computer's webcam index. A built-in webcam will almost always be index 0. 
camerInfo:x_res | camera's x-resolution
cameraInfo:y_res | camera's y-resolution
cameraInfo:fx | camera's x focal length
cameraInfo:fy | camera's y focal length
cameraInfo:cx | camera's x center point coordinate in pixels
cameraInfo:cy | camera's y center point coordinate in pixels
tagsize | side length of the apriltag in mm (black border)
Robots | List of all robots being used and their associated AprilTag ID numbers and labels
Pucks | List of all pucks being used and their associated AprilTag numbers
Obstacles | **Unused field:** List of obstacles marked with AprilTags and their associated tag numbers
Waypoints | List of waypoint coordinates to send to each microUSV
loop_waypoints | flag indicating if waypoint list should be repeated after each one has been reached

To launch the application, navigate to the CVSensorSimulator binary and execute the following:

```
$ ./CVSensorSimulator [PATH_TO_CONFIG.JSON]
```
