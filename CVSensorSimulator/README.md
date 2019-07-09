# CVSensorSimulator

CVSensorSimulator is a desktop application to simulate a microUSV's onboard sensors using an overhead camera system and [AprilTags](https://github.com/AprilRobotics/apriltag). It requires the OpenCV and Protobuf libraries. The host PC most also be outfitted with a webcam and connected to a wifi network, the same wifi network as any microUSV's being tested. 

The application uses a json config file to which should be modified to reflect a user's setup. 

Config Field | Type |  Value
-------------|------|-------
visualize | boolean | Enables or disables the application showing a video feed with labelled tag detections.
cameraInfo |  | Camera calibration parameters (can be found using openCV)
cameraInfo:cameraID | Integer | Overhead camera's index number on the host PC. This can be found by using the ```v4l2-ctl --list-devices``` command
camerInfo:x_res | Integer | camera's x-resolution in pixels
cameraInfo:y_res | Integer | camera's y-resolution in pixels
cameraInfo:fx | Float | camera's x focal length in mm
cameraInfo:fy | Float | camera's y focal length in mm
cameraInfo:cx | Float | camera's x center point coordinate in pixels
cameraInfo:cy | Float | camera's y center point coordinate in pixels
tagsize | Float |  side length of the apriltags in mm (measure the black border)
Robots | Integer, String | List of all robots being used and their associated AprilTag ID numbers and labels
Pucks | Integer | List of all pucks being used and their associated AprilTag numbers
Obstacles | Integer | **Unused field:** List of obstacles marked with AprilTags and their associated tag numbers
Waypoints | Float, Float | List of waypoint coordinates to send to each microUSV
loop_waypoints | Boolean | Flag indicating if waypoint list should be repeated after each one has been reached
output_csv | Boolean | Flag indicating if the application should export the pose history for each tagged object to a CSV file

To launch the application, navigate to the CVSensorSimulator binary and execute the following:

```
$ ./CVSensorSimulator [PATH_TO_CONFIG.JSON]
```
