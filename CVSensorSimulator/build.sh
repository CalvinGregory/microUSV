# Build AprilTag Library Objects
gcc -c -pthread \
src/apriltag/common/g2d.c \
src/apriltag/common/getopt.c \
src/apriltag/common/homography.c \
src/apriltag/common/image_u8.c \
src/apriltag/common/image_u8x3.c \
src/apriltag/common/image_u8x4.c \
src/apriltag/common/matd.c \
src/apriltag/common/pam.c \
src/apriltag/common/pjpeg-idct.c \
src/apriltag/common/pjpeg.c \
src/apriltag/common/pnm.c \
src/apriltag/common/string_util.c \
src/apriltag/common/svd22.c \
src/apriltag/common/time_util.c \
src/apriltag/common/unionfind.c \
src/apriltag/common/workerpool.c \
src/apriltag/common/zarray.c \
src/apriltag/common/zhash.c \
src/apriltag/common/zmaxheap.c \
src/apriltag/apriltag.c \
src/apriltag/apriltag_pose.c \
src/apriltag/apriltag_quad_thresh.c \
src/apriltag/tag25h9.c \
src/apriltag/tag36h11.c

# Build CVSensorSimulator Objects
g++ -c -std=c++11 -pthread \
src/ConfigParser.cpp \
src/CVSS_util.cpp \
src/CVSensorSimulator.cpp \
src/FrameBuffer.cpp \
src/musv_msg.pb.cc \
src/Obstacle.cpp \
src/PoseDetector.cpp \
src/Puck.cpp \
src/Robot.cpp

# Compile Application
g++ -std=c++11 -pthread -o CVSensorSimulator \
apriltag.o \
apriltag_pose.o \
apriltag_quad_thresh.o \
ConfigParser.o \
CVSensorSimulator.o \
CVSS_util.o \
FrameBuffer.o \
g2d.o getopt.o \
homography.o \
image_u8.o \
image_u8x3.o \
image_u8x4.o \
matd.o \
musv_msg.pb.o \
Obstacle.o \
pam.o \
pjpeg.o \
pjpeg-idct.o \
pnm.o \
PoseDetector.o \
Puck.o \
Robot.o \
string_util.o \
svd22.o \
tag25h9.o \
tag36h11.o \
time_util.o \
unionfind.o \
workerpool.o \
zarray.o \
zhash.o \
zmaxheap.o \
-L/usr/local/lib -lopencv_core -lopencv_highgui -lopencv_features2d -lopencv_calib3d -lopencv_videoio -lopencv_imgcodecs -lopencv_imgproc -lprotobuf
