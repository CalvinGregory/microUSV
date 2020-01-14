# Compiles protobuf message classes to c++ and python directories.
protoc -I=./ --cpp_out=./CVSensorSimulator/src ./musv_msg.proto
protoc -I=./ --python_out=./MUSVController/src/mUSV ./musv_msg.proto