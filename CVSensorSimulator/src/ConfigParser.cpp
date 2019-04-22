/*
 * ConfigParser.cpp
 *
 *  Created on: Apr. 20, 2019
 *      Author: calvin
 */

#include "ConfigParser.h"

using namespace std;

enum ArgType { 	VisualizeFlag,
				CameraID,
				xResolution,
				yResolution,
				xFocalLength,
				yFocalLength,
				xCenter,
				yCenter,
				TagSize,
				Robot_,
				Puck_ };

static std::map<std::string, ArgType> argType;

ConfigParser::ConfigParser() {
	initArgType();
}

Config ConfigParser::getConfigs(string filepath) {
	Config config;

	ifstream cFile (filepath);
	if (cFile.is_open()) {
		string line;
		while(getline(cFile, line)) {
			line.erase(remove_if(line.begin(), line.end(), ::isspace), line.end());
			if(line[0] == '#' || line.empty()) {
				continue;
			}

			int delimiterPos = line.find("=");
			string name = line.substr(0, delimiterPos);
			string value = line.substr(delimiterPos + 1);
			switch(argType[name])
			{
				case VisualizeFlag:
					config.visualize = stoi(value);
					break;
				case CameraID:
					config.cameraID = stoi(value);
					break;
				case xResolution:
					config.x_res = stoi(value);
					break;
				case yResolution:
					config.y_res = stoi(value);
					break;
				case xFocalLength:
					config.fx = stod(value);
					break;
				case yFocalLength:
					config.fy = stod(value);
					break;
				case xCenter:
					config.cx = stod(value);
					break;
				case yCenter:
					config.cy = stod(value);
					break;
				case TagSize:
					config.tagsize = stod(value);
					break;
				case Puck_:
					// TODO add Puck parser
					break;
				case Robot_:
					int delim1 = value.find(",");
					string id = value.substr(0, delim1);
					string label = value.substr(delim1 + 1);
					Robot robot(stoi(id), 0, 0, label);
					config.robots.push_back(robot);
					break;
			}
		}
	}
	else {
		cerr << "Could not open config file " << filepath << endl;
	}
	cFile.close();
	return config;
}

void ConfigParser::initArgType() {
	argType["visualize"] = VisualizeFlag;
	argType["cameraID"] = CameraID;
	argType["res_x"] = xResolution;
	argType["res_y"] = yResolution;
	argType["fx"] = xFocalLength;
	argType["fy"] = yFocalLength;
	argType["cx"] = xCenter;
	argType["cy"] = yCenter;
	argType["tagsize"] = TagSize;
	argType["Robot"] = Robot_;
	argType["Puck"] = Puck_;
}
