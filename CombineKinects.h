#pragma once
#include "XnCppWrapper.h"
#include "CameraProperties.h"
#include <boost/thread.hpp>
#include <boost/date_time.hpp> 
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "stdio.h"
#include "Utils.h"
#include "filePaths.h"
#include <Windows.h>
#include <iostream>
#include <fstream>
#include <string>
#include <new>
#include <list>

using namespace std;
using namespace xn;
using std::string;


//Select the camera cs that will be transformed into the other.
const int CAMID_TRANSFORMATION = 1;
const int MAX_DEPTH = 10000;
static  ofstream outDebug;


struct str_actMap
{
	float maxX;
	float maxY;
	float maxZ;
	float minX;
	float minY;
	float minZ;
	float stepX;
	float stepY;
	float stepZ;
};