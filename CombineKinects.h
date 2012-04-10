/**
* CombineKinects.h
* Class for combine the information from several cameras into one image.
* Simulates a top-down view of the scene.
*
* Libraries: OpenNI, OpenCV, Boost.
*
* Author: Emilio J. Almazan <emilio.almazan@kingston.ac.uk>, 2012
*/
#pragma once
#include "XnCppWrapper.h"
#include "Utils.h"
#include "CameraProperties.h"
#include <iostream>
#include <assert.h>
#include <boost/thread.hpp>
#include <boost/lambda/lambda.hpp>

using namespace std;
using namespace xn;
using namespace boost;

const int MAX_DEPTH = 10000;
const int MAX_Z = 5000; // delimits the depth of the display
const int MAX_Y = -500; // the Y axis goes from possitive to negative

//Data type to store the limits and bins (X,Y,Z) for the activiytMap
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
	bool init;
};

class CombineKinects
{
public:
	CombineKinects(void);
	CombineKinects(int, int);
	~CombineKinects(void);
	//Initialize the limits and the size of the pixels of the ground plane where the points will be projected.
	void initializeCombiner(const vector<const CameraProperties>& cameraList, const vector<XnDepthPixel*>& depthMapList);

	//Combine the information from all the cameras "cameraList" into one 2D image "out" that is the activityMap.
	//The activity map is a simulated top down view of the scene.
	void combine(const vector<const CameraProperties>& cameraList, const vector<XnDepthPixel*>& depthMapList, IplImage* out);

	//Set the value of "transFlag"
	void setTransformationFlag(bool val);

	//Assign a value to "transFlag"
	bool getTransformationFlag();

	//Return the big image (original size)
	const IplImage* getBigImage();

	const str_actMap* getMapValues();

	//Initialize the list of points for the next frame
	void reset();

private:

	void combineImages(vector<IplImage*>& actMapImg_Indiv );

	//Updates the values of "actMapValues" with the information from depthMap. If the id of "cam" is different to 
	// "referenceCS" then the data will be transformed (R and t)
	void updateActMapValues(const CameraProperties& cam, const XnDepthPixel* depthMap, str_actMap* mapValues);

	//Combine the information in the same coordinate system
	void combine_Trans(const vector<const CameraProperties>& cameraList, const vector<XnDepthPixel*>& depthMapList, IplImage* out);

	//Combine the information in different coordinate systems
	void combine_NoTrans(const vector<const CameraProperties>& cameraList, const vector<XnDepthPixel*>& depthMapList, IplImage* out);

	//Fill the vector of 3D points with the information from "cam". If the id of "cam" is not "referenceCS", then
	//the points are transformed.
	void fill3DPointVector(const CameraProperties& cam, const XnDepthPixel* depthMap);

	//Create an actibity map image from the depth passed
	void fillImage_Indiv(const CameraProperties& cam, const XnDepthPixel* depthMap, str_actMap& mapValues, IplImage* img);

	//Fill the image with the 3D points. Create the activity map, where the highest points are represeted.
	//Simulated top-down view of the scene
	void fillImage();

	//Calculate the size of the bin.
	float findStep(float fmin, float fmax, int nBins);

	//Calcuate the grid where "value" projects.
	int findCoordinate(float value, float minValue, float maxValue, double step);

	//Like findCoordinate but it nevers return a negative value
	int findColor(float value, float minValue, float maxValue, double step);

	//Convert a grey scale color to a heat color
	void findHeatColour(int alpha, int* r, int* g, int* b);

	str_actMap actMapVals;
	//Define the camera coordinate system, wich will be the common one for the rest of the cameras
	const int referenceCS;
	//Define the number of cameras to be combine
	const int numCameras;

	vector<str_actMap>* mapVals_all;

	//Stores all the points in 3D from all the cameras
	vector<XnPoint3D> point3D_All;
	int contPoint;

	//threading pourposes. To access the shared memory of "actMapValues"
	mutex Initmutex, point3D_Mutex;

	//Indicates if the transformation will be performed
	bool transFlag;

	IplImage* bigImage;
};

