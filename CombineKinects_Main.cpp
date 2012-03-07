#include "CombineKinects.h"

/*Global variables*/
boost::mutex mymutex;

/*header methods*/
//Take the rgb image and the depth map of cam
void grabImage(CameraProperties* cam, IplImage* depthImg, XnDepthPixel* depthMap);

//Calcualte the dimensionts in 3D of the activity map. Set the size of the grids in the plane XZ
//to project the points.
void initActMapValues(str_actMap* actMap_confVal, CameraProperties* cam2, const XnDepthPixel* depthMap2, CameraProperties* cam1, const XnDepthPixel* depthMap1);

//Backproject the pixel (x,y) into a 3D point in the space (X,Y,Z) and check if the coordinates of the point are edges in the monitored area.
void checkCamPoint(int x, int y, const XnDepthPixel** depthMap, CameraProperties* cam, float* minX, float* maxX, float* minY, float* maxY, float* minZ, float* maxZ, bool* firstTime);

//Compare de coordinates of p3D with the minimum and maximum values in each axes. These values are updated in case any coordinate of p3D
//is a limit
void compareValues(XnPoint3D *p3D, float* minX, float* maxX, float* minY, float* maxY, float* minZ, float* maxZ);

//Calcualte the width of the grid based on the min and max values and the number of bins.
float findStep(float fmin, float fmax, int nBins);

//Create a combined activity map from the depthMaps
void getActivityMapImage(const str_actMap* actMap, IplImage* actMapImg, const XnDepthPixel** depthMap1, CameraProperties* cam1 ,const XnDepthPixel** depthMap2, CameraProperties* cam2);

//Fill the activity map with the information from the depthMap. BackProject the pixels into points in the space and then project them again
// but into the ground plane. if camId = CAMID_TRANSFORMATION then the point in the space is transformed (R and t).
void fillImages(int x, int y, const XnDepthPixel** depthMap, CameraProperties* cam, const str_actMap* actMap, IplImage* actMapImg, int** colorMap);

//Finde the projection of value in the ground plane.
int findCoordinate(float value, float minValue, float maxValue, double step);

//Transform the height into a gray scale color (0-255)
int findColor(float value, float minValue, float maxValue, double step);

//Transform the gray scale color map into a heat map color.
void findHeatColour(int alpha, int* r, int* g, int* b);
/****************************************************************************************************/

/*Implementation*/
void grabImage(CameraProperties* cam, IplImage* rgbImg, XnDepthPixel* depthMap)
{
		cam->getContext()->WaitAndUpdateAll();	
		const XnDepthPixel* dM = cam->getDepthNode()->GetDepthMap();
		const XnRGB24Pixel* rgbMap = cam->getImageNode()->GetRGB24ImageMap();
		Utils::fillImageDataFull(rgbImg, rgbMap);
		int total = XN_VGA_X_RES*XN_VGA_Y_RES;
		for (int i = 0; i < total; i++)
			depthMap[i] = dM[i];

}

void initActMapValues(str_actMap* actMap_confVal, CameraProperties* cam2, const XnDepthPixel* depthMap2, CameraProperties* cam1, const XnDepthPixel* depthMap1)
{
	bool firstTime = true;
	float minX, minY, minZ;
	float maxX, maxY, maxZ;
	for (int y = 0; y < XN_VGA_Y_RES; y++)
	{
		for (int x= 0; x < XN_VGA_X_RES; x++)
		{
			//cam1
			checkCamPoint(x, y, &depthMap1, cam1, &minX, &maxX, &minY, &maxY, &minZ, &maxZ, &firstTime);
			//cam2
			checkCamPoint(x, y, &depthMap2, cam2, &minX, &maxX, &minY, &maxY, &minZ, &maxZ, &firstTime);
		}
	}
	actMap_confVal->maxX = maxX; actMap_confVal->maxY = maxY; actMap_confVal->maxZ = maxZ;
	actMap_confVal->minX = minX; actMap_confVal->minY = minY; actMap_confVal->minZ = minZ;
	//Find the size of the bins (x (X), y(Z) and color(Y)). 3D-2D
	actMap_confVal->stepX = findStep(minX, maxX, XN_VGA_X_RES*2);
	actMap_confVal->stepY = findStep(minY, maxY, 256);
	actMap_confVal->stepZ = findStep(minZ, maxZ, XN_VGA_Y_RES);
}

float findStep(float fmin, float fmax, int nBins)
{
	float totalValues = abs(fmax - fmin);
	return totalValues/nBins;
}

void checkCamPoint(int x, int y, const XnDepthPixel** depthMap, CameraProperties* cam, float* minX, float* maxX, float* minY, float* maxY, float* minZ, float* maxZ, bool* firstTime)
{
	XnPoint3D p3D;
	int z = (*depthMap)[y*XN_VGA_X_RES+x];	
	if (z != 0)
	{
		XnPoint3D p;
		p.X = (XnFloat)x;
		p.Y = (XnFloat)y;
		p.Z = (XnFloat)z;
		cam->backProjectPoint(&p, &p3D);
		if (cam->getCamId() == CAMID_TRANSFORMATION)
			Utils::transformPoint(&p3D, cam);
		if (*firstTime)
		{
			*minX = p3D.X; *minY = p3D.Y; *minZ = p3D.Z;
			*maxX = p3D.X; *maxY = p3D.Y; *maxZ = p3D.Z;
			*firstTime = false;
		}
		else
			compareValues(&p3D, minX, maxX, minY, maxY, minZ, maxZ); 
	}
}

void compareValues(XnPoint3D *p3D, float* minX, float* maxX, float* minY, float* maxY, float* minZ, float* maxZ)
{
	if (p3D->X < *minX)
		*minX = p3D->X;
	else if (p3D->X > *maxX)
		*maxX = p3D->X;

	if (p3D->Y < *minY)
		*minY = p3D->Y;
	else if (p3D->Y > *maxY)
		*maxY = p3D->Y;

	if (p3D->Z < *minZ)
		*minZ = p3D->Z;
	else if (p3D->Z > *maxZ)
		*maxZ = p3D->Z;				

}

void getActivityMapImage(const str_actMap* actMap, IplImage* actMapImg, const XnDepthPixel** depthMap1, CameraProperties* cam1 ,const XnDepthPixel** depthMap2, CameraProperties* cam2)
{
	//Color map to store the height from 0 to 255.
	int* colorMap = new int[(XN_VGA_X_RES*2)*XN_VGA_Y_RES];
	//init the colorMap to 0;
/*	for (int y = 0; y < XN_VGA_Y_RES; y++)
	{
		for (int x = 0; x < XN_VGA_X_RES*2; x++)
		{
			colorMap[y*XN_VGA_X_RES+x] = 0;
		}
	}
*/
	Utils::initImage3Channel(actMapImg, 0);

	for (int y = 0; y < XN_VGA_Y_RES; y++)
	{
		for (int x= 0; x < XN_VGA_X_RES; x++)
		{
			//cam1
			//boost::thread thr(fillImages, x, y, depthMap1, cam1, actMap, actMapImg, &colorMap);
			fillImages(x, y, depthMap1, cam1, actMap, actMapImg, &colorMap);
			//cam2
			fillImages(x, y, depthMap2, cam2, actMap, actMapImg, &colorMap);
			//thr.join();
		}
	}
	cout << "Frame" << endl;
	delete(colorMap);
}

void fillImages(int x, int y, const XnDepthPixel** depthMap, CameraProperties* cam, const str_actMap* actMap, IplImage* actMapImg, int** colorMap)
{
//	boost::mutex::scoped_lock mylock(mymutex, boost::defer_lock); // defer_lock makes it initially unlocked
	XnPoint3D p3D, p;
	int z = (*depthMap)[y*XN_VGA_X_RES+x];	
	if (z != 0)
	{
		p.X = (XnFloat)x;
		p.Y = (XnFloat)y;
		p.Z = (XnFloat)z;
		cam->backProjectPoint(&p, &p3D);
		if (cam->getCamId() == CAMID_TRANSFORMATION)
			Utils::transformPoint(&p3D, cam);
		if (p3D.Y > -500) // filter the height.
		{
			int x_2d = findCoordinate(p3D.X, actMap->minX, actMap->maxX, actMap->stepX);
			int y_2d = findCoordinate(p3D.Z, actMap->minZ, actMap->maxZ, actMap->stepZ);
			int color = findColor(p3D.Y, actMap->minY, actMap->maxY, actMap->stepY); //between 0 and 255

			//Create depth image using heat color for the height
			int r, g, b;
			findHeatColour(color, &r, &g, &b);

			//Create activity map using heat color for the height
//			mylock.lock();
			uchar* ptr_Bs = (uchar*)(actMapImg->imageData + (y_2d*actMapImg->widthStep));
			if ((*colorMap)[y_2d*XN_VGA_X_RES+x_2d] < color)
			{			
				ptr_Bs[x_2d*3] = b;
				ptr_Bs[x_2d*3 + 1] = g;
				ptr_Bs[x_2d*3 + 2] = r;
				(*colorMap)[y_2d*XN_VGA_X_RES+x_2d] = color;
			}
//			mylock.unlock();
			//test
		}
	}
}

int findColor(float value, float minValue, float maxValue, double step)
{
	if (value < minValue)
		value = minValue;
	if (value > maxValue)
		value = maxValue;

	return (int)floor(-(value-maxValue)/step);

}

void findHeatColour(int alpha, int* r, int* g, int* b)
{
	int tmp;
	*r = 0;
	*g = 0;
	*b = 0;
	if(alpha <= 255 && alpha >= 215){
		tmp=255-alpha;
		*r=255-tmp;
		*g=tmp*8;
	}else if(alpha <= 214 && alpha >= 180){
		tmp=214-alpha;
		*r=255-(tmp*8);
		*g=255;
	}else if(alpha <= 179 && alpha >= 130){
		tmp=179-alpha;
		*g=255;
		*b=tmp*1;
	}else if(alpha <= 129 && alpha >= 80){
		tmp=129-alpha;
		*g=255-(tmp*1);
		*b=255;
	}else
		*b=255;	
}


int findCoordinate(float value, float minValue, float maxValue, double step)
{
	if (value < minValue)
		value = minValue;
	if (value > maxValue)
		value = maxValue;

	return (int)floor((value-minValue)/step);
}

int main()
{
	//Init all sensors (init context)
	CameraProperties cam1, cam2;
	Utils::rgbdInit(&cam1, &cam2);
	Utils::loadCameraParameters(&cam1);
	Utils::loadCameraParameters(&cam2);

	//declare image variables
	IplImage* rgbImg1 = cvCreateImage(cvSize(XN_VGA_X_RES, XN_VGA_Y_RES), IPL_DEPTH_8U, 3);
	IplImage* rgbImg2 = cvCreateImage(cvSize(XN_VGA_X_RES, XN_VGA_Y_RES), IPL_DEPTH_8U, 3);
	IplImage* actMapImg = cvCreateImage(cvSize(XN_VGA_X_RES*2, XN_VGA_Y_RES), IPL_DEPTH_8U, 3);
	XnDepthPixel *depthMap1, *depthMap2;

	depthMap1 = new XnDepthPixel[XN_VGA_X_RES*XN_VGA_Y_RES];
	depthMap2 = new XnDepthPixel[XN_VGA_X_RES*XN_VGA_Y_RES];

	//Window names
	char* wind_actMap = "Combined Activity Map";
	char* wind_rgbImg1 = "RGB cam1";
	char* wind_rgbImg2 = "RGB cam2";

	cvNamedWindow(wind_actMap);
	cvNamedWindow(wind_rgbImg1);
	cvNamedWindow(wind_rgbImg2);

	str_actMap actMap_confVal;
	bool stop = false;
	//to initialize the boundaries and size of the bins in the projected plane
	bool first = true; 

	//Start capturing images from cameras 1 and 2.
	cam1.getContext()->StartGeneratingAll();
	cam2.getContext()->StartGeneratingAll();

	while (!stop)
	{
		//take images (threads) 
		boost::thread thr(grabImage, &cam2, rgbImg2, depthMap2);
		grabImage(&cam1, rgbImg1, depthMap1);
		thr.join();


		if (first)//only first time - expensive
		{
			initActMapValues(&actMap_confVal, &cam2, depthMap2, &cam1, depthMap1);
			first = false;
		}

		//create activity map for both sensors
		getActivityMapImage(&actMap_confVal, actMapImg, (const XnDepthPixel**)&depthMap1, &cam1,(const XnDepthPixel**)&depthMap2, &cam2);
		cvFlip(actMapImg, NULL,0);
		
		//display them.
		cvShowImage(wind_rgbImg1 , rgbImg1);
		cvShowImage(wind_rgbImg2, rgbImg2);
		cvShowImage(wind_actMap, actMapImg);
		char c = cvWaitKey(1);
		stop = (c == 27);

	}
	//Teminate capturing images
	cam1.getContext()->StopGeneratingAll();
	cam2.getContext()->StopGeneratingAll();
	//Free memory
	delete(depthMap1);
	delete(depthMap2);
	cvDestroyAllWindows();
	cvReleaseImage(&rgbImg1);
	cvReleaseImage(&rgbImg2);
	cvReleaseImage(&actMapImg);
	return 0;
}