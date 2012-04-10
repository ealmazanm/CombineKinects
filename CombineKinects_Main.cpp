#include "CombineKinects.h"
#include <boost/date_time/posix_time/ptime.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/date_time/posix_time/posix_time_types.hpp"
#include <BackgroundSubtraction_factory.h>
#include <BackgroundDepthSubtraction.h>

using namespace boost::posix_time;

IplImage* backGroundModel;
bool bg_completed = false;
bool transformFlag = true;
boost::mutex mymutex;
int contImages = 0;
bool saveImages = false;

const int CAM_REF = 2;
const int NUM_CAMERAS = 2;


void transformArrayPoints(XnPoint3D* points, const CameraProperties& cam, int numPoints)
{
	CvMat* pointMat = cvCreateMat(3,1,CV_32FC1);
	CvMat* rotTmp = cvCreateMat(3,1,CV_32FC1);
	CvMat* outMat = cvCreateMat(3,1,CV_32FC1);

	XnPoint3D p;	
	for (int i = 0; i < numPoints; i++)
	{
		Utils::fillTheMatrix(pointMat, &(points[i]),3,1);
		cvMatMul(cam.getRotationMatrix(), pointMat, rotTmp);

		cvAdd(rotTmp, cam.getTranslationMatrix(), outMat);

		p.X = (XnFloat)*(outMat->data.fl);
		p.Y = (XnFloat)*(outMat->data.fl + outMat->step/sizeof(float));
		p.Z = (XnFloat)*(outMat->data.fl + 2*outMat->step/sizeof(float));

		points[i] = p;

	}
	cvReleaseMat(&pointMat);
	cvReleaseMat(&rotTmp);
	cvReleaseMat(&outMat);
}

int findCoordinate(float value, float minValue, float maxValue, double step)
{
	if (value < minValue)
		value = minValue;
	if (value > maxValue)
		value = maxValue;

	return (int)floor((value-minValue)/step);
}


void updateImage(XnPoint3D* points3D, int numPoints, const str_actMap* mapValues)
{
	for (int i = 0; i < numPoints; i++)
	{
		int x_2D = findCoordinate(points3D[i].X, mapValues->minX, mapValues->maxX, mapValues->stepX);
		int y_2D = findCoordinate(points3D[i].Z, mapValues->minZ, mapValues->maxZ, mapValues->stepZ);
	
		if (x_2D < XN_VGA_X_RES && y_2D < XN_VGA_Y_RES)
		{
			int y = XN_VGA_Y_RES - y_2D;
			//Create activity map using heat color for the height
			uchar* ptr_Bs = (uchar*)(backGroundModel->imageData + (y*backGroundModel->widthStep));
			ptr_Bs[x_2D*3] = 0;
			ptr_Bs[x_2D*3 + 1] = 0;
			ptr_Bs[x_2D*3 + 2] = 255;
		}
	}
}

void updateForeground(const CameraProperties& cam, BackgroundDepthSubtraction* subtractor, XnDepthPixel* depthMap, const str_actMap* mapValues)
{
	boost::mutex::scoped_lock mylock(mymutex, boost::defer_lock); // defer_lock makes it initially unlocked
	XnPoint3D* points2D = new XnPoint3D[MAX_FORGROUND_POINTS];
	XnPoint3D* points3D = new XnPoint3D[MAX_FORGROUND_POINTS];

	int numPoints = 0;
	bool stop = false;
	int cont = 0;

	numPoints = subtractor->subtraction(points2D, depthMap); //returns the num poins of foreground
//BEGIN Save images
	if (saveImages)
	{
		IplImage* depthImg = cvCreateImageHeader(cvSize(XN_VGA_X_RES, XN_VGA_Y_RES), IPL_DEPTH_8U, 3);
		IplImage* bgModelImg = cvCreateImageHeader(cvSize(XN_VGA_X_RES, XN_VGA_Y_RES), IPL_DEPTH_8U, 3);
		char nameImg_Depth[150];
		char nameImg_BGModel[150];
		char nameImg_Binary[150];
		char idCam[10];
		char idCont[10];
		itoa(contImages, idCont, 10);
		itoa(cam.getCamId(), idCam, 10);

		strcpy(nameImg_Depth, "Depth_");
		strcat(nameImg_Depth, idCam);
		strcat(nameImg_Depth, idCont);
		strcat(nameImg_Depth, ".jpg");

		strcpy(nameImg_BGModel, "BGModel_");
		strcat(nameImg_BGModel, idCam);
		strcat(nameImg_BGModel, idCont);
		strcat(nameImg_BGModel, ".jpg");

		strcpy(nameImg_Binary, "Binary_");
		strcat(nameImg_Binary, idCam);
		strcat(nameImg_Binary, idCont);
		strcat(nameImg_Binary, ".jpg");

		//create depth image
		unsigned short depth[MAX_DEPTH];
		char* depth_data = new char[640*480*3];
		Utils::raw2depth(depth, MAX_DEPTH);
		Utils::depth2rgb(depthMap, depth, depth_data);
		cvSetData(depthImg, depth_data, 640*3);
		cvSaveImage(nameImg_Depth, depthImg);
		cvReleaseImageHeader(&depthImg);
		delete(depth_data);
		//create background model depth image
		const XnDepthPixel *backDepthMap = (const XnDepthPixel *)subtractor->getBackgroundModel();
		depth_data = new char[640*480*3];
		Utils::raw2depth(depth, MAX_DEPTH);
		Utils::depth2rgb(backDepthMap, depth, depth_data);
		cvSetData(bgModelImg, depth_data, 640*3);
		cvSaveImage(nameImg_BGModel, bgModelImg);
		cvReleaseImageHeader(&bgModelImg);
		delete(depth_data);


		IplImage* binaryImg = cvCreateImage(cvSize(XN_VGA_X_RES, XN_VGA_Y_RES), IPL_DEPTH_8U, 1);
		Utils::initImage(binaryImg, 0);
		for (int i = 0; i < numPoints; i++)
		{
			XnPoint3D p = points2D[i];
			uchar *ptrBackImg = (uchar*)binaryImg->imageData + (int)p.Y*binaryImg->widthStep;
			ptrBackImg[(int)p.X] = 255;
		}
		cvSaveImage(nameImg_Binary, binaryImg);
		cvReleaseImage(&binaryImg);
	}
//END Save images

	Utils::backProjectArrayOfPoints(points3D, points2D, cam, numPoints);
	if (cam.getCamId() != CAM_REF)
			transformArrayPoints(points3D, cam, numPoints);

		////update actMapImg with the points
		mylock.lock();
		{
			updateImage(points3D, numPoints, mapValues);
		}
		mylock.unlock();

	delete(points2D);
	delete(points3D);
}

void detectMovement(const vector<const CameraProperties>& cameraList, vector<XnDepthPixel*>& depthMapList, vector<BackgroundDepthSubtraction*>& subtractors, IplImage* actMap, const str_actMap* mapValues)
{
	vector<const CameraProperties>::const_iterator camIter;
	vector<XnDepthPixel*>::iterator depthMapIter;
	vector<BackgroundDepthSubtraction*>::iterator subtIter;
	
	camIter = cameraList.begin();
	depthMapIter = depthMapList.begin();
	subtIter = subtractors.begin();
	thread_group tg;
	while(camIter != cameraList.end())
	{
		XnDepthPixel* dm = *depthMapIter;
		tg.create_thread(boost::bind(updateForeground, *camIter, *subtIter, dm, mapValues));

		camIter++;
		depthMapIter++;
		subtIter++;
	}
	tg.join_all();
	cvResize(backGroundModel, actMap);

}

void grabImage(const CameraProperties& cam, IplImage* rgbImg, XnDepthPixel* depthMap)
{
		cam.getContext()->WaitAndUpdateAll();	
		const XnDepthPixel* dm = cam.getDepthNode()->GetDepthMap();
		const XnRGB24Pixel* rgbMap = cam.getImageNode()->GetRGB24ImageMap();
		if (rgbImg != NULL)
			Utils::fillImageDataFull(rgbImg, rgbMap);

		int total = XN_VGA_X_RES*XN_VGA_Y_RES;
		for (int i = 0; i < total ; i++)
		{
			depthMap[i] = dm[i];
		}
}

void captureInfo_all(const vector<const CameraProperties>& cameraList, vector<XnDepthPixel*>& depthMapList, vector<IplImage*>* rgbImageList)
{
	vector<const CameraProperties>::const_iterator camIter;
	vector<XnDepthPixel*>::iterator depthMapIter;
	vector<IplImage*>::iterator rgbImgIter;
	
	camIter = cameraList.begin();
	depthMapIter = depthMapList.begin();
	rgbImgIter = rgbImageList->begin();
	thread_group tg;
	while(camIter != cameraList.end())
	{
		//const CameraProperties cam = *camIter;
		XnDepthPixel* dm = *depthMapIter;
		IplImage* rgbImg = *rgbImgIter;
//		thread th(grabImage, &cam, rgbImg, dm); 
//		tg.add_thread(&th);
		tg.create_thread(boost::bind(grabImage, *camIter, rgbImg, dm));

		camIter++;
		depthMapIter++;
		rgbImgIter++;
	}
	tg.join_all();
	cout << "All information captured" << endl;
}

int main()
{
	backGroundModel = cvCreateImage(cvSize(XN_VGA_X_RES*2, XN_VGA_Y_RES), IPL_DEPTH_8U, 3);
	vector<const CameraProperties> camList(NUM_CAMERAS);
	vector<XnDepthPixel*> depthMapList(NUM_CAMERAS);
	vector<IplImage*> rgbImageList(NUM_CAMERAS);
	vector<BackgroundDepthSubtraction*> subtractors(NUM_CAMERAS);
	CombineKinects combinator(2,NUM_CAMERAS);
	vector<const CameraProperties>::iterator camIter;
	vector<XnDepthPixel*>::iterator depthMapIter;
	vector<IplImage>::iterator rgbImgIter;
	IplImage* actMap = cvCreateImage(cvSize(XN_VGA_X_RES, XN_VGA_Y_RES), IPL_DEPTH_8U, 3);
	
	for (int i = 0; i < NUM_CAMERAS; i++)
	{
		rgbImageList[i] = cvCreateImage(cvSize(XN_VGA_X_RES, XN_VGA_Y_RES), IPL_DEPTH_8U, 3);
	}

	Utils::rgbdInit(&(camList[0]), &(camList[1]));
	Utils::loadCameraParameters(&(camList[0]));
	Utils::loadCameraParameters(&(camList[1]));
	
	char nameImg[50];
	strcpy(nameImg, "ActMap_");

	//Start cameras
	camIter = camList.begin();
	depthMapIter = depthMapList.begin();
	int total = XN_VGA_Y_RES*XN_VGA_X_RES;
	while(camIter != camList.end())
	{
		*depthMapIter = new XnDepthPixel[total];
		CameraProperties cam = *camIter;
		cam.getContext()->StartGeneratingAll();
		camIter++;
		depthMapIter++;
		
	}

	captureInfo_all(camList, depthMapList, &rgbImageList);

	cvNamedWindow("Activity Map");
	combinator.initializeCombiner(camList, depthMapList);

	bool stop = false;
	int c;
	while (!stop)
	{
		ptime time_start_wait(microsec_clock::local_time());
//		cout << "New frame" << endl;
		captureInfo_all(camList, depthMapList, &rgbImageList);	
		if (bg_completed)
		{
			const str_actMap* mapValues = combinator.getMapValues();
			cvCopyImage(combinator.getBigImage(), backGroundModel);
			detectMovement(camList, depthMapList, subtractors, actMap, mapValues);
		}
		else
		{
			combinator.combine(camList, depthMapList, actMap);
		}

		if (saveImages)
		{
			vector<IplImage*>::iterator rgbIter = rgbImageList.begin();
			int contCams = 0;
			char idCont[10];
			itoa(contImages, idCont, 10);

			char idCam[10];
			char imageName[120];
			strcpy(imageName, "RGB_");
			while(rgbIter != rgbImageList.end())
			{
				IplImage* rgbImg = *rgbIter;
				itoa(contCams, idCam, 10);
				strcat(imageName, idCam);
				strcat(imageName, idCont);
				strcat(imageName, ".jpg");
				cvSaveImage(imageName, rgbImg);
				rgbIter++;
				contCams++;
			}
		
			strcpy(imageName, nameImg);
			strcat(imageName, idCont);
			strcat(imageName, ".jpg");

			cvSaveImage(imageName, actMap);
			contImages++;
			saveImages = false;
		}

		cvShowImage("Activity Map", actMap);
		c = cvWaitKey(1);
		cout << c << " " << endl;
		switch (c)
		{
		case 27: //esc
			{
				stop = true;
				break;
			}
		case 116: //t
			{
				if (combinator.getTransformationFlag())
				{
					combinator.setTransformationFlag(false);
					combinator.initializeCombiner(camList, depthMapList);
					strcpy(nameImg, "ActMap_NoCombined_");
				}
				else
				{
					combinator.setTransformationFlag(true);
					strcpy(nameImg, "ActMap_Combined_");
				}


				break;
			}
		case 115: //s
			{
				saveImages = true;
				break;
			}
		case 99: //c
			{

				int total = subtractors.size();
				for (int i = 0; i < total; i++)
				{

					BackgroundDepthSubtraction* subtractor = new BackgroundDepthSubtraction(depthMapList[i]);
					subtractors[i] = subtractor;
				}
				bg_completed = true;
				break;
			}
		}
		stop  = (27 == c);

		combinator.reset();


		ptime time_end_wait(microsec_clock::local_time());
		time_duration duration_wait(time_end_wait - time_start_wait);
		double frames = (1./duration_wait.total_seconds());
		cout << "fps: " << frames << endl;
		
	}
	//Stop cameras
	camIter = camList.begin();
	depthMapIter = depthMapList.begin();
	vector<IplImage*>::iterator rgbIter = rgbImageList.begin();
	while(camIter != camList.end())
	{
		delete(*depthMapIter);
		CameraProperties cam = *camIter;
		cam.getContext()->StopGeneratingAll();
		cvReleaseImage(&(*rgbIter));
		camIter++;
		depthMapIter++;
		rgbIter++;
	}

	cvDestroyAllWindows();
	cvReleaseImage(&actMap);
	return 1;
}
