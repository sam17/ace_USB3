#ifndef ACE_USB3_ACE_USB3_H
#define ACE_USB3_ACE_USB3_H

//Pleora SDK Headers
#include <PvSampleUtils.h>
#include <PvDevice.h>
#include <PvDeviceGEV.h>
#include <PvDeviceU3V.h>
#include <PvStream.h>
#include <PvStreamGEV.h>
#include <PvStreamU3V.h>
#include <PvPipeline.h>
#include <PvBuffer.h>
#include <PvSystem.h>

//OpenCV Headers
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

//ROS Headers
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#define BUFFER_COUNT ( 16 )


class AceUSB3
{
	const PvDeviceInfo *lDeviceInfo = NULL;
	PvDevice *lDevice = NULL;
	PvStream *lStream;
	PvSystem *lPvSystem;
	PvPipeline *lPipeline = NULL;
	PvGenParameterArray *lDeviceParams = NULL;
	PvGenParameterArray *lStreamParams = NULL;
	PvGenCommand *lStart;
	PvGenCommand *lStop;
	struct 
	{
	  int height,width;
	  double frameRate,bandwidth;
	}camInfo;

public:
	
	AceUSB3();
	~AceUSB3();

	void ConnectToDevice();
	void OpenStream();
	void ConfigureStream();
	void CreatePipeline();
	void CacheParams();
	void SelectDevice();
	bool GrabImage(sensor_msgs::Image &,sensor_msgs::CameraInfo &);
	void Connect();
	void Disconnect();
	void StartAcquisition();
	void StopAcquisition();

	//const PvDeviceInfo *SelectDevice( PvSystem *aPvSystem );
	//PvDevice *ConnectToDevice( const PvDeviceInfo *aDeviceInfo );
	//PvStream *OpenStream( const PvDeviceInfo *aDeviceInfo );
	//void ConfigureStream( PvDevice *aDevice, PvStream *aStream );
	//PvPipeline* CreatePipeline( PvDevice *aDevice, PvStream *aStream );
	//sensor_msgs::ImagePtr AcquireImages( PvDevice *aDevice, PvStream *aStream, PvPipeline *aPipeline );
	//void run(ros::NodeHandle);
};

#endif
