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

#define BUFFER_COUNT ( 16 )


class AceUSB3
{
	const PvDeviceInfo *lDeviceInfo = NULL;
	PvDevice *lDevice = NULL;
	PvStream *lStream = NULL;
	PvSystem *lPvSystem;
public:
	
	AceUSB3();
	~AceUSB3();

	const PvDeviceInfo *SelectDevice( PvSystem *aPvSystem );
	PvDevice *ConnectToDevice( const PvDeviceInfo *aDeviceInfo );
	PvStream *OpenStream( const PvDeviceInfo *aDeviceInfo );
	void ConfigureStream( PvDevice *aDevice, PvStream *aStream );
	PvPipeline* CreatePipeline( PvDevice *aDevice, PvStream *aStream );
	void AcquireImages( PvDevice *aDevice, PvStream *aStream, PvPipeline *aPipeline );
	void run();
};

#endif
