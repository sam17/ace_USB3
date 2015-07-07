
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include "ace_usb3.h"

using namespace std;

using namespace cv;

PV_INIT_SIGNAL_HANDLER();

AceUSB3::AceUSB3()
{
	//PV_INIT_SIGNAL_HANDLER();
	lPvSystem = new PvSystem;
	lDeviceInfo = SelectDevice( lPvSystem );
}

AceUSB3::~AceUSB3()
{
	delete lPvSystem;
	lPvSystem = NULL;
}

const PvDeviceInfo *AceUSB3::SelectDevice( PvSystem *aPvSystem )
{
	const PvDeviceInfo *lDeviceInfo = NULL;
	PvResult lResult;

	if (NULL != aPvSystem)
	{
		// Get the selected device information.
		lDeviceInfo = PvSelectDevice( *aPvSystem );
	}

	return lDeviceInfo;
}

PvDevice *AceUSB3::ConnectToDevice( const PvDeviceInfo *aDeviceInfo )
{
	PvDevice *lDevice;
	PvResult lResult;

	// Connect to the GigE Vision or USB3 Vision device
	cout << "Connecting to " << aDeviceInfo->GetDisplayID().GetAscii() << "." << endl;
	lDevice = PvDevice::CreateAndConnect( aDeviceInfo, &lResult );
	if ( lDevice == NULL )
	{
		cout << "Unable to connect to " << aDeviceInfo->GetDisplayID().GetAscii() << "." << endl;
	}
	return lDevice;
}

PvStream *AceUSB3::OpenStream( const PvDeviceInfo *aDeviceInfo )
{
	PvStream *lStream;
	PvResult lResult;
	// Open stream to the GigE Vision or USB3 Vision device
	cout << "Opening stream to device." << endl;
	lStream = PvStream::CreateAndOpen( aDeviceInfo->GetConnectionID(), &lResult );
	if ( lStream == NULL )
	{
		cout << "Unable to stream from " << aDeviceInfo->GetDisplayID().GetAscii() << "." << endl;
	}
	return lStream;
}

void AceUSB3::ConfigureStream( PvDevice *aDevice, PvStream *aStream )
{
	// If this is a GigE Vision device, configure GigE Vision specific streaming parameters
	PvDeviceGEV* lDeviceGEV = dynamic_cast<PvDeviceGEV *>( aDevice );
	if ( lDeviceGEV != NULL )
	{
		PvStreamGEV *lStreamGEV = static_cast<PvStreamGEV *>( aStream );

		// Negotiate packet size
		lDeviceGEV->NegotiatePacketSize();

		// Configure device streaming destination
		lDeviceGEV->SetStreamDestination( lStreamGEV->GetLocalIPAddress(), lStreamGEV->GetLocalPort() );
	}
}

PvPipeline* AceUSB3::CreatePipeline( PvDevice *aDevice, PvStream *aStream )
{
	// Create the PvPipeline object
	PvPipeline* lPipeline = new PvPipeline( aStream );

    if ( lPipeline != NULL )
	{        
        // Reading payload size from device
        uint32_t lSize = aDevice->GetPayloadSize();
	
        // Set the Buffer count and the Buffer size
	    lPipeline->SetBufferCount( BUFFER_COUNT );
        lPipeline->SetBufferSize( lSize );
    }
    
	return lPipeline;
}

void AceUSB3::AcquireImages( PvDevice *aDevice, PvStream *aStream, PvPipeline *aPipeline )
{
	// Get device parameters need to control streaming
	PvGenParameterArray *lDeviceParams = aDevice->GetParameters();

	// Map the GenICam AcquisitionStart and AcquisitionStop commands
	PvGenCommand *lStart = dynamic_cast<PvGenCommand *>( lDeviceParams->Get( "AcquisitionStart" ) );
	PvGenCommand *lStop = dynamic_cast<PvGenCommand *>( lDeviceParams->Get( "AcquisitionStop" ) );

	// Note: the pipeline must be initialized before we start acquisition
	cout << "Starting pipeline" << endl;
	aPipeline->Start();

	// Get stream parameters
	PvGenParameterArray *lStreamParams = aStream->GetParameters();

	// Map a few GenICam stream stats counters
	PvGenFloat *lFrameRate = dynamic_cast<PvGenFloat *>( lStreamParams->Get( "AcquisitionRate" ) );
	PvGenFloat *lBandwidth = dynamic_cast<PvGenFloat *>( lStreamParams->Get( "Bandwidth" ) );

	// Enable streaming and send the AcquisitionStart command
	cout << "Enabling streaming and sending AcquisitionStart command." << endl;
	aDevice->StreamEnable();
	lStart->Execute();

	char lDoodle[] = "|\\-|-/";
	int lDoodleIndex = 0;
	double lFrameRateVal = 0.0;
	double lBandwidthVal = 0.0;

	// Acquire images until the user instructs us to stop.
	cout << endl << "<press a key to stop streaming>" << endl;
	PvFlushKb();
	Mat image;
	gpu::GpuMat rgb;
	namedWindow("Window",WINDOW_NORMAL);
	while ( !PvKbHit() )
	{
		PvBuffer *lBuffer = NULL;
		PvResult lOperationResult;

		// Retrieve next buffer
		PvResult lResult = aPipeline->RetrieveNextBuffer( &lBuffer, 1000, &lOperationResult );
		if ( lResult.IsOK() )
		{
			if ( lOperationResult.IsOK() )
			{
				PvPayloadType lType;

				//
				// We now have a valid buffer. This is where you would typically process the buffer.
				// -----------------------------------------------------------------------------------------
				// ...

				lFrameRate->GetValue( lFrameRateVal );
				lBandwidth->GetValue( lBandwidthVal );

				// If the buffer contains an image, display width and height.
				uint32_t lWidth = 0, lHeight = 0;
				lType = lBuffer->GetPayloadType();

				cout << fixed << setprecision( 1 );
				cout << lDoodle[ lDoodleIndex ];
				cout << " BlockID: " << uppercase << hex << setfill( '0' ) << setw( 16 ) << lBuffer->GetBlockID();
				if ( lType == PvPayloadTypeImage )
				{
				  
				  // Get image specific buffer interface.
				  PvImage *lImage = lBuffer->GetImage();
				  unsigned int width = lImage->GetWidth();
				  unsigned int height = lImage->GetHeight();;
				  unsigned char* dataBuffer = (unsigned char*)lImage->GetBuffer();
				  lWidth = lImage->GetWidth();
				  lHeight = lImage->GetHeight();
				  //image = zeros(Size(width,height),CV_8UC3);
				  IplImage iplImage;
				  iplImage.imageData = (char*)lImage->GetDataPointer();
				  iplImage.imageDataOrigin = iplImage.imageData;
				  iplImage.depth = IPL_DEPTH_8U;
				  iplImage.nChannels = 1;
				  iplImage.width = width;
				  iplImage.height =height ;
				  iplImage.widthStep = lImage->GetBitsPerPixel()/8*width + lImage->GetPaddingX();
				  iplImage.imageSize = lImage->GetImageSize();
				  iplImage.nSize = sizeof(IplImage);
				  iplImage.ID = 0;
				  iplImage.dataOrder = 0;
				  iplImage.origin = 0;
				  iplImage.roi = NULL;
				  iplImage.maskROI = NULL;
				  iplImage.imageId = NULL;
				  iplImage.tileInfo = NULL;
				  IplImage *im = (IplImage*)&iplImage;
				  Mat image = cvarrToMat(im,false);
				  //				  cvCvtColor(im,im,CV_BayerBG2BGR);
				  //				  cvShowImage("ipl",im);
				  
				  //image = Mat(Size(width, height),CV_8SC1,dataBuffer,lImage->GetBitsPerPixel()/8*width + lImage->GetPaddingX());
				  //imshow("Hey",image);
				  gpu::GpuMat img;
				  img.upload(image);
				  //img.convertTo(img,CV_8UC1,0.0625);
				  //rgb = Mat(height,width,CV_8UC3);
				  //gpu::cvtColor(img, rgb, COLOR_BayerBG2RGB,0);
				  gpu::demosaicing(img,rgb,COLOR_BayerBG2RGB);
				  cv::Mat result (rgb);
				  imshow("Window",result);
				  
				  // Read width, height.
				  waitKey(1);
				  cout << "  W: " << dec << lWidth << " H: " << lHeight;
				}
				else {
					cout << " (buffer does not contain image)";
				}
				cout << "  " << lFrameRateVal << " FPS  " << ( lBandwidthVal / 1000000.0 ) << " Mb/s   \r"<<image.total();
			        

				 
			}
			else
			{
				// Non OK operational result
				cout << lDoodle[ lDoodleIndex ] << " " << lOperationResult.GetCodeString().GetAscii() << "\r";
			}

			// Release the buffer back to the pipeline
			aPipeline->ReleaseBuffer( lBuffer );
		}
		else
		{
			// Retrieve buffer failure
			cout << lDoodle[ lDoodleIndex ] << " " << lResult.GetCodeString().GetAscii() << "\r";
		}

		++lDoodleIndex %= 6;
	}

	PvGetChar(); // Flush key buffer for next stop.
	cout << endl << endl;

	// Tell the device to stop sending images.
	cout << "Sending AcquisitionStop command to the device" << endl;
	lStop->Execute();

	// Disable streaming on the device
	cout << "Disable streaming on the controller." << endl;
	aDevice->StreamDisable();

	// Stop the pipeline
	cout << "Stop pipeline" << endl;
	aPipeline->Stop();
}

void AceUSB3::run()
{
	if ( NULL != lDeviceInfo )
	{
        lDevice = ConnectToDevice( lDeviceInfo );
		if ( lDevice != NULL )
		{
            lStream = OpenStream( lDeviceInfo );
			if ( lStream != NULL )
			{
				PvPipeline *lPipeline = NULL;

				ConfigureStream( lDevice, lStream );
				lPipeline = CreatePipeline( lDevice, lStream );
                if( lPipeline )
                {
                    AcquireImages( lDevice, lStream, lPipeline );
                    delete lPipeline;
                }
                
                // Close the stream
                cout << "Closing stream" << endl;
                lStream->Close();
                PvStream::Free( lStream );
			}

            // Disconnect the device
            cout << "Disconnecting device" << endl;
            lDevice->Disconnect();
            PvDevice::Free( lDevice );
		}
	}

	cout << endl;
	cout << "<press a key to exit>" << endl;
	PvWaitForKeyPress();
}
