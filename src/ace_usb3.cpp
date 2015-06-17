
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include "ace_usb3.h"

using namespace std;

using namespace cv;

PV_INIT_SIGNAL_HANDLER();

AceUSB3::AceUSB3()
{
	lPvSystem = new PvSystem;
	SelectDevice();
}

AceUSB3::~AceUSB3()
{
	delete lPvSystem;
}

void AceUSB3::SelectDevice( )
{
	PvResult lResult;

	if (NULL != lPvSystem)
	{
		lDeviceInfo = PvSelectDevice(*lPvSystem );
	}
}

void AceUSB3::ConnectToDevice( )
{
	PvResult lResult;
	cout << "Connecting to " << lDeviceInfo->GetDisplayID().GetAscii() << "." << endl;
	lDevice = PvDevice::CreateAndConnect(lDeviceInfo, &lResult );
	if ( lDevice == NULL )
	{
		cout << "Unable to connect to " << lDeviceInfo->GetDisplayID().GetAscii() << "." << endl;
	}
	lDeviceParams = lDevice->GetParameters();
}

void AceUSB3::OpenStream( )
{
	PvResult lResult;
	cout << "Opening stream to device." << endl;
	lStream = PvStream::CreateAndOpen(lDeviceInfo->GetConnectionID(),&lResult);
	if ( lStream == NULL )
	{
		cout << "Unable to stream from " << lDeviceInfo->GetDisplayID().GetAscii() << "." << endl;
	}
}

void AceUSB3::ConfigureStream( )
{
	PvDeviceGEV* lDeviceGEV = dynamic_cast<PvDeviceGEV *>( lDevice );
	if ( lDeviceGEV != NULL )
	{
		PvStreamGEV *lStreamGEV = static_cast<PvStreamGEV *>( lStream );
		// Negotiate packet size
		lDeviceGEV->NegotiatePacketSize();
		// Configure device streaming destination
		lDeviceGEV->SetStreamDestination( lStreamGEV->GetLocalIPAddress(), lStreamGEV->GetLocalPort() );
	}
}

void AceUSB3::CreatePipeline()
{
	// Create the PvPipeline object
  lPipeline = new PvPipeline( lStream );

  if ( lPipeline != NULL )
    {        
      // Reading payload size from device
      uint32_t lSize = lDevice->GetPayloadSize();
      // Set the Buffer count and the Buffer size
      lPipeline->SetBufferCount( BUFFER_COUNT );
      lPipeline->SetBufferSize( lSize );
    }
}

void AceUSB3::Connect()
{
  ConnectToDevice();
  OpenStream();
  ConfigureStream();
  CreatePipeline();
  lStart = dynamic_cast<PvGenCommand *>( lDeviceParams->Get( "AcquisitionStart" ) );
  lStop = dynamic_cast<PvGenCommand *>( lDeviceParams->Get( "AcquisitionStop" ) );
}

void AceUSB3::Disconnect()
{
  delete lPipeline;
  lStream->Close();
  PvStream::Free(lStream );
  lDevice->Disconnect();
  PvDevice::Free(lDevice);
  lDeviceParams = NULL;
  lStreamParams = NULL;
}

void AceUSB3::StartAcquisition()
{
  lPipeline->Start();
  lDevice->StreamEnable();
  lStart->Execute();
  CacheParams();
}

void AceUSB3::StopAcquisition()
{
  if(!lDeviceParams)
    return;
  lStop->Execute();
  lDevice->StreamDisable();
  lPipeline->Stop();
}
 
void AceUSB3::CacheParams()
{
  int64_t width, height;
  lStreamParams = lStream->GetParameters();
  PvGenFloat *lFrameRate = dynamic_cast<PvGenFloat *>( lStreamParams->Get( "AcquisitionRate" ) );
  PvGenFloat *lBandwidth = dynamic_cast<PvGenFloat *>( lStreamParams->Get( "Bandwidth" ) );
  double lFrameRateVal = 0.0;
  double lBandwidthVal = 0.0;
  lFrameRate->GetValue( lFrameRateVal );
  lBandwidth->GetValue( lBandwidthVal );
  lStreamParams->GetIntegerValue("Width",width);
  lStreamParams->GetIntegerValue("Height",height);
  camInfo.height = height;
  camInfo.width = width;
  camInfo.frameRate = lFrameRateVal;
  camInfo.bandwidth = lBandwidthVal;
} 

bool AceUSB3::GrabImage(sensor_msgs::Image &image_msg,sensor_msgs::CameraInfo &cinfo_msg)
{
  static bool skip_next_frame = false;

  // Start loop for acquisition
  PvBuffer *buffer;
  PvResult op_result;

  // Skip next frame when operation is not ok
  if (skip_next_frame) 
    {
      skip_next_frame = false;
      sleep(1);
    }

  // Retrieve next buffer
  PvResult result = lPipeline->RetrieveNextBuffer(&buffer, 1000, &op_result);

  // Failed to retrieve buffer
  if (result.IsFailure()) 
  {
    
     return false;
  }

  // Operation not ok, need to return buffer back to pipeline
  if (op_result.IsFailure()) 
  {
      skip_next_frame = true;
      // Release the buffer back to the pipeline
      lPipeline->ReleaseBuffer(buffer);
      return false;
  }

  // Buffer is not an image
  if ((buffer->GetPayloadType()) != PvPayloadTypeImage) 
    {
      lPipeline->ReleaseBuffer(buffer);
      return false;
    }

  // Get image specific buffer interface
  PvImage *lImage = buffer->GetImage();
  unsigned int width = lImage->GetWidth();
  unsigned int height = lImage->GetHeight();

  // Assemble image msg
  image_msg.height = height;
  image_msg.width = width;
  image_msg.encoding = sensor_msgs::image_encodings::BAYER_BGGR8;
  image_msg.step = lImage->GetBitsPerPixel()/8*width + lImage->GetPaddingX();
  unsigned char* dataBuffer = (unsigned char*)lImage;
  const size_t data_size = lImage->GetImageSize();
  if (image_msg.data.size() != data_size) 
  {
      image_msg.data.resize(data_size);
  }
  memcpy(&image_msg.data[0], lImage->GetDataPointer(), lImage->GetImageSize());
  cout<<"Width: "<<width<<" Height: "<<height<<endl;

  lPipeline->ReleaseBuffer(buffer);
  return true;
}
