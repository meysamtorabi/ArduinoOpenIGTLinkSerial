/*=========================================================================

  Program:   Open IGT Link -- Example for Data Receiving Server Program
  Module:    $RCSfile: $
  Language:  C++
  Date:      $Date: $
  Version:   $Revision: $

  Copyright (c) Insight Software Consortium. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#include <iostream>
#include <iomanip>
#include <math.h>
#include <cstdlib>
#include <cstring>

#include "igtlOSUtil.h"
#include "igtlMessageHeader.h"
#include "igtlTransformMessage.h"
#include "igtlImageMessage.h"
#include "igtlServerSocket.h"
#include "igtlStatusMessage.h"
#include "igtlPositionMessage.h"

#if OpenIGTLink_PROTOCOL_VERSION >= 2
#include "igtlPointMessage.h"
#include "igtlTrajectoryMessage.h"
#include "igtlStringMessage.h"
#include "igtlBindMessage.h"
#include "igtlCapabilityMessage.h"
#endif //OpenIGTLink_PROTOCOL_VERSION >= 2


// This file is originally taken from OpenIGTLink examples.
// The file is edited by Meysam Torabi in Nov-Dec 2013 at SNR Lab, BWH.
// The part related for serial communication is taken from https://forums.freebsd.org/viewtopic.php?f=35&t=24453


// Meysam, Nov 29, 2013
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>


// Meysam: Dec 3, 2013
#define SERIALPORT "/dev/ttyACM0" // Specify the serial port
int LEDBrightnessReceiver(igtl::Socket * socket, igtl::MessageHeader * header);
void LEDBrightnessSender(int lEDBrightness);



int main(int argc, char* argv[])
{
  //------------------------------------------------------------
  // Parse Arguments

  if (argc != 2) // check number of arguments
    {
    // If not correct, print usage
    std::cerr << "Usage: " << argv[0] << " <port>"    << std::endl;
    std::cerr << "    <port>     : Port # (18944 in Slicer default)"   << std::endl;
    exit(0);
    }

  int    port     = atoi(argv[1]);

  igtl::ServerSocket::Pointer serverSocket;
  serverSocket = igtl::ServerSocket::New();
  int r = serverSocket->CreateServer(port);

  if (r < 0)
    {
    std::cerr << "Cannot create a server socket." << std::endl;
    exit(0);
    }

  igtl::Socket::Pointer socket;
  
  while (1)
    {
    //------------------------------------------------------------
    // Waiting for Connection
    socket = serverSocket->WaitForConnection(1000);
    
    if (socket.IsNotNull()) // if client connected
      {
      // Create a message buffer to receive header
      igtl::MessageHeader::Pointer headerMsg;
      headerMsg = igtl::MessageHeader::New();

      //------------------------------------------------------------
      // Allocate a time stamp 
      igtl::TimeStamp::Pointer ts;
      ts = igtl::TimeStamp::New();

      //------------------------------------------------------------
      // loop
      for (int i = 0; i < 100; i ++)
        {

        // Initialize receive buffer
        headerMsg->InitPack();

        // Receive generic header from the socket
        int r = socket->Receive(headerMsg->GetPackPointer(), headerMsg->GetPackSize());
        if (r == 0)
          {
          socket->CloseSocket();
          }
        if (r != headerMsg->GetPackSize())
          {
          continue;
          }

        // Deserialize the header
        headerMsg->Unpack();

        // Get time stamp
        igtlUint32 sec;
        igtlUint32 nanosec;
        
        headerMsg->GetTimeStamp(ts);
        ts->GetTimeStamp(&sec, &nanosec);

        // Commented out by Meysam, Dec 3, 2013
        /*std::cerr << "Time stamp: " 
                  << sec << "." << std::setw(9) << std::setfill('0') 
                  << nanosec << std::endl;*/

        // Check data type and receive data body
        if (strcmp(headerMsg->GetDeviceType(), "TRANSFORM") == 0)
          {
          LEDBrightnessReceiver(socket, headerMsg);
          }
          else
          {
          // if the data type is unknown, skip reading.
          std::cerr << "Receiving : " << headerMsg->GetDeviceType() << std::endl;
          std::cerr << "Size : " << headerMsg->GetBodySizeToRead() << std::endl;
          socket->Skip(headerMsg->GetBodySizeToRead(), 0);
          }
        }
      }
    }
    
  //------------------------------------------------------------
  // Close connection (The example code never reaches to this section ...)
  
  socket->CloseSocket();

}





int LEDBrightnessReceiver(igtl::Socket * socket, igtl::MessageHeader * header)
{
  
  // Create a message buffer to receive transform data
  igtl::TransformMessage::Pointer transMsg;
  transMsg = igtl::TransformMessage::New();
  transMsg->SetMessageHeader(header);
  transMsg->AllocatePack();
  
  // Receive transform data from the socket
  socket->Receive(transMsg->GetPackBodyPointer(), transMsg->GetPackBodySize());
  
  // Deserialize the transform data
  // If you want to skip CRC check, call Unpack() without argument.
  int c = transMsg->Unpack(1);
  
  if (c & igtl::MessageHeader::UNPACK_BODY) // if CRC check is OK
    {
    // Retrive the transform data
    igtl::Matrix4x4 matrix;
    transMsg->GetMatrix(matrix);
    // igtl::PrintMatrix(matrix); Commented out by Meysam, Dec 3, 2013

    // Meysam Nov 29, 2013:
    int lEDBrightness = int(matrix[0][3]);
    LEDBrightnessSender(lEDBrightness);
    std::cerr << "The LED's brightness is set to " << lEDBrightness << " out of 255." << std::endl;
    
    return 1;
    }

  return 0;

}
 



// Meysam Dec 3, 2013
void LEDBrightnessSender(int lEDBrightness)
{

    unsigned char wbuf[1];
    int bytes;

    struct termios serialConfig;
    int serial = open(SERIALPORT,O_RDWR | O_NOCTTY);
    if (serial < 0)
    {
       printf("Could not open serial port %d",serial);
    }

    memset(&serialConfig,0,sizeof(serialConfig));
    cfmakeraw(&serialConfig);

    serialConfig.c_cflag = B9600;     // 9600 baud rate (the same as Arduino)
    serialConfig.c_cflag |= CS8;      
    serialConfig.c_cflag |= CLOCAL;   // Local connection, no modem contol
    serialConfig.c_cflag |= CREAD;    // Enable receiving characters
    serialConfig.c_iflag = IGNPAR;    
    serialConfig.c_cc[VMIN] = 0;
    serialConfig.c_cc[VTIME] = 10;    // In deciseconds
    tcflush(serial,TCIOFLUSH);
    tcsetattr(serial,TCSANOW,&serialConfig);

    wbuf[0] = lEDBrightness;          // Get the Value sent by OpenIGTLink
    bytes = write(serial,wbuf,1);
    
}




