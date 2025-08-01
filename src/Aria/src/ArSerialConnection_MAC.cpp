#include "ArSerialConnection.h"
#include "ariaUtil.h"
#include <sys/ioctl.h>
#include <termios.h>
#include <fcntl.h>
// #include <errno.h>
// #include <IOKit/serial/IOSerialKeys.h>

// Add missing baud rate defines
#ifndef B460800
#define B460800 460800
#endif

#ifndef TIOSTARTTIMESTAMP
#define TIOSTARTTIMESTAMP  _IOW('T', 200, unsigned long)
#endif

#ifndef TIOGETTIMESTAMP
#define TIOGETTIMESTAMP   _IOR('T', 201, struct timeval)
#endif

AREXPORT ArSerialConnection::ArSerialConnection() 
{
    myPort = -1;
    myBaud = 9600;
    myHardwareControl = false;
    myStatus = STATUS_NEVER_OPENED;
    // strncpy(myPortName, "none", sizeof(myPortName)-1);
    strncpy(myPortName, "/dev/ttyS0", sizeof(myPortName)-1);
    myPortName[sizeof(myPortName)-1] = '\0';
//   myPort = -1;
//   myPortName = "none";
//   myBaud = 9600;
//   myHardwareControl = false;
//   myStatus = STATUS_NEVER_OPENED;
//   myStringBuf[0] = '\0';

//   myPortNameStr = "none";
//   strncpy(myPortName, myPortNameStr.c_str(), sizeof(myPortName)-1);
//   strncpy(myPortName, "none", sizeof(myPortName)-1);
//   myPortName[sizeof(myPortName)-1] = '\0';
//   myBaud = 9600;
//   myStatus = STATUS_NEVER_OPENED;
//   myHardwareControl = false;
//   myPort = -1;

}

AREXPORT ArSerialConnection::~ArSerialConnection() {
  if (myPort != -1)
    close();
}

void ArSerialConnection::buildStrMap(void)
{
  myStrMap[OPEN_COULD_NOT_OPEN_PORT] = "Could not open serial port.";
  myStrMap[OPEN_COULD_NOT_SET_UP_PORT] = "Could not set up serial port.";
  myStrMap[OPEN_INVALID_BAUD_RATE] = "Baud rate invalid, could not set baud on serial port.";
  myStrMap[OPEN_COULD_NOT_SET_BAUD] = "Could not set baud rate on serial port.";
  myStrMap[OPEN_ALREADY_OPEN] = "Serial port already open.";
}

AREXPORT const char * ArSerialConnection::getOpenMessage(int messageNumber)
{
  return myStrMap[messageNumber].c_str();
}

AREXPORT int ArSerialConnection::internalOpen(void)
{
  struct termios tio;

  if (myStatus == STATUS_OPEN) 
  {
    ArLog::log(ArLog::Terse, "ArSerialConnection::internalOpen: Serial port already open");
    return OPEN_ALREADY_OPEN;
  }

//   if (myIs422)
//     ArLog::log(ArLog::Verbose, "ArSerialConnection::internalOpen: Connecting to serial422 port '%s'", myPortName.c_str());
//   else
//     ArLog::log(ArLog::Verbose, "ArSerialConnection::internalOpen: Connecting to serial port '%s'", myPortName.c_str());

  /* open the port */
//   if (!myIs422)
//   {
// 	  if ((myPort = ArUtil::open(myPortName.c_str(),O_RDWR | O_NDELAY)) < 0)
// 	  {
// 		  ArLog::logErrorFromOS(ArLog::Terse, "ArSerialConnection::open: Could not open serial port '%s'", myPortName.c_str());
// 		  return OPEN_COULD_NOT_OPEN_PORT;
// 	  }
//   }
//   else
//   {
// 	  // PS 9/9/11 - the RDONLY worked for the s3series as it did not
// 	  // do any writes, but the sZseries needs to do writes, so changed the
// 	  // flag to RDWR
// 	  //	 if ((myPort = ArUtil::open(myPortName.c_str(),O_RDONLY | O_NOCTTY)) < 0)
// 	  if ((myPort = ArUtil::open(myPortName.c_str(),O_RDWR | O_NOCTTY)) < 0)
// 	  {
// 		  ArLog::logErrorFromOS(ArLog::Terse, "ArSerialConnection::open: Could not open serial port '%s'", myPortName.c_str());
// 		  return OPEN_COULD_NOT_OPEN_PORT;
// 	  }
//   }

  
  /* set the tty baud, buffering and modes */
  if (tcgetattr(myPort, &tio) != 0)
  {
    ArLog::logErrorFromOS(ArLog::Terse, "ArSerialConnection::open: Could not get port data to set up port");
    close();
    myStatus = STATUS_OPEN_FAILED;
    return OPEN_COULD_NOT_SET_UP_PORT;
  }    

  /* turn off echo, canonical mode, extended processing, signals */
  tio.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);

  /* turn off break sig, cr->nl, parity off, 8 bit strip, flow control */
  tio.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);

  /* clear size, turn off parity bit */
  tio.c_cflag &= ~(CSIZE | PARENB);

  /* set size to 8 bits */
  tio.c_cflag |= CS8;

  /* turn output processing off */
  tio.c_oflag &= ~(OPOST);

  /* Set time and bytes to read at once */
  tio.c_cc[VTIME] = 0;
  tio.c_cc[VMIN] = 0;

  // PS 7/3/11 - check if dev is RS422, if so then the cflags need
  // to be set different, for now test for dev/ttyS3, later we need to input that
  // the device is RS422

  // PS 9/9/11 - taking out the B38500, we now set the baud below
//   if (myIs422)
// 	  tio.c_cflag = CS8 | CLOCAL | CREAD |IGNPAR;
      //tio.c_cflag = B57600 | CS8 | CLOCAL | CREAD |IGNPAR;

  if (tcflush(myPort,TCIFLUSH) == -1)
  {
	  ArLog::logErrorFromOS(ArLog::Terse,
			  "ArSerialConnection::open: Could not set up port tcflush failed");
	  close();
	  myStatus = STATUS_OPEN_FAILED;
	  return OPEN_COULD_NOT_SET_UP_PORT;
  }

  if (tcsetattr(myPort,TCSAFLUSH,&tio) == -1)
  {
	  ArLog::logErrorFromOS(ArLog::Terse,
			  "ArSerialConnection::open: Could not set up port");
	  close();
	  myStatus = STATUS_OPEN_FAILED;
	  return OPEN_COULD_NOT_SET_UP_PORT;
  }

  myStatus = STATUS_OPEN;

  // PS 7/3/11 - only set the baud and hw control if RS232
  // for now test for dev/ttyS3, later we need to input that
  // the device is RS422

//   if (!myIs422)
//   {
// 	  if (myBaudRate != 0 && rateToBaud(myBaudRate) == -1)
// 	  {
// 		  ArLog::logErrorFromOS(ArLog::Terse, "ArSerialConnection::open: Invalid baud rate.");
// 		  close();
// 		  myStatus = STATUS_OPEN_FAILED;
// 		  return OPEN_INVALID_BAUD_RATE;
// 	  }

// 	  //printf("my baud rate = %d\n",myBaudRate);
// 	  if (myBaudRate != 0 && !setBaud(myBaudRate))
// 	  {
// 		  ArLog::log(ArLog::Terse,
// 				  "ArSerialConnection::open: Could not set baud rate.");
// 		  close();
// 		  myStatus = STATUS_OPEN_FAILED;
// 		  return OPEN_COULD_NOT_SET_BAUD;
// 	  }

// 	  if (!setHardwareControl(myHardwareControl))
// 	  {
// 		  ArLog::log(ArLog::Terse,
// 				  "ArSerialConnection::open: Could not set hardware control.");
// 		  close();
// 		  myStatus = STATUS_OPEN_FAILED;
// 		  return OPEN_COULD_NOT_SET_UP_PORT;
// 	  }
//   }
//   // PS 9/9/11 - added else for SZ
//   else
//   {
// 	  if (myBaudRate != 0 && rateToBaud(myBaudRate) == -1)
// 	  {
// 		  ArLog::logErrorFromOS(ArLog::Terse, "ArSerialConnection::open: Invalid baud rate.");
// 		  close();
// 		  myStatus = STATUS_OPEN_FAILED;
// 		  return OPEN_INVALID_BAUD_RATE;
// 	  }

// 	  //printf("my baud rate = %d\n",myBaudRate);
// 	  if (myBaudRate != 0 && !setBaud(myBaudRate))
// 	  {
// 		  ArLog::log(ArLog::Terse,
// 				  "ArSerialConnection::open: Could not set baud rate.");
// 		  close();
// 		  myStatus = STATUS_OPEN_FAILED;
// 		  return OPEN_COULD_NOT_SET_BAUD;
// 	  }
//   }

//   ArLog::log(ArLog::Verbose, "ArSerialConnection::open: Successfully opened and configured serial port '%s'.", myPortName.c_str());
  ArLog::log(ArLog::Verbose, "ArSerialConnection::open: Successfully opened and configured serial port '%s'.", myPortName);
  return 0;
}

// AREXPORT bool ArSerialConnection::openSimple(void)
// {
// //   if (internalOpen() == 0)
// //     return true;
// //   else
// //     return false;
//     return open();
// }

/**
   @param port The serial port to connect to, or NULL which defaults to 
   COM1 for windows and /dev/ttyS0 for linux
   @see getOpenMessage
*/
// AREXPORT void ArSerialConnection::setPort(const char *port)
// {
//   if (port == NULL)
//   {
//     // myPortName = "/dev/ttyS0";
//     strncpy(myPortName, "/dev/tty0", sizeof(myPortName)-1);
//     myPortName[sizeof(myPortName)-1] = '\0';
//   }
//   else
//   {
//     // myPortName = port;
//     strncpy(myPortName, port, sizeof(myPortName)-1);
//     myPortName[sizeof(myPortName)-1] = '\0';
// //   setPortName(myPortName.c_str());
//   }   
//   setPortName(myPortName);
// }

AREXPORT void ArSerialConnection::setPort(const char *port)
{
  const char* defaultPort = "/dev/tty0";
  
  // Use the provided port or default if NULL
  const char* portToUse = (port != NULL) ? port : defaultPort;
  
  // Safely copy the port name
  strncpy(myPortName, portToUse, sizeof(myPortName) - 1);
  myPortName[sizeof(myPortName) - 1] = '\0';
  
  setPortName(myPortName);
}
/**
   @return The serial port to connect to
**/
AREXPORT const char * ArSerialConnection::getPort(void)
{
//   return myPortName.c_str();
  return myPortName;
}

/**
   @param port The serial port to connect to, or NULL which defaults to 
   COM1 for windows and /dev/ttyS0 for linux
   @return 0 for success, otherwise one of the open enums
   @see getOpenMessage
*/
// AREXPORT int ArSerialConnection::open(const char *port)
// {
//   setPort(port);
//   return internalOpen();
// }

// AREXPORT bool ArSerialConnection::open(const char *port, int baud)
// {
//     if (port != NULL) {
//         strncpy(myPortName, port, sizeof(myPortName)-1);
//         myPortName[sizeof(myPortName)-1] = '\0';
//         myPortNameStr = port;
//     }
    
//     if (baud != -1)
//         myBaud = baud;

//     return internalOpen();
// }

AREXPORT bool ArSerialConnection::open(const char *port, int baud)
{
    if (port != NULL) 
    {
        strncpy(myPortName, port, sizeof(myPortName)-1);
        myPortName[sizeof(myPortName)-1] = '\0';
    }
    
    if (baud != -1)
        myBaud = baud;

    // if (myStatus == STATUS_OPEN) {
    //     ArLog::log(ArLog::Terse, 
    //         "ArSerialConnection::open: Serial port already open");
    //     return false;
    // }

    // // Open serial port
    // if ((myPort = ::open(myPortName, O_RDWR | O_NOCTTY | O_NONBLOCK)) < 0) {
    //     ArLog::log(ArLog::Terse,
    //         "ArSerialConnection::open: Could not open serial port '%s'", myPortName);
    //     return false;
    // }

    // // Set proper baud rate
    // struct termios tio;
    // if (tcgetattr(myPort, &tio) != 0) {
    //     ArLog::log(ArLog::Terse, 
    //         "ArSerialConnection::open: Error getting serial port attributes");
    //     ::close(myPort);
    //     return false;
    // }

    // // Success log
    // // ArLog::log(ArLog::Verbose, 
    // //     "ArSerialConnection::open: Successfully opened port '%s'", myPortName);
    // // myStatus = STATUS_OPEN;
    // ArLog::log(ArLog::Verbose, 
    //     "ArSerialConnection::open: Successfully opened and configured serial port '%s'.", 
    //     myPortName);

    // return true;
    return internalOpen();
}

// AREXPORT const char *ArSerialConnection::getPortName(void) const
// // AREXPORT const char *ArSerialConnection::getPortName(void) 
// // AREXPORT const char *ArSerialConnection::getPortName() const
// // AREXPORT const char *ArSerialConnection::getPortName()
// {
//   return myPortName;
// }

// void ArSerialConnection::setPortName(const char *port)
// {
//     if (port != NULL)
//     {
//         strncpy(myPortName, port, sizeof(myPortName)-1);
//         myPortName[sizeof(myPortName)-1] = '\0';
//     }
// }

AREXPORT bool ArSerialConnection::close(void)
{
  int ret;

  myStatus = STATUS_CLOSED_NORMALLY;
  if (myPort == -1)
    return true;
  

  ret = ::close(myPort);

  if (ret == 0)
    ArLog::log(ArLog::Verbose,
	       "ArSerialConnection::close: Successfully closed serial port.");
  else
    ArLog::logErrorFromOS(ArLog::Verbose, 
	       "ArSerialConnection::close: Unsuccessfully closed serial port.");

  myPort = -1;
  if (ret == 0)
    return true;
  else
    return false;
}

AREXPORT bool ArSerialConnection::setBaud(int rate)
{
  struct termios tio;  
  int baud;

  myBaud = rate;
  
  if (getStatus() != STATUS_OPEN)
    return true;

  if (myBaud == 0)
    return true;

//   if ((baud = rateToBaud(myBaudR)) == -1)
//     return false;
  if ((baud = rateToBaud(myBaud)) == -1)  // Use myBaud not myBaudR
  {
    ArLog::log(ArLog::Terse, "ArSerialConnection::open: Invalid baud rate.");
    return false;
 }
  
  if (tcgetattr(myPort, &tio) != 0)
  {
    ArLog::logErrorFromOS(ArLog::Terse, "ArSerialConnection::setBaud: Could not get port data.");
    return false;
  }
  
  if (cfsetospeed(&tio, baud)) 
  {
    ArLog::logErrorFromOS(ArLog::Terse, "ArSerialConnection::setBaud: Could not set output baud rate on termios struct.");
    return false;
  }
       
  if (cfsetispeed(&tio, baud)) 
  {
    ArLog::logErrorFromOS(ArLog::Terse, "ArSerialConnection::setBaud: Could not set input baud rate on termios struct.");
    return false;
  }

  if(tcsetattr(myPort,TCSAFLUSH,&tio) < 0) 
  {
    ArLog::logErrorFromOS(ArLog::Terse, "ArSerialConnection::setBaud: Could not set baud rate.");
    return false;
  }

  startTimeStamping();
  
  return true;
}

AREXPORT void ArSerialConnection::startTimeStamping(void)
{
  long baud;
  baud = myBaud;
  if (ioctl(myPort, TIOSTARTTIMESTAMP, &baud) != 0)
    myTakingTimeStamps = false;
  else
    myTakingTimeStamps = true;
}

/**
   @return the current baud rate of the connection
*/

AREXPORT int ArSerialConnection::getBaud(void)
{
  return myBaud;
}

int ArSerialConnection::rateToBaud(int rate)
{
  switch (rate) {
  case 300: return B300;
  case 1200: return B1200;
  case 1800: return B1800;
  case 2400: return B2400;
  case 4800: return B4800;
  case 9600: return B9600;
  case 19200: return B19200;
  case 38400: return B38400;
  case 57600: return B57600;
  case 115200: return B115200;
  case 230400: return B230400;
  case 460800: return B460800;
  default: 
    ArLog::log(ArLog::Terse, "ArSerialConnection::rateToBaud: Did not know baud for rate %d.", rate);
    return -1;
  }
}

int ArSerialConnection::baudToRate(int baud)
{
  switch (baud) {
  case B300: return 300;
  case B1200: return 1200;
  case B1800: return 1800;
  case B2400: return 2400;
  case B4800: return 4800;
  case B9600: return 9600;
  case B19200: return 19200;
  case B38400: return 38400;
  case B57600: return 57600;
  case B115200: return 115200;
  case B230400: return 230400;
  case B460800: return 460800;
  default: 
    ArLog::log(ArLog::Terse, "ArSerialConnection:baudToRate: Did not know rate for baud.");
    return -1;
  }
  
}

/**
   @param hardwareControl true to enable hardware control of lines
   @return true if the set succeeded
*/

AREXPORT bool ArSerialConnection::setHardwareControl(bool hardwareControl)
{
  struct termios tio;

  myHardwareControl = hardwareControl;

  if (getStatus() != STATUS_OPEN) 
    return true;

  //tcgetattr(myPort, &tio);

  if (tcgetattr(myPort, &tio) != 0)
  {
    ArLog::logErrorFromOS(ArLog::Terse, "ArSerialConnection::setHardwareControl: Could not get port data.");
    return false;
  }


  /* check for hardware flow control */
  if (myHardwareControl)
    tio.c_cflag |= CRTSCTS;
  else
    tio.c_cflag &= ~CRTSCTS;
      
  if(tcsetattr(myPort,TCSAFLUSH,&tio) < 0) {
    ArLog::logErrorFromOS(ArLog::Terse, "ArSerialConnection::setHardwareControl: Could not set hardware control.");
    return false;
  }
  
  return true;
}

/**
   @return true if hardware control of lines is enabled, false otherwise
*/
AREXPORT bool ArSerialConnection::getHardwareControl(void)
{
  return myHardwareControl;
}

AREXPORT int ArSerialConnection::write(const char *data, unsigned int size) 
{
  int n;
  /*
  printf("SERIAL_WRITE(%3d): ", size);
  for (int i = 0; i < size; i++)
    printf("0x%x %c", data[i], data[i]);
  printf("\n");
  */
  
  /* 
  char buf[10000];
  sprintf(buf, "SERIAL_WRITE(%3d bytes %d): ", size, myPort);
  for (int i = 0; i < size; i++)
    sprintf(buf, "%s %02x", buf, (unsigned char)data[i]);
  ArLog::log(ArLog::Normal, buf);
  */

  if (myPort >= 0) 
  {
    n = ::write(myPort, data, size);
    if (n == -1) 
    {
#if 0
      if (errno == EAGAIN)   /* try it again, for USB/serial */
	{
	  usleep(10);
	  n = ::write(myPort, data, size);
	  if (n >= 0)
	    return n;
	}
#endif 
      ArLog::logErrorFromOS(ArLog::Terse, "ArSerialConnection::write: Error on writing.");
    }
    return n;
  }
  ArLog::log(ArLog::Terse, "ArSerialConnection::write: Connection invalid.");
  return -1;
}

AREXPORT int ArSerialConnection::read(const char *data, unsigned int size,
				      unsigned int msWait) 
{
  struct timeval tp;		/* time interval structure for timeout */
  fd_set fdset;			/* fd set ??? */
  int n;
  long timeLeft;
  unsigned int bytesRead = 0;
  ArTime timeDone;

  if (myPort >= 0)
  {
    if (msWait >= 0)
    {
      timeDone.setToNow();
      if (!timeDone.addMSec(msWait)) {
        ArLog::log(ArLog::Normal,
                   "ArSerialConnection::read() error adding msecs (%i)",
                   msWait);
      }
      while ((timeLeft = timeDone.mSecTo()) >= 0) 
      {
	tp.tv_sec = (timeLeft) / 1000;	/* we're polling */
	tp.tv_usec = (timeLeft % 1000) * 1000;
	FD_ZERO(&fdset);
	FD_SET(myPort,&fdset);
	if (select(myPort+1,&fdset,NULL,NULL,&tp) <= 0) 
	  return bytesRead;
	if ((n = ::read(myPort, const_cast<char *>(data)+bytesRead, 
			size-bytesRead)) == -1)
	{
	  ArLog::logErrorFromOS(ArLog::Terse, "ArSerialConnection::read:  Blocking read failed.");
	  return bytesRead;
	}
	bytesRead += n;
	if (bytesRead >= size)
	  return bytesRead;
      }
      return bytesRead;
    }
    else 
    {
      n = ::read(myPort, const_cast<char *>(data), size);
      if (n == -1)
	ArLog::logErrorFromOS(ArLog::Terse, "ArSerialConnection::read:  Non-Blocking read failed.");
      return n;
    }
  }
  ArLog::log(ArLog::Normal, "ArSerialConnection::read:  Connection invalid.");
  return -1;
}

AREXPORT int ArSerialConnection::getStatus(void)
{
  return myStatus;
}

AREXPORT ArTime ArSerialConnection::getTimeRead(int index)
{
  ArTime ret;
  struct timeval timeStamp;
  if (myPort <= 0)
  {
    ret.setToNow();
    return ret;
  }

  if (myTakingTimeStamps)
  {
    timeStamp.tv_sec = index;
    if (ioctl(myPort, TIOGETTIMESTAMP, &timeStamp) == 0)
    {
      ret.setSec(timeStamp.tv_sec);
      ret.setMSec(timeStamp.tv_usec / 1000);
    }
    else
      ret.setToNow();
  }
  else
    ret.setToNow();

  return ret;
}

AREXPORT bool ArSerialConnection::isTimeStamping(void)
{
  return myTakingTimeStamps;
}


AREXPORT bool ArSerialConnection::getCTS(void)
{
  unsigned int value;
  if (ioctl(myPort, TIOCMGET, &value) == 0)
  {
    return (bool) (value & TIOCM_CTS);
  }
  else
  {
    ArLog::logErrorFromOS(ArLog::Normal, "ArSerialConnection::getCTS: ioctl(TIOCMGET)");
    return false;
  }
}

AREXPORT bool ArSerialConnection::getDSR(void)
{
  unsigned int value;
  if (ioctl(myPort, TIOCMGET, &value) == 0)
  {
    return (bool) (value & TIOCM_DSR);
  }
  else
  {
    ArLog::logErrorFromOS(ArLog::Normal, "ArSerialConnection::getDSR: ioctl(TIOCMGET)");
    return false;
  }
}

AREXPORT bool ArSerialConnection::getDCD(void)
{
  unsigned int value;
  if (ioctl(myPort, TIOCMGET, &value) == 0)
  {
    return (bool) (value & TIOCM_CAR);
  }
  else
  {
    ArLog::logErrorFromOS(ArLog::Normal, "ArSerialConnection::getDCD: ioctl(TIOCMGET)");
    return false;
  }
}

AREXPORT bool ArSerialConnection::getRing(void)
{
  unsigned int value;
  if (ioctl(myPort, TIOCMGET, &value) == 0)
  {
    return (bool) (value & TIOCM_RI);
  }
  else
  {
    ArLog::logErrorFromOS(ArLog::Normal, "ArSerialConnection::getRing: ioctl(TIOCMGET)");
    return false;
  }
}
