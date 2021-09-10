/*
***************************************************************************
*
* Author: Teunis van Beelen
*
* Copyright (C) 2005 - 2020 Teunis van Beelen
*
* Email: teuniz@protonmail.com
*
***************************************************************************
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
***************************************************************************
*/


/* Last revision: August 6, 2020 */
/* Solved a problem related to FreeBSD and some baudrates not recognized. */
/* For more info and how to use this library, visit: http://www.teuniz.net/RS-232/ */


#include "rs232.h"


static int baudrate = 57600;
int fd = -1;

#ifndef _WIN32   /* Linux & FreeBSD */
static int error = 0;

static struct termios new_port_settings, old_port_settings;

int RS232_SetupPort(int bauds, const char *mode, int flowctrl)
{
  int baudr,
      status;
   baudrate = bauds;
  switch(baudrate)
  {
    case      50 : baudr = B50;
                   break;
    case      75 : baudr = B75;
                   break;
    case     110 : baudr = B110;
                   break;
    case     134 : baudr = B134;
                   break;
    case     150 : baudr = B150;
                   break;
    case     200 : baudr = B200;
                   break;
    case     300 : baudr = B300;
                   break;
    case     600 : baudr = B600;
                   break;
    case    1200 : baudr = B1200;
                   break;
    case    1800 : baudr = B1800;
                   break;
    case    2400 : baudr = B2400;
                   break;
    case    4800 : baudr = B4800;
                   break;
    case    9600 : baudr = B9600;
                   break;
    case   19200 : baudr = B19200;
                   break;
    case   38400 : baudr = B38400;
                   break;
    case   57600 : baudr = B57600;
                   break;
    case  115200 : baudr = B115200;
                   break;
    case  230400 : baudr = B230400;
                   break;
    case  460800 : baudr = B460800;
                   break;
#if defined(__linux__)
    case  500000 : baudr = B500000;
                   break;
    case  576000 : baudr = B576000;
                   break;
    case  921600 : baudr = B921600;
                   break;
    case 1000000 : baudr = B1000000;
                   break;
    case 1152000 : baudr = B1152000;
                   break;
    case 1500000 : baudr = B1500000;
                   break;
    case 2000000 : baudr = B2000000;
                   break;
    case 2500000 : baudr = B2500000;
                   break;
    case 3000000 : baudr = B3000000;
                   break;
    case 3500000 : baudr = B3500000;
                   break;
    case 4000000 : baudr = B4000000;
                   break;
#endif
    default      : printf("invalid baudrate\n");
                   return 1;
  }

  int cbits=CS8,
      cpar=0,
      ipar=IGNPAR,
      bstop=0;

  if(strlen(mode) != 3)
  {
    printf("invalid mode \"%s\"\n", mode);
    return 1;
  }

  switch(mode[0])
  {
    case '8': cbits = CS8;
              break;
    case '7': cbits = CS7;
              break;
    case '6': cbits = CS6;
              break;
    case '5': cbits = CS5;
              break;
    default : printf("invalid number of data-bits '%c'\n", mode[0]);
              return 1;
  }

  switch(mode[1])
  {
    case 'N':
    case 'n': cpar = 0;
              ipar = IGNPAR;
              break;
    case 'E':
    case 'e': cpar = PARENB;
              ipar = INPCK;
              break;
    case 'O':
    case 'o': cpar = (PARENB | PARODD);
              ipar = INPCK;
              break;
    default : printf("invalid parity '%c'\n", mode[1]);
              return 1;
  }

  switch(mode[2])
  {
    case '1': bstop = 0;
              break;
    case '2': bstop = CSTOPB;
              break;
    default : printf("invalid number of stop bits '%c'\n", mode[2]);
              return 1;
  }

  error = tcgetattr(fd, &old_port_settings);
  if(error==-1)
  {
    fprintf(stderr, "unable to read portsettings \n");
    return 1;
  }
  memset(&new_port_settings, 0, sizeof(new_port_settings));  /* clear the new struct */

  new_port_settings.c_cflag = (tcflag_t)(cbits | cpar | bstop | CLOCAL | CREAD);
  if(flowctrl)
  {
    new_port_settings.c_cflag |= CRTSCTS;
  }
  new_port_settings.c_iflag = (tcflag_t)ipar;
  new_port_settings.c_oflag = 0;
  new_port_settings.c_lflag = 0;
  new_port_settings.c_cc[VMIN] = 0;      /* block untill n bytes are received */
  new_port_settings.c_cc[VTIME] = 0;     /* block untill a timer expires (n * 100 mSec.) */

  cfsetispeed(&new_port_settings, (speed_t)baudr);
  cfsetospeed(&new_port_settings, (speed_t)baudr);

  error = tcsetattr(fd, TCSANOW, &new_port_settings);
  if(error==-1)
  {
    tcsetattr(fd, TCSANOW, &old_port_settings);
    fprintf(stderr, "unable to adjust portsettings \n");
    return 1;
  }

/* http://man7.org/linux/man-pages/man4/tty_ioctl.4.html */

  if(ioctl(fd, TIOCMGET, &status) == -1)
  {
    tcsetattr(fd, TCSANOW, &old_port_settings);
    fprintf(stderr, "unable to get portstatus\n");
    return 1;
  }

  status |= TIOCM_DTR;    /* turn on DTR */
  status |= TIOCM_RTS;    /* turn on RTS */

  if(ioctl(fd, TIOCMSET, &status) == -1)
  {
    tcsetattr(fd, TCSANOW, &old_port_settings);
    fprintf(stderr, "unable to set portstatus\n");
    return 1;
  }

  return 0;
}

void RS232_flushRX()
{
  tcflush(fd, TCIFLUSH);
}


void RS232_flushTX()
{
  tcflush(fd, TCOFLUSH);
}


void RS232_flushRXTX()
{
  tcflush(fd, TCIOFLUSH);
}

#else

int RS232_SetupPort(int bauds, const char *mode, int flowctrl)
{
    baudrate = bauds;
    HANDLE pHandle = (HANDLE)_get_osfhandle(fd);
/*
http://msdn.microsoft.com/en-us/library/windows/desktop/aa363145%28v=vs.85%29.aspx

http://technet.microsoft.com/en-us/library/cc732236.aspx

https://docs.microsoft.com/en-us/windows/desktop/api/winbase/ns-winbase-_dcb
*/

  COMMTIMEOUTS Cptimeouts;

  Cptimeouts.ReadIntervalTimeout         = MAXDWORD;
  Cptimeouts.ReadTotalTimeoutMultiplier  = 0;
  Cptimeouts.ReadTotalTimeoutConstant    = 0;
  Cptimeouts.WriteTotalTimeoutMultiplier = 0;
  Cptimeouts.WriteTotalTimeoutConstant   = 0;

  if(!SetCommTimeouts(pHandle, &Cptimeouts))
  {
    printf("unable to set comport time-out settings\n");
    return 1;
  }

  DCB port_settings;
  memset(&port_settings, 0, sizeof(port_settings));  /* clear the new struct  */
  port_settings.DCBlength = sizeof(port_settings);

  if(!GetCommState(pHandle, &port_settings))
  {
    printf("unable to set comport cfg settings\n");
    return 1;
  }

  port_settings.BaudRate = baudrate;
  port_settings.Parity = (tolower(mode[1]) == 'n') ? NOPARITY : (tolower(mode[1]) == 'o' ? ODDPARITY : EVENPARITY);
  port_settings.StopBits = (mode[2] == '2') ? TWOSTOPBITS : ONESTOPBIT;
  switch(mode[0]) {
    case '5': port_settings.ByteSize = DATABITS_5; break;
    case '6': port_settings.ByteSize = DATABITS_6; break;
    case '7': port_settings.ByteSize = DATABITS_7; break;
    case '8': port_settings.ByteSize = DATABITS_8; break;
  default:
      fprintf(stderr, "unable to set comport cfg settings\n");
      return 1;
  }

  if(flowctrl)
  {
    port_settings.fOutxCtsFlow = TRUE;
    port_settings.fDtrControl = DTR_CONTROL_HANDSHAKE;
    port_settings.fRtsControl = RTS_CONTROL_HANDSHAKE;
  }

  if(!SetCommState(pHandle, &port_settings))
  {
    fprintf(stderr, "unable to set comport cfg settings\n");
    return 1;
  }

  return 0;
}

/*
https://msdn.microsoft.com/en-us/library/windows/desktop/aa363428%28v=vs.85%29.aspx
*/

void RS232_flushRX()
{
    HANDLE pHandle = (HANDLE)_get_osfhandle(fd);
  PurgeComm(pHandle, PURGE_RXCLEAR | PURGE_RXABORT);
}


void RS232_flushTX()
{
    HANDLE pHandle = (HANDLE)_get_osfhandle(fd);
  PurgeComm(pHandle, PURGE_TXCLEAR | PURGE_TXABORT);
}


void RS232_flushRXTX()
{
    HANDLE pHandle = (HANDLE)_get_osfhandle(fd);
  PurgeComm(pHandle, PURGE_RXCLEAR | PURGE_RXABORT);
  PurgeComm(pHandle, PURGE_TXCLEAR | PURGE_TXABORT);
}

#endif

int RS232_OpenComport(const char* devname)
{
    char dev_name[128];
#ifdef _WIN32
    sprintf(dev_name, "\\\\.\\%s", devname);
#else
    sprintf(dev_name, "%s", devname);
#endif
    fd = open(dev_name, O_RDWR);

    if(fd==-1)
    {
      fprintf(stderr, "unable to setup comport\n");
      return 1;
    }

    return 0;
}

void RS232_CloseComport()
{
    close(fd);
}

int RS232_AlignFrame(int sof, int maxtries)
{
    int n;
    int c = 0;
    RS232_flushRX();
    while(c != sof && maxtries-- > 0) {
        n = RS232_PollComport((char*)&c, 1);
        if(n<0) {
          if(errno == EAGAIN)
              continue;
          else
              return 1;
        }
    }
    return 0;
}

int RS232_PollComport(char *buf, int size)
{
    int nread = 0;
    int ntries = size;
    int to_read = size;
    ssize_t n;
#ifdef _WIN32
    LPWSTR tmp = (LPWSTR)malloc(sizeof(WCHAR)*size);
#else
    char* tmp = buf;
#endif
    while(to_read > 0 && ntries-->0) {
        usleep(10000000*to_read/(unsigned)baudrate);
        n = read(fd, tmp+nread, (size_t)to_read);
        if(n<0) {
          if(errno == EAGAIN)
              return nread;
          else
              return (int)n;
        }
        nread += n;
        to_read -= n;
    }
#ifdef _WIN32
    WideCharToMultiByte(
      CP_ACP,
      WC_NO_BEST_FIT_CHARS,
      tmp,
      -1,
      (LPSTR)buf,
      nread,
      NULL,
      FALSE
    );
  free(tmp);
#endif
    return nread;
}


int RS232_SendByte(unsigned char byte)
{
#ifdef _WIN32
    LPWSTR buf = (LPWSTR)malloc(sizeof(WCHAR)*1);
    MultiByteToWideChar(
    CP_ACP,
    MB_PRECOMPOSED,
    (LPSTR)&byte,
    1,
    buf,
    1
  );
#else
    char* buf = &byte;
#endif
  ssize_t n = write(fd, buf, (int)1);
  usleep(10000000/(unsigned)baudrate);
  if(n < 1)
  {
      return 1;
  }
#ifdef _WIN32
  free(buf);
#endif
  return 0;
}


int RS232_SendBuf(unsigned char *buf, int size)
{
#ifdef _WIN32
    LPWSTR tmp = (LPWSTR)malloc(sizeof(WCHAR)*size);
    MultiByteToWideChar(
    CP_ACP,
    MB_PRECOMPOSED,
    (LPSTR)buf,
    1,
    tmp,
    1
  );
#else
    char* tmp = buf;
#endif
  ssize_t n = write(fd, tmp, (size_t)size);
  if(n < 0)
  {
    if(errno == EAGAIN)
    {
      return 0;
    }
    else
    {
      return -1;
    }
  }
#ifdef _WIN32
  free(tmp);
#endif
  return((int)n);
}

void RS232_SetFD(int f)
{
    fd = f;
}

void RS232_cputs(const char *text)  /* sends a string to serial port */
{
  while(*text != 0)   RS232_SendByte(*((const unsigned char*)text++));
}
