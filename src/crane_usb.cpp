#include <stdio.h>
#include <err.h>
#include <math.h>
#include <iostream>
#include <ftdi.h>

#include <sys/time.h>

#include "crane_usb.hh"


/* --- local data ---------------------------------------------------------- */

static struct ftdi_context *ftdic;
//static struct MsgStruct CrSensor;

/* --- connect ------------------------------------------------------------- */

int
ptp_connect(void)
{
  int s;

  ftdic = ftdi_new();
  if (!ftdic) {
    warnx("no usb devices");
    return 1;
  }

  s = ftdi_usb_open_desc(ftdic,
			 USB_VENDOR_FTDI, USB_PRODUCT_SERIAL, NULL, NULL);
  if (s) {
    warnx("ftdi_usb_open_desc: %s", ftdi_get_error_string(ftdic));
    ftdi_free(ftdic);
    ftdic = NULL;
    warnx("no petipa usb device");
    return 1;
  }

  ftdi_usb_purge_buffers(ftdic);
  warnx("found petipa usb device");
  return 0;
}


/* --- disconnect ---------------------------------------------------------- */

void
ptp_disconnect(void)
{
  if (!ftdic) return;

  ftdi_usb_close(ftdic);
  ftdi_free(ftdic);
  ftdic = NULL;
}

void ptp_reconnect_until_ok_or_k_demands(unsigned int k)
{
  // std::cerr << "Disconnect crane" << std::endl; 
  struct timeval begin,end;
  gettimeofday(&begin,0);

  bool ok=true;
  unsigned int count=0;
  int r=0;
  do 
    {
      ok=true;
      ptp_disconnect();
      usleep(1000.0);
      r=ptp_connect();
      if (r!=0)
	{
	  count ++;
	  if (count<k)
	    ok=false;
	}
    }
  while(!ok);

  gettimeofday(&end,0);
  std::cerr<< "Time to reconnect:" << end.tv_sec - begin.tv_sec 
    + 0.000001 * (end.tv_usec - begin.tv_usec) << std::endl;
  /*
  if (r)
    std::cerr<< "Could not reconnect crane after " << count << " trials" 
	     << " limit of trials: " << k << std::endl;
  else
    std::cerr<< "Reconnect crane" << std::endl;
  */
}




/* --- ptp_control --------------------------------------------------------- */

/** Send a speed reference (X Y Z, m/s). s[] should contain 3 booleans (for X,
 * Y, Z). True means "control the axis with v[i]", false means "manual mode
 * (remote controller)".
 */
int ptp_control(double v[3], int s[3])
{
  static const int factor[3] = { 0x8000, 0x8000, 0x3 };
  static unsigned char buffer[32];
  unsigned char *p;
  double sv[3];
  int i, e, w;
  
  p = buffer;

  /* scale velocities */
  for(i=0; i<3; i++) {
    if (fabs(v[i]) < 4./60.)
      sv[i] = 0.;
    else if (v[i] > 0.)
      sv[i] = v[i] / (32./60.) - 4./60;
    else
      sv[i] = v[i] / (32./60.) + 4./60;
  }

  /* header */
  *(p++) = 0x10;

  /* velocities */
  for(i=0; i<3; i++) {
    if (s[i]) w = sv[i] * factor[i]; else w = 0;
    if (w >= factor[i]) w = factor[i] - 1;
    if (w <= -factor[i]) w = -factor[i] + 1;

    *(p++) = (w >> 8); if (*(p-1) == 0x10) { *(p++) = 0x10; }
    *(p++) = (w & 0xff); if (*(p-1) == 0x10) { *(p++) = 0x10; }
  }

  /* axis selector */
  *(p++) = 0;
  *p = 0;
  for(i=0; i<3; i++) if (s[i]) *p |= (1<<i);
  p++; if (*(p-1) == 0x10) { *(p++) = 0x10; }

  /* trailer */
  *(p++) = 0x10;
  *(p++) = 0x03;

  if (!ftdic) return -1;
  e = ftdi_write_data(ftdic, buffer, p-buffer);
  if (e < 0) {
    warnx("ftdi_write_data: %s", ftdi_get_error_string(ftdic));
    return 1;
  }

  return 0;
}

int ptp_recontrol(double v[3],
		   int s[3],
		   unsigned int k)
{
  bool ok=true;
  unsigned int count=0;
  int r=0;
  do {
    ok=true;
    usleep(1000.0);
    r=ptp_control(v,s);
    if (r!=0) {
      count ++;
      if (count<k)
	ok=false;
    }
  }
  while(!ok);

  return r;
}

int ptp_read_encoders(unsigned int *Label, double Position[3], double Velocity[3]) {
  int FtdiReturn = 0;
  unsigned int BufferIdx = 0, CntComplet = 0, i = 0;
  static enum MsgStep MsgCrStep = DLE_HEADER, MsgNxStep = DLE_HEADER;
  static struct MsgStruct NxSensor;
  static unsigned char SensorIdx = 0;
  unsigned char Buffer[SIZE_IOCTL_MSG];
  unsigned char Debug = 1;
  static bool DleDouble = false;
  bool MsgComplet = false;
    
  /* Read & test USB link */
  if (!ftdic)
    return -1;
  
  FtdiReturn = ftdi_read_data(ftdic, Buffer, SIZE_IOCTL_MSG);
  if (FtdiReturn < 0) {
    warnx("ftdi_read_data: %s", ftdi_get_error_string(ftdic));
    return 1;
  }

  if ((FtdiReturn == 0) || (FtdiReturn < SIZE_ENCODER_MSG))
    return 0;
  
  while (BufferIdx < (unsigned int)FtdiReturn) {
    /* Extract message */
    switch(MsgCrStep) {
    case DLE_HEADER :
      for (i = 0; i < SENSOR_NB; i++)
	NxSensor.Sensor_Msg[i] = 0;
      DleDouble = false;
      MsgNxStep = (Buffer[BufferIdx] == DLE) ? (ADDRESS_ID) : (DLE_HEADER);
      break;
    case ADDRESS_ID :
      MsgNxStep = (Buffer[BufferIdx] == ADDR_TX) ? (LABEL_MSB) : (DLE_HEADER);
      break;
    case LABEL_MSB :
      if((!DleDouble && Buffer[BufferIdx] != DLE) || (DleDouble && Buffer[BufferIdx] == DLE)) {
    	NxSensor.Sensor_Label   = (unsigned int)Buffer[BufferIdx];
	NxSensor.Sensor_Label <<= 8;
    	DleDouble = false;
    	MsgNxStep = LABEL_LSB;
      }
      else if (Buffer[BufferIdx] == DLE) 
	DleDouble = true;
      else
	MsgNxStep = DLE_HEADER;      
      break;
    case LABEL_LSB :
      if((!DleDouble && Buffer[BufferIdx] != DLE) || (DleDouble && Buffer[BufferIdx] == DLE)) {
    	NxSensor.Sensor_Label |= (unsigned int)Buffer[BufferIdx];
    	DleDouble = false;
    	SensorIdx = 0;
    	MsgNxStep = SENSOR_BYTE_0;
      }
      else if (Buffer[BufferIdx] == DLE) 
	DleDouble = true;
      else
	MsgNxStep = DLE_HEADER;      
      break;
    case SENSOR_BYTE_0 :
    case SENSOR_BYTE_1 :
    case SENSOR_BYTE_2 :
    case SENSOR_BYTE_3 :
      if((!DleDouble && Buffer[BufferIdx] != DLE) || (DleDouble && Buffer[BufferIdx] == DLE)) {
	NxSensor.Sensor_Msg[SensorIdx]  |= (unsigned int)Buffer[BufferIdx];
	NxSensor.Sensor_Msg[SensorIdx] <<= ((MsgCrStep == SENSOR_BYTE_3) ? (0) : (8));
       	DleDouble = false;
	SensorIdx = (MsgCrStep == SENSOR_BYTE_3) ? (SensorIdx + 1) : (SensorIdx);
	MsgNxStep = (MsgCrStep == SENSOR_BYTE_3 && SensorIdx < SENSOR_NB) ? (SENSOR_BYTE_0) : (DLE_FOOTER);
	MsgNxStep = (MsgCrStep == SENSOR_BYTE_2) ? (SENSOR_BYTE_3) : (MsgNxStep);
	MsgNxStep = (MsgCrStep == SENSOR_BYTE_1) ? (SENSOR_BYTE_2) : (MsgNxStep);
	MsgNxStep = (MsgCrStep == SENSOR_BYTE_0) ? (SENSOR_BYTE_1) : (MsgNxStep);
      }
      else if (Buffer[BufferIdx] == DLE) 
	DleDouble = true;
      else
	MsgNxStep = DLE_HEADER;      
      break;
    case DLE_FOOTER :
      MsgNxStep = (Buffer[BufferIdx] == DLE) ? (ETX_FOOTER) : (DLE_HEADER);
      break;
    case ETX_FOOTER :
      MsgComplet = (Buffer[BufferIdx] == ETX) ? (true) : (false);
      MsgNxStep = DLE_HEADER;
    }
    BufferIdx++;
    MsgCrStep = MsgNxStep;
    if (MsgComplet) {
      CntComplet++;
      for (i = 0; i < SENSOR_NB; i++) {
	NxSensor.Sensor_Position[i] = (NxSensor.Sensor_Msg[i] & POSITION_MASK) >> POSITION_SHIFT;
	NxSensor.Sensor_Velocity[i] = (NxSensor.Sensor_Msg[i] & VELOCITY_MASK) >> VELOCITY_SHIFT;
	NxSensor.Sensor_NoPos[i]    = (NxSensor.Sensor_Msg[i] & NO_POS_MASK)   >> NO_POS_SHIFT;
	NxSensor.Sensor_Warn[i]     = (NxSensor.Sensor_Msg[i] & WARNING_MASK)  >> WARN_SHIFT;
	NxSensor.Sensor_Error[i]    = (NxSensor.Sensor_Msg[i] & ERROR_MASK)    >> ERROR_SHIFT;
      }
      MsgComplet = false;
    }
  }
  // END LOOP
  *Label = (CntComplet) ? (NxSensor.Sensor_Label) : (0.0);
  for (i = 0; i < SENSOR_NB; i++) {
    Position[i] = (CntComplet) ? ((double)NxSensor.Sensor_Position[i]) : (0.0);
    Velocity[i] = (CntComplet) ? ((double)NxSensor.Sensor_Velocity[i]) : (0.0);
  }
  // Print header.
  if (Debug > LEVEL_FOR_DISPLAYING_ENCODERS && CntComplet)
    printf("%5d, %5d, %5d, %5d\n", NxSensor.Sensor_Label, NxSensor.Sensor_Position[0], 
	   NxSensor.Sensor_Position[1], NxSensor.Sensor_Position[2]);
  CntComplet = 0;
  return 0;
}

int ptp_reread_encoders(unsigned int *Label, double Position[3], double Velocity[3], unsigned int k) {
  bool ok=true;
  unsigned int count=0;
  int r=0;
  do {
    ok=true;
    usleep(1000.0);
    r=ptp_read_encoders(Label, Position, Velocity);
    if (r!=0) {
      count ++;
      if (count<k)
	ok=false;
    }
  }
  while(!ok);
  return r;
}


/*
  Current limits of the crane:
  X-> length: from 0 to 
  1021677
  1054290

  Y-> width: from 0 to 600374
  586944
  589005
 */



