#include <stdio.h>
#include <err.h>
#include <math.h>
#include <iostream>
#include <ftdi.h>

#include <sys/time.h>

#define USB_VENDOR_FTDI 0x0403
#define USB_PRODUCT_SERIAL 0x6001

#define LEVEL_FOR_DISPLAYING_ENCODERS 1

/* --- local data ---------------------------------------------------------- */

static struct ftdi_context *ftdic;


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
  do 
    {
      ok=true;
      usleep(1000.0);
      r=ptp_control(v,s);
      if (r!=0)
	{
	  count ++;
	  if (count<k)
	    ok=false;
	}
    }
  while(!ok);

  return r;
}

/*
        The purpose of this function is to convert a reflected binary
        Gray code number to a binary number.
*/
int grayToBinary(unsigned int num)
{
    unsigned int numBits = 8 * sizeof(num);
    unsigned int shift=0;
    unsigned int mask=0;

    bool Negative=false;
    mask = 1;
    mask = mask << 24;
    if (mask&num)
      Negative = true;
    num = num & ~mask;

    for (shift = 1; shift < 24; shift *= 2)
      num ^= num >> shift;

    if (Negative)
	return -num;
    return num;
}

#define SIZE_ENCODER_MSG 18
#define SIZE_IOCTL_MSG 36
int ptp_read_encoders(double encoders[3])
{
  unsigned int debug=1;

  static unsigned char buffer[SIZE_IOCTL_MSG];
  
  int e;

  if (!ftdic) return -1;
  e = ftdi_read_data(ftdic, buffer, SIZE_IOCTL_MSG);

  if (e < 0) {
    warnx("ftdi_read_data: %s", ftdi_get_error_string(ftdic));
    return 1;
  }

 
  if ((e==0) || (e<SIZE_ENCODER_MSG))
    return 0;

  // Find head of packet.
  unsigned head_packet=0;
  while( ((buffer[head_packet]!=0x10) ||
	 (buffer[head_packet+1]!=0xaa) ) &&
	 head_packet<SIZE_IOCTL_MSG-1)
    head_packet++;


  // Print header.
  if (debug>LEVEL_FOR_DISPLAYING_ENCODERS)
    {
      for(unsigned int i=head_packet;
	  i<head_packet+2;i++)
	printf ("0x%x ", buffer[i]);
      printf("||");

      // Print label 
      for(unsigned int i=head_packet+2;
	  i<head_packet+4;i++)
	printf ("0x%x ", buffer[i]);
      printf("||");
    }

  typedef union 
  {
    int x;
    unsigned char c[4];
  }position_type;

  position_type X[3];
  
  int Positions[3];
  unsigned int shift=0;
  unsigned int idX=0;

  double scale[3] = {0.0, 5.8/586944.0, -11.0/1021677.0};

  for (unsigned int i=head_packet+4,j=0;
       i<head_packet+SIZE_ENCODER_MSG-2+shift;i++)
    {

      if (debug>LEVEL_FOR_DISPLAYING_ENCODERS)
	printf ("0x%x ", buffer[i]);

      X[idX].c[j] = buffer[i];
      j++;
      if (j==4)
	{ 
	  Positions[idX] = grayToBinary(X[idX].x);
	  encoders[idX] = (double)(Positions[idX]) *scale[idX];
	  if (debug>LEVEL_FOR_DISPLAYING_ENCODERS)
	    printf(" %d ",Positions[idX]);
	  idX++;j=0;
	}
      if (buffer[i]==0x10)
	{ i++;shift++;}
    }

  static unsigned int tail[2];
  for (unsigned int i=head_packet+SIZE_ENCODER_MSG-2+shift,j=0;
	   i<head_packet+SIZE_ENCODER_MSG+shift;
       i++,j++)
    tail[j] = buffer[i];

  if (debug > LEVEL_FOR_DISPLAYING_ENCODERS)
    {
      printf(" %d 0x%x 0x%x 0x%x 0x%x||",sizeof(X[2].x),
	     X[2].c[0], X[2].c[1], X[2].c[2], X[2].c[3]);
      
      // Print tail
      for (unsigned int i=head_packet+SIZE_ENCODER_MSG-2+shift;
	   i<head_packet+SIZE_ENCODER_MSG+shift;
	   i++)
	printf ("0x%x ", buffer[i]);
      
    }

  if ((tail[0]!=0x10) ||
      (tail[1]!=0x3))
    return -1;
    
  if (debug>LEVEL_FOR_DISPLAYING_ENCODERS)
    printf("\n");

  return 0;
}

int ptp_reread_encoders(double encoder_values[3],
			 unsigned int k)
{
  bool ok=true;
  unsigned int count=0;
  int r=0;
  do 
    {
      ok=true;
      usleep(1000.0);
      r=ptp_read_encoders(encoder_values);
      if (r!=0)
	{
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
