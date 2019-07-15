/* Author: Olivier Stasse,
   Copyright: CNRS, LAAS, Gepetto
   File: Header for the object controlling the crane
   in the experimental room of HRP-2 n 14.
*/

#ifndef _CRANE_USB_H_
#define _CRANE_USB_H_

#define USB_VENDOR_FTDI 0x0403
#define USB_PRODUCT_SERIAL 0x6001

#define LEVEL_FOR_DISPLAYING_ENCODERS 1

#define SIZE_ENCODER_MSG 18
#define SIZE_IOCTL_MSG   36
#define SENSOR_NB        3
#define DLE              0x10
#define ETX              0x03
#define ADDR_TX          0xAA
#define POSITION_MASK    0x00FFFC00
#define VELOCITY_MASK    0x000003F8
#define NO_POS_MASK      0x00000004
#define WARNING_MASK     0x00000002
#define ERROR_MASK       0x00000001
#define FLAGS_MASK       0x00000005
#define POSITION_SHIFT   10
#define VELOCITY_SHIFT   3
#define NO_POS_SHIFT     2
#define WARN_SHIFT       1
#define ERROR_SHIFT      0

enum MsgStep {
  DLE_HEADER,
  ADDRESS_ID,
  LABEL_MSB,
  LABEL_LSB,
  SENSOR_BYTE_0,
  SENSOR_BYTE_1,
  SENSOR_BYTE_2, 
  SENSOR_BYTE_3,
  DLE_FOOTER,
  ETX_FOOTER 
};

struct MsgStruct {
  unsigned int   Sensor_Label;
  unsigned int   Sensor_Msg[SENSOR_NB];
  unsigned int   Sensor_Position[SENSOR_NB];
  unsigned int   Sensor_Velocity[SENSOR_NB];
  bool           Sensor_NoPos[SENSOR_NB];
  bool           Sensor_Warn[SENSOR_NB];
  bool           Sensor_Error[SENSOR_NB];
};

extern int ptp_connect(void);
extern void ptp_disconnect(void);
extern int ptp_control(double v[3], int s[3]);
extern int ptp_recontrol(double v[3], int s[3],unsigned int k);
extern int ptp_read_encoders(unsigned int *Label, double Position[3], double Velocity[3]);
extern int ptp_reread_encoders(unsigned int *Label, double Position[3], double Velocity[3], unsigned int k);
extern void ptp_reconnect_until_ok_or_k_demands(unsigned int k);
#endif /* _CRANE_USB_H_ */
