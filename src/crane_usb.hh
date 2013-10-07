/* Author: Olivier Stasse,
   Copyright: CNRS, LAAS, Gepetto
   File: Header for the object controlling the crane
   in the experimental room of HRP-2 n 14.
*/

#ifndef _CRANE_USB_H_
#define _CRANE_USB_H_

extern int ptp_connect(void);
extern void ptp_disconnect(void);
extern int ptp_control(double v[3], int s[3]);
extern int ptp_recontrol(double v[3], int s[3],unsigned int k);
extern int ptp_read_encoders(double encoders[3]);
extern int ptp_reread_encoders(double encoders[3],
			       unsigned int k);
extern void ptp_reconnect_until_ok_or_k_demands(unsigned int k);
#endif /* _CRANE_USB_H_ */
