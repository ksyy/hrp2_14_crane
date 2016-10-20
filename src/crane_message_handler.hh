
#ifndef _CRANE_MESSAGE_HANDLER_HH_
#define _CRANE_MESSAGE_HANDLER_HH_

#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TransformStamped.h>
#include "crane_usb.hh"

enum CraneStrategy {JOYSTICK,POSITION};

class CraneMessageHandler
{

protected:
  // Scaling for the speed on X and Y axis.
  double a_scale_ ;
  double l_scale_ ;

  // Velocities to be send to the crane controller.
  double v_[3];
  // Boolean choices on the axis to be controlled.
  int s_[3];

  // Position values of the crane in millimeters
  // using encoders.
  // The maximum possible values are:
  // sensor_position_[0] (X-left rail): [0.98-10.94]
  // sensor_position_[1] (X-right rail): [1.09-11.00]
  // sensor_position_[2] (Y-transversal rail): [0.38-5.72] (But not under 0.97 from 2.44-2.59]  
  double       sensor_position_[3];
  double       sensor_velocity_[3];
  unsigned int sensor_label_;  

  // Detect if new joystick is to be applied
  bool apply_;

  // Choice on the control strategy:
  // JOYSTICK: read the joystick value from the 
  // corresponding ros node.
  // POSITION: apply position control.
  int strategy_;

  // Desired position value (X,Y).
  double desired_position_[3];
  
  // Previous error (X,Y)
  double previous_error_[3];

  // Integral of error (X,Y)
  double integral_[3];

  // PID gains.
  double Kp_[3], Kd_[3], Ki_[3];

  // Saturation threshold
  double Saturation_integral_;

  void joystickStrategy();
  void positionControlStrategy();
  void setHomeAsDesiredPosition();
  
  unsigned int count_;
  unsigned int mocap_status_;

  struct timeval lastcontrol_;
  struct timeval lastreading_;
public:
  CraneMessageHandler();
  ~CraneMessageHandler();
  
  // %Tag(CALLBACK)%
  void chatterCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
  void updatePositionCallback(const geometry_msgs::TransformStamped &tf);

  void applyControlStrategy();
  
  void checkVelocities(double *v, int *s);

  void setStrategy(CraneStrategy aStrategy);

  void setMocapStatus(unsigned int mocap_status);

  void stopEverything();

};
#endif /* _CRANE_MESSAGE_HANDLER_HH_ */
