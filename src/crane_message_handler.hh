
#ifndef _CRANE_MESSAGE_HANDLER_HH_
#define _CRANE_MESSAGE_HANDLER_HH_

#include <sensor_msgs/Joy.h>
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

  // Position values of the crane in meters
  // using encoders.
  double encoder_values_[3];

  // Detect if new joystick is to be applied
  bool apply_;

  // Choice on the control strategy:
  // JOYSTICK: read the joystick value from the 
  // corresponding ros node.
  // POSITION: apply position control.
  int strategy_;

  // Desired position value (X,Y).
  double desired_position_[2];
  
  // Previous error (X,Y)
  double previous_error_[2];

  // Integral of error (X,Y)
  double integral_[2];

  // PID gains.
  double Kp_[2], Kd_[2], Ki_[2];

  void joystickStrategy();
  void positionControlStrategy();

  unsigned int count_;

  struct timeval lastcontrol_;
  struct timeval lastreading_;
public:
  CraneMessageHandler();
  ~CraneMessageHandler();
  
  // %Tag(CALLBACK)%
  void chatterCallback(const sensor_msgs::Joy::ConstPtr& joy);

  void applyControlStrategy();

  void setStrategy(CraneStrategy aStrategy);

};
#endif /* _CRANE_MESSAGE_HANDLER_HH_ */
