#include "crane_message_handler.hh"

double computeTime(struct timeval &begin,
		   struct timeval &end)
{
  return (double)(end.tv_sec -begin.tv_sec+
		  0.000001 * (end.tv_usec - begin.tv_usec));
}

CraneMessageHandler::CraneMessageHandler():
  a_scale_(0.4),
  l_scale_(0.4),
  apply_(false),
  //strategy_(JOYSTICK)
  strategy_(POSITION)
{
  v_[0] = v_[1] = v_[2] = 0.0;
  s_[0] = s_[1] = s_[2] = 1;
  for(unsigned int i=0;i<3;i++)
    sensor_position_[i] = 0.0;

  std::cout << "Try to connect to the crane." << std::endl;
  ptp_connect();
  std::cout << "Connection finished" << std::endl;

  for(unsigned int i=0;i<2;i++)
    {
      previous_error_[i] = 0.0;
      integral_[i] = 0.0;
    }

  desired_position_[0] = 4.3;
  desired_position_[1] = 4.3;  

  Kp_[0] = 0.5; Kp_[1] = 0.5;
  Kd_[0] = 0.0; Kd_[1] = 0.0;
  Ki_[0] = 0.0; Ki_[1] = 0.0;

  count_=0;

  gettimeofday(&lastreading_,0);
  gettimeofday(&lastcontrol_,0);
}

CraneMessageHandler::~CraneMessageHandler()
{
  ptp_disconnect();
}

void CraneMessageHandler::chatterCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  v_[0] = a_scale_ * joy->axes[0];
  v_[1] = a_scale_ * joy->axes[1];
  v_[2] = a_scale_ * joy->axes[2];
  apply_= true;
}

void CraneMessageHandler::joystickStrategy()
{
 if (apply_)
   {
     ROS_INFO("I'll write: %f %f %f", 
	      v_[0],v_[1],v_[2]);
     apply_=false;
   }
 
 if (ptp_control(v_,s_))
   ptp_reconnect_until_ok_or_k_demands(3); 

}

#define DIM_TO_CONTROL 2
void CraneMessageHandler::positionControlStrategy()
{
  double error[DIM_TO_CONTROL];
  double dt=0.1;
  double derivate[2], integral[2];

  for(unsigned int i=0;i<DIM_TO_CONTROL;i++)
    {
      error[i] = desired_position_[i] - sensor_position_[i+1];
      derivate[i] = (error[i] - previous_error_[i])/ dt;
      integral[i] = integral[i] + error[i]*dt;

      v_[i] = Kp_[i] * error[i] + Kd_[i] * derivate[i] 
	+ Ki_[i] * integral[i];

       previous_error_[i] = error[i];

     }

  
  struct timeval controltime;
  gettimeofday(&controltime,0);

  double intervalcontrol = computeTime(lastcontrol_,
				       controltime);

  double intervalreadingandnow = computeTime(lastreading_,
					     controltime);

  if ((intervalcontrol>0.07) && (intervalreadingandnow>0.02)) {
      /*
      std::cout << "interval control: " 
		<< intervalcontrol 
		<< "interval reading and now: " 
		<< intervalreadingandnow
		<< std::endl;*/
      lastcontrol_ = controltime;
      if (ptp_control(v_,s_))
	ptp_reconnect_until_ok_or_k_demands(3);  
    }

}

void CraneMessageHandler::applyControlStrategy() 
{
  static double sensor_compare_[3] = {0.0, 0.0, 0.0};
  int r = 0, i = 0;
  bool valid_encoders = true, flag_compare = false;
  struct timeval reading;
  gettimeofday(&reading,0);
  
  double intervalreading = computeTime(lastreading_, reading);
  double intervalcontrolandnow  = computeTime(lastcontrol_, reading);
  
  if ((intervalreading > 0.07) && (intervalcontrolandnow > 0.02)) {
    r=ptp_read_encoders(&sensor_label_, sensor_position_, sensor_velocity_);
    lastreading_ = reading;
  }
  
  if (r==1) {
    //ptp_reconnect_until_ok_or_k_demands(3); 
    valid_encoders = false;
  }
  else if (r==-1) { 
    valid_encoders = false;
  }
  
  switch(strategy_) {
  case JOYSTICK :
    joystickStrategy();
    break;
  case POSITION :
    if (valid_encoders)
      positionControlStrategy();
    break;
  }
  
  flag_compare |= (sensor_compare_[0] -= sensor_position_[0]) ? (true) : (false);
  flag_compare |= (sensor_compare_[1] -= sensor_position_[1]) ? (true) : (false);
  flag_compare |= (sensor_compare_[2] -= sensor_position_[2]) ? (true) : (false);
  
  if (intervalreading > 0.07)
    if (valid_encoders && flag_compare)
      printf("%lf, %d, Positions : %4.0lf, %4.0lf, %4.0lf Speed : %4.0lf, %4.0lf, %4.0lf  \n",intervalreading, 
	     (unsigned int)sensor_label_, sensor_position_[0], sensor_position_[1], sensor_position_[2], 
	     sensor_velocity_[0], sensor_velocity_[1],sensor_velocity_[2]);
  for (i = 0; i < SENSOR_NB; i++) 
    sensor_compare_[i] = sensor_position_[i];
  flag_compare = false;
  
      //std::cout << intervalreading << " Positions:" << sensor_position_[0] << " "<< sensor_position_[1] <<" " << sensor_position_[2] << std::endl;
}

void CraneMessageHandler::setStrategy(CraneStrategy aStrategy)
{
  strategy_ = aStrategy;
}
