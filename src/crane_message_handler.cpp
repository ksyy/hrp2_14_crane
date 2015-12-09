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
  mocap_status_= 0;
  v_[0] = v_[1] = v_[2] = 0.0;
  s_[0] = s_[1] = s_[2] = 1;
  for(unsigned int i=0;i<3;i++)
    sensor_position_[i] = 0.0;

  std::cout << "Try to connect to the crane." << std::endl;
  ptp_connect();
  std::cout << "Connection finished" << std::endl;

  for(unsigned int i=0;i<3;i++)
    {
      previous_error_[i] = 0.0;
      integral_[i] = 0.0;
    }

  setHomeAsDesiredPosition();


  Kp_[0] = 0.75; Kp_[1] = 1.5; Kp_[2] = 1.5;
  Kd_[0] = 1.0; Kd_[1] = 1.0; Kd_[2] = 0.0;
  Ki_[0] = 0.0; Ki_[1] = 0.0; Ki_[2] = 0.0;

  count_=0;

  gettimeofday(&lastreading_,0);
  gettimeofday(&lastcontrol_,0);
}

void CraneMessageHandler::setHomeAsDesiredPosition()
{
  desired_position_[0] = 0.935;
  desired_position_[1] = desired_position_[0]+0.09;  
  desired_position_[2] = 0.379;  
}

#if 0
  desired_position_[0] = 7.154;
  desired_position_[1] = desired_position_[0]+0.09;  
  desired_position_[2] = 3.432; 
#endif

CraneMessageHandler::~CraneMessageHandler()
{
  ptp_disconnect();
}


void CraneMessageHandler::checkVelocities(double * v, int *s)
{
  // Check after the closet along the X Axis.
  if (sensor_position_[0]>2.44)
    {
      // If the crane is to close to the cameras
      if (sensor_position_[2]<2.00)
	{
	  // If the speed is negative slow down
	  if (v[1]<0.0)
	  {
	    v[1]=0.05*(sensor_position_[2] - 1.0);
	    s[1] = 1;
	  }
	}
      // Check if we are not close to the boundary.
      if (sensor_position_[0]>10.0)
	{
	  if (v[0]>0.0)
	  {
	    v[0]=-0.05*(sensor_position_[1] - 10.0);
	    s[0] = 1;
	  }
	}

      // Check max. velocities along X clamped it to one.
      if (v[0]*v[0]> 1.0)
	{
	  if (v[0]<0.0)
	    v[0] = -1.0;
	  else
	    v[0] = 1.0;
	}

      // Check max. velocities along Y clamped it to one.
      if (v[1]*v[1]> 1.0)
	{
	  if (v[1]<0.0)
	    v[1] = -1.0;
	  else
	    v[1] = 1.0;
	}

    }

  if (sensor_position_[0]<2.44)
    {
      if (sensor_position_[2]<1.00)
	{
	  // If the speed is negative
	  if (v[1]<0.0)
	  {
	    v[1]=0.05*(sensor_position_[2] - 1.0);
	    s[1] = 1;
	  }
	}

    }

  if (mocap_status_==0)
    {
      std::cout << "Protect the crane" << std::endl;
      v[0]=0.0;
      v[1]=0.0;
    }
}

void CraneMessageHandler::chatterCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  v_[0] = a_scale_ * joy->axes[0];
  v_[1] = a_scale_ * joy->axes[1];
  v_[2] = a_scale_ * joy->axes[2];
  apply_= true;
}

void CraneMessageHandler::updatePositionCallback(const geometry_msgs::TransformStamped &tf)
{
  desired_position_[0] = tf.transform.translation.x+6.91;
  desired_position_[1] = desired_position_[0]+0.03;
  desired_position_[2] = tf.transform.translation.y+3.59;
  /*
  desired_position_[0] = 0.935;
  desired_position_[1] = desired_position_[0]+0.09;  
  desired_position_[2] = 0.379;  

  */
  /*
  std::cout << "updatePositionCallback: " 
	    << desired_position_[0] << " "   
	    << desired_position_[1] << " "   
	    << desired_position_[2] << std::endl;
  */
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

#define DIM_TO_CONTROL 3
void CraneMessageHandler::positionControlStrategy()
{
  double error[DIM_TO_CONTROL];
  double dt=0.1;
  double derivate[DIM_TO_CONTROL], integral[DIM_TO_CONTROL];

  for(unsigned int i=0;i<DIM_TO_CONTROL;i++)
    {
      error[i] = desired_position_[i] - sensor_position_[i];
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
    lastcontrol_ = controltime;

    double map_v[3];
    map_v[0] = v_[0];
    map_v[1] = v_[2];
    map_v[2] = 0.0;

    checkVelocities(map_v,s_);

    if (ptp_control(map_v,s_))
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
    if(r >=2)
      for(unsigned int i=0;i<3;i++)
	sensor_position_[i] = sensor_position_[i]/1000.0;

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

  printf("%lf, %d, Positions : (%4.2lf, %4.2lf) (%4.2lf, %4.2lf) (%4.2lf,%4.2lf) Speed : %4.2lf, %4.2lf, %4.2lf \n",intervalreading, 
	 (unsigned int)sensor_label_, 
	 sensor_position_[0], desired_position_[0],
	 sensor_position_[1], desired_position_[1],
	 sensor_position_[2], desired_position_[2],
	 sensor_velocity_[0], sensor_velocity_[1],sensor_velocity_[2]);
  
  if (intervalreading > 0.07)
    if (valid_encoders && flag_compare)
      printf("%lf, %d, Positions : (%4.2lf, %4.2lf) (%4.2lf, %4.2lf) (%4.2lf,%4.2lf) Speed : %4.2lf, %4.2lf, %4.2lf \n",intervalreading, 
	     (unsigned int)sensor_label_, 
	     sensor_position_[0], desired_position_[0],
	     sensor_position_[1], desired_position_[1],
	     sensor_position_[2], desired_position_[2],
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

void CraneMessageHandler::setMocapStatus(unsigned int mocap_status)
{
  mocap_status_ = mocap_status;
}

void CraneMessageHandler::stopEverything()
{
  double lv[3]={0.0,0.0,0.0};
  int ls[3]={1,1,1};

  ptp_control(lv,ls);
}
