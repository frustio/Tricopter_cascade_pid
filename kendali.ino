void controlDrone(){
  currentTime_pid=millis();
  if(currentTime_pid-previousTime_pid>9){
  dt=(currentTime_pid-previousTime_pid);
//====================================================OUTER LOOP==============================================================
  if(ch6!=0){yaw_target=roll_input;}
  else if(ch6 == 0){yaw_target = yaw_input;}
  error_roll1=(roll_input)-roll_deg;
  error_pitch1=(pitch_input*-1)-pitch_deg;
  error_yaw1=(yaw_target*-1)-yaw_control;

  integral_error_roll1+=error_roll1*0.01;     integral_error_roll1 = constrain(integral_error_roll1, -150, 150);
  integral_error_pitch1+=error_pitch1*0.01;   integral_error_pitch1 = constrain(integral_error_pitch1, -150, 150); 
  integral_error_yaw1+=error_yaw1*0.01;       integral_error_yaw1 = constrain(integral_error_yaw1, -100, 100);

  derivative_error_roll1=(error_roll1-last_error_roll1)/0.01;
  derivative_error_pitch1=(error_pitch1-last_error_pitch1)/0.01;
  derivative_error_yaw1=(error_yaw1-last_error_yaw1)/0.01;

  Proll1=Kp_roll*error_roll1;                 Ppitch1=Kp_pitch*error_pitch1;                  Pyaw1=Kp_yaw*error_yaw1;                                   
  Iroll1=Ki_roll*integral_error_roll1;        Ipitch1=Ki_pitch*integral_error_pitch1;         Iyaw1=Ki_yaw*integral_error_yaw1;                
  Droll1=Kd_roll*(derivative_error_roll1);    Dpitch1=Kd_pitch*(derivative_error_pitch1);     Dyaw1=Kd_yaw*(derivative_error_yaw1);       
  last_error_roll1=error_roll1;               last_error_pitch1=error_pitch1;                 last_error_yaw1=error_yaw1;                           

  PID_value_roll1 = Proll1 + Iroll1 + Droll1;   PID_value_pitch1 = Ppitch1 + Ipitch1 + Dpitch1;   PID_value_yaw1 = Pyaw1 + Iyaw1 + Dyaw1;     

//=====================================================INNER LOOP===============================================================
  error_roll=PID_value_roll1-gx;
  error_pitch=PID_value_pitch1-(gy*-1);
  error_yaw=(PID_value_yaw1)-gz;

  integral_error_roll+=error_roll*0.01;
  integral_error_pitch+=error_pitch*0.01;
  integral_error_yaw+=error_yaw*0.01;

  derivative_error_roll=(error_roll-last_error_roll)/0.01;
  derivative_error_pitch=(error_pitch-last_error_pitch)/0.01;
  derivative_error_yaw=(error_yaw-last_error_yaw)/0.01;

  Proll=Kp_roll2*error_roll;                 Ppitch=Kp_pitch2*error_pitch;                  Pyaw=Kp_yaw2*error_yaw;              
  Iroll=Ki_roll2*integral_error_roll;        Ipitch=Ki_pitch2*integral_error_pitch;         Iyaw=Ki_yaw2*integral_error_yaw;       
  Droll=Kd_roll2*(derivative_error_roll);    Dpitch=Kd_pitch2*(derivative_error_pitch);     Dyaw=Kd_yaw2*(derivative_error_yaw);  
  last_error_roll=error_roll;                last_error_pitch=error_pitch;                  last_error_yaw=error_yaw;             

  PID_value_roll = Proll + Iroll + Droll;   PID_value_pitch = Ppitch + Ipitch + Dpitch;   PID_value_yaw = Pyaw + Iyaw + Dyaw;
  previousTime_pid=currentTime_pid;

  if(PID_value_roll > PID_max){PID_value_roll = PID_max;} else if(PID_value_roll < PID_max*-1){PID_value_roll = PID_max*-1;}
  if(PID_value_pitch > PID_max){PID_value_pitch = PID_max;} else if(PID_value_pitch < PID_max*-1){PID_value_pitch = PID_max*-1;}
  if(PID_value_yaw > PID_max){PID_value_yaw = PID_max;} else if(PID_value_yaw < PID_max*-1){PID_value_yaw = PID_max*-1;}
  }

    rollControl  = PID_value_roll;
    pitchControl = PID_value_pitch;
    yawControl   = PID_value_yaw;

    pulse_length_esc1 = throttle_channel + (pitchControl) + (rollControl); 
    pulse_length_esc3 = throttle_channel - (pitchControl); 
    pulse_length_esc2 = throttle_channel + (pitchControl) - (rollControl);
    pulse_length_servo1 = (yawFiltered*1);
    pulse_length_servo2 = (yawFiltered*1);

    pulse_length_esc1 = constrain(pulse_length_esc1, 1100, 2000);
    pulse_length_esc2 = constrain(pulse_length_esc2, 1100, 2000);
    pulse_length_esc3 = constrain(pulse_length_esc3, 1100, 2000);
    pulse_length_esc3 = map(pulse_length_esc3, 1100, 2000, 1100, 1700);
    pulse_length_servo1 = constrain(pulse_length_servo1, -25, 15);pulse_length_servo1 = map(pulse_length_servo1, -30, 30, servo1_down, servo1_up);
    pulse_length_servo2 = constrain(pulse_length_servo2, -15, 25);pulse_length_servo2 = map(pulse_length_servo2, -30, 30, servo2_down, servo2_up);
    
}

float lpf(float x, float y_prev, float alpha){ return alpha*x + (1-alpha)*y_prev; }
