void motor_setup()
{
  motA.attach(PE0, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  motB.attach(PE1, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  motC.attach(PE2, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  myservoX.attach(PE3);
  myservoY.attach(PE4);
  myservoA.attach(PE15);
  myservoB.attach(PE14);
  delay(3000);
}

void update_motor()
{ 
  if(ch5==0)
  {
    error_roll=error_pitch=error_yaw=integral_error_roll=integral_error_pitch=integral_error_yaw=derivative_error_roll=derivative_error_pitch=derivative_error_yaw=last_error_roll=last_error_pitch=last_error_yaw=0;
    error_roll1=error_pitch1=error_yaw1=integral_error_roll1=integral_error_pitch1=integral_error_yaw1=derivative_error_roll1=derivative_error_pitch1=derivative_error_yaw1=last_error_roll1=last_error_pitch1=last_error_yaw1=0;
    motA.writeMicroseconds(1000);
    motB.writeMicroseconds(1000);
    motC.writeMicroseconds(1000);
    myservoX.write(servoAngleInit1);
    myservoY.write(servoAngleInit2);
    myservoA.write(servoAngleInitA);
    myservoB.write(servoAngleInitB);
  }

  if(ch5==1 && throttle_channel<1000){armStatus=1;}

  if(armStatus==1 && ch6==0){  
    controlDrone();
    motA.writeMicroseconds(pulse_length_esc1);
    motB.writeMicroseconds(pulse_length_esc2);
    motC.writeMicroseconds(pulse_length_esc3);
    myservoX.write(pulse_length_servo1);
    myservoY.write(pulse_length_servo2);
    myservoA.write(servoAngleInitA);
    myservoB.write(servoAngleInitB);
    }

}