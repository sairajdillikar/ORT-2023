
#include <ros.h>
#include <std_msgs/String.h>
#include <stdint.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Joy.h>

#include <std_msgs/UInt16.h>
#include <std_msgs/Float32MultiArray.h>

#include <Servo.h>


//wheels
Servo pin2;
Servo pin3;
Servo pin4;
Servo pin5;

//manipulator
Servo pin8;
Servo pin9;
Servo pin10;
Servo pin11;


bool slow ;
bool manipulator;
bool locomotion;
float RT;
float LT;
int A;
int power;

void normal_speed(float RT,float LT)
{
    if (RT <= -0.5)
  {
    pin2.write(-180);
    pin3.write(-180);
    pin4.write(180);
    pin5.write(180);
  }
//  else if (LT <= -0.5)
//  {
//    pin2.write(-180);
//    pin3.write(180);
//    pin4.write(180);
//    pin5.write(-180);
//  }
  else
  {
    pin2.writeMicroseconds(1500);  
  }

}

void slow_speed(float RT,float LT)
{
   if (RT <= -0.5)
  {
     pin2.writeMicroseconds(2000); 
     pin3.writeMicroseconds(1000);
     pin4.writeMicroseconds(1000);
     pin5.writeMicroseconds(2000);
  }
  else if (LT <= -0.5)
  {  pin2.writeMicroseconds(1000); 
     pin3.writeMicroseconds(2000);
     pin4.writeMicroseconds(2000);
     pin5.writeMicroseconds(1000);
   
  }
  else
  {
    pin2.writeMicroseconds(1500);  
  }

}

void joydata(const sensor_msgs :: Joy& joy)
{
  //joy_acc = joy.axes[5];
  //joyVal = constrain(joy.axes[1], -0.5,0.5);

  RT = joy.axes[5];
  LT = joy.axes[2];
  power = joy.buttons[8];
  A = joy.buttons[0];
  
  if (A == 1)
  {
   if(!slow)Int16MultiArray
   {
    slow = true ;
   }
   else
    slow =false;
  }

  if(power == 1)
  {
    manipulator = true;  
  }
  

  if(!manipulator)
  {
      if(!slow)
      {
      normal_speed(RT,LT);  
      } 
      else if(slow)
      { 
      slow_speed(RT,LT); 
      }
  }
  
}


void servo_cb( const std_msgs::Int16MultiArray& cmd_msg ){

  if(manipulator)
  {
  pin8.write(cmd_msg.data[0]); //set servo angle, should be from 0-180  
  pin9.write(cmd_msg.data[1]);
  pin10.write(cmd_msg.data[2]);
  pin11.write(cmd_msg.data[3]); 
  } 
}


ros::NodeHandle nh;
ros::Subscriber<sensor_msgs::Joy> sub1("joy", joydata);
ros::Subscriber<std_msgs::Float32MultiArray> sub2("servo", servo_cb);

//char hello[13] = "20";
void setup()
{
  //add servos for wheels
  pin2.attach(2);//RL +180 //2000
  pin3.attach(3);//RR -180 //1500
  pin4.attach(4);//FR -180 //1500
  pin5.attach(5);//FL +180 //2000

  //manipulator
  pin8.attach(8);
  pin9.attach(9);
  pin10.attach(10);
  pin11.attach(11);

  
  slow = false;
  manipulator = false;
  RT = 0.0;
  LT = 0.0;
  A=0;
  power = 0;
  
  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  
  nh.spinOnce();
}

void loop()
{
  //str_msg.data = hello;
  //chatter.publish( &str_msg );
  //Serial.println(joyVal);
 

}
