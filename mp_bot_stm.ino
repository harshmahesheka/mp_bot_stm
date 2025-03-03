#include "ros_headers.h"
#include "motor.h"

ros::NodeHandle nh;

Motor* l;
Motor* r;

void vel_cb( const geometry_msgs::Point& cmd_msg){
  x_vel=cmd_msg.x;
  th_vel= cmd_msg.z;
  l_wheel_vel= x_vel- (th_vel*wheeltrack/2);
  r_wheel_vel= x_vel+ (th_vel*wheeltrack/2);
  l_vel= l_wheel_vel*100/(2*PI*wheelradius);
  r_vel= r_wheel_vel*100/(2*PI*wheelradius);
  
}

void vacuum_cb( const std_msgs::UInt16& control_msg ){
    analogWrite(RPWM,control_msg.data);
}

  
ros::Publisher odom_publisher("odom_point", &odom);
ros::Subscriber<geometry_msgs::Point> vel_sub("wheel_vel", vel_cb);
ros::Subscriber<std_msgs::UInt16> vacuum_sub("vacuum_control", vacuum_cb);

void updateEncoderL(){    
    int MSB = digitalRead(l->motor_encA);
    int LSB = digitalRead(l->motor_encB);

    int encoded = (MSB<<1)|LSB;
    int sum = (l->inter_val<<2)|encoded;
    
    if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
        l->pos--;
    if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
        l->pos++;

    l->inter_val = encoded;
    // if((millis() - l->prev_check)>5){ // update at 200 hz
    //   l->curr_vel = (double)MILLIINV*(l->read()-l->read(l->prev_pos))/(double)(millis() - l->prev_check);
    //   l->prev_check = millis();
    //   l->prev_pos = l->pos;
    // }
}

void updateEncoderR(){    
    int MSB = digitalRead(r->motor_encA);
    int LSB = digitalRead(r->motor_encB);

    int encoded = (MSB<<1)|LSB;
    int sum = (r->inter_val<<2)|encoded;
    
    if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
        r->pos--;
    if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
        r->pos++;

    r->inter_val = encoded;
    // if((millis() - l->prev_check)>5){ // update at 200 hz
    //   l->curr_vel = (double)MILLIINV*(l->read()-l->read(l->prev_pos))/(double)(millis() - l->prev_check);
    //   l->prev_check = millis();
    //   l->prev_pos = l->pos;
    // }
}

void odom_pub(double l_pos,double r_pos, double l_vel,double r_vel){
  
    dl = 2 * PI * wheelradius * (l_pos-l_last_tick) / 100;
    dr = 2 * PI * wheelradius * (r_pos-r_last_tick) / 100;
    double cycleDistance = (dr + dl) / 2;
   
    // Calculate the number of radians the robot has turned since the last cycle
    double dc = asin((dr-dl)/wheeltrack);
   
    // Average angle during the last cycle
    double avgAngle = dc/2 + last_th;
    if (avgAngle > PI) {
      avgAngle -= 2*PI;
    }
    else if (avgAngle < -PI) {
      avgAngle += 2*PI;
    }
    else{}

   

    odom_x += cos(avgAngle)*cycleDistance;
    odom_y += sin(avgAngle)*cycleDistance;
    odom_th +=dc;


    // Prevent lockup from a single bad cycle
    if (isnan(odom_x) || isnan(odom_y)
       || isnan(odom_th)) {
      odom_x = last_x;
      odom_y = last_y;
      odom_th = last_th;
    }
   
    // Make sure theta stays in the correct range
    if (odom_th > PI) {
      odom_th -= 2 * PI;
    }
    else if (odom_th < -PI) {
      odom_th += 2 * PI;
    }
    else{}
    odom.x=odom_x;
    odom.y=odom_y;
    odom.z=odom_th;

//    odom_quat = tf::createQuaternionFromYaw(odom_th);
//    
//    odom_trans.header.stamp =  nh.now();
//    odom_trans.header.frame_id = "odom";
//    odom_trans.child_frame_id = "base_link";
//
//    odom_trans.transform.translation.x = odom_x;
//    odom_trans.transform.translation.y = odom_y;
//    odom_trans.transform.translation.z = 0.0;
//    odom_trans.transform.rotation = odom_quat;
//    odom_broadcaster.sendTransform(odom_trans);
//    
//    odom.header.stamp = nh.now();
//    odom.header.frame_id = "odom";
//    odom.child_frame_id = "base_link";
//
//    //set the position
//    odom.pose.pose.position.x = odom_x;
//    odom.pose.pose.position.y = odom_y;
//    odom.pose.pose.position.z = 0.0;
//    odom.pose.pose.orientation = odom_quat;
    odom_publisher.publish(&odom);

    last_x=odom_th;
    last_y=odom_y;
    last_th=odom_th;
    l_last_tick = l_pos;
    r_last_tick = r_pos;
    
}

void setup() {
//  Serial.begin(9600);




  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(vel_sub);
  nh.advertise(odom_publisher);
  nh.subscribe(vacuum_sub);

  l = new Motor(LMOTOR_P, LMOTOR_N, LMOTOR_M, LMOTOR_ENCA, LMOTOR_ENCB, 0.65,0.5, 0);
  r = new Motor(RMOTOR_P, RMOTOR_N, RMOTOR_M, RMOTOR_ENCA, RMOTOR_ENCB,  0.65, 0.5,0);


  pinMode(l->motor_encA, INPUT);
  pinMode(l->motor_encB, INPUT);
  pinMode(l->motor_p, OUTPUT);
  pinMode(l->motor_n, OUTPUT);
  pinMode(l->motor_e, OUTPUT);
  pinMode(RPWM,OUTPUT);

  
  
  pinMode(r->motor_encA, INPUT);
  pinMode(r->motor_encB, INPUT);
  pinMode(r->motor_p, OUTPUT);
  pinMode(r->motor_n, OUTPUT);
  pinMode(r->motor_e, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(l->motor_encA), updateEncoderL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(l->motor_encB), updateEncoderL, CHANGE);

  attachInterrupt(digitalPinToInterrupt(r->motor_encA), updateEncoderR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(r->motor_encB), updateEncoderR, CHANGE);

  prev_time = micros();
  cnt = 0;
}
void loop() {


  prev_time = micros();
  l->setVel(l_vel);
  r->setVel(r_vel);

  if((millis() - l->prev_check)>20){ // update at 200 hz
    l->curr_vel = (double)MILLIINV*(l->read()-l->read(l->prev_pos))/(double)(millis() - l->prev_check);
    l->prev_check = millis();
    l->prev_pos = l->pos;
  }
  
  if((millis() - r->prev_check)>20){ // update at 200 hz
    r->curr_vel = (double)MILLIINV*(r->read()-r->read(r->prev_pos))/(double)(millis() - r->prev_check);
    r->prev_check = millis();
    r->prev_pos = r->pos;
  }
  
  l->control();
  r->control();

  cnt++;
  if (cnt == 10){ // Rate/10
    odom_pub(l->read(), r->read(), l->curr_vel, r->curr_vel);
    cnt = 0;
  }
  //delay(10); // 100 hz
  long unsigned int dt = (micros() - prev_time);
  delayMicroseconds(max(((long unsigned int)666 - dt), (long unsigned int)0)); // 1.5KHz  = 1000000/1500 = 666.66
  //ros_pot.print("********************************");
  //ros_pot.print(1000000./(micros()-prev_time));
  //ros_pot.print("********************************\n");
  //prev_time = micros();
  nh.spinOnce();
  }
