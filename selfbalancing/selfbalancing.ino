#include <hubMotorV2.h>
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <Kalman.h>
#include <math.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <avr/wdt.h>

//----- Data for Velocity calculation ----------
#define CYCLE_ENCODER_COUNT 90.0
#define WHEEL_DIAMETER 0.1651
#define BASE_WIDTH 0.46
#define M_PI 3.141592653589793238
//----------------------------------------------

//----- Motor Pins and Base Speeds -------------
#define L_M_ENCODER A0//18
#define R_M_ENCODER A1//19
#define L_M_PWM 6
#define R_M_PWM 5

#define LEFT_BASE_MOTOR_SPEED_FWD 15
#define LEFT_BASE_MOTOR_SPEED_BWD 0
#define RIGHT_BASE_MOTOR_SPEED_FWD 15
#define RIGHT_BASE_MOTOR_SPEED_BWD 0
//----------------------------------------------

//----- Base angles and angle limits -----------
#define BASE_ANGLE -0.65//-19.0
#define MAX_ANGLE 20
#define MIN_ANGLE 0.5
//----------------------------------------------


//----- Base angle and Rotation speed calculation -------------
float linear_vel_error = 0;
float linear_vel_error_sum = 0;
float last_linear_vel_error = 0;
float linear_kp = 5;
float linear_kd = 0;
float linear_ki = 0.01;

float angular_vel_error = 0;
float angular_vel_error_sum = 0;
float angular_kp = 40;
float angular_ki = 0.1;

double current_vx = 0;
double current_vth = 0;

double raw_vx;
double raw_vth;

float base_angle = BASE_ANGLE;
int rot_speed = 0;

unsigned long time_1 = 0;
unsigned long time_2 = 0;
double time_diff = 0;
//-------------------------------------------------------------


//------- Motor Controlling Definitions -----------------------
struct motorProperties {
  unsigned long EncoderCount;
  unsigned long encoder_count_1;
  unsigned long encoder_count_2;
  double cur_speed;
  int direction;
};

motorProperties L_M_prop;
motorProperties R_M_prop;

hubMotorV2 LM(L_M_PWM);
hubMotorV2 RM(R_M_PWM);
//--------------------------------------------------------------

//----- IMU Definitions ----------------------------------------
MPU6050 accelgyro;
Kalman kalman;

int16_t ax, ay, az;
int16_t gx, gy, gz;
double kalmanAngle;
double pitch_angle = 0;
unsigned long prev_time = 0;

//--------------------------------------------------------------

//----- PID related --------------------------------------------
float kp_angle = 14.2;
float ki_angle = 0.1;
float kd_angle = 30;

float prev_angle_error = 0;
float angle_error_sum = 0;
//--------------------------------------------------------------

//----- ROS related --------------------------------------------

ros::NodeHandle  nh;
std_msgs::Float32MultiArray readings;
std_msgs::MultiArrayDimension myDim;
ros::Publisher ros_pub("base_controller_data", &readings);

double x = 0.0;
double y = 0.0;
double theta = 0.0;
double vx = 0;
double vth = 0;

void cb(const geometry_msgs::Twist& twist_msg) {
  vx = twist_msg.linear.x;
  vth = twist_msg.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", cb);
//--------------------------------------------------------------


void setup()
{
  LM.init();
  RM.init();

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(ros_pub);
  initRosPub();

//    Serial.begin(9600);

  setMotorProperties();

  pinMode(L_M_ENCODER, INPUT_PULLUP);
  pinMode(R_M_ENCODER, INPUT_PULLUP);

  Wire.begin();
  accelgyro.initialize();

  prev_time = millis();
  initKalman();

  Serial3.begin(9600);
  // delay(100);


  Serial3.println("Initiation Done");
  wdt_enable(WDTO_250MS);

  controlLoop();

}


void loop()
{
  //----- for testing -----------------------
  Serial3.println(getKalmanAngle());

  delay(100);

}

void controlLoop() {
  unsigned long control_loop_time = 100; // milliseconds
  unsigned long last_control_loop_time = 0;
  while (1) {
    //        serial_pid(&kp_angle, &ki_angle, &kd_angle,&base_angle);
    updateIMU();
    pid();
    updateEncoderValues();
    joystick_read();
    if (millis() - last_control_loop_time > control_loop_time) {
      setBaseAngleAndRotation(&base_angle, &rot_speed);
      publishOdom();
      last_control_loop_time = millis();
    }
    nh.spinOnce();
    wdt_reset();
  }

}

void initRosPub() {
  char label[] =  "test";
  int array_size = 9;
  myDim.label = label;
  myDim.size = array_size;
  myDim.stride = 1 * array_size;
  readings.layout.data_offset = 0;
  readings.layout.dim[0] = myDim;
  readings.data_length = array_size;
  readings.data = (float *)malloc(sizeof(float) * array_size);

}

void publishOdom() {
  double vx_pub = raw_vx;
  double vth_pub = raw_vth; 

  double dt = time_diff / 1000.0;
  double dx = vx_pub * cos(theta) * dt;
  double dy = vx_pub * sin(theta) * dt;
  double dtheta = vth_pub * dt;

  x += dx;
  y += dy;
  theta += dtheta;

  //----gyro_rate------------------------------------------------------
  double wx = (gx / 131.0);
  double wz = (gz / 131.0);

  double gyro_rate = -wx * sin(pitch_angle*M_PI / 180.0) + wz * cos(pitch_angle*M_PI / 180.0);
  //-------------------------------------------------------------------

  //----accelerometer--------------------------------------------------
  double a_x = ax/16384.0;
  double a_z = az/16384.0;

  double accelerometer_rate = a_x*cos(pitch_angle*M_PI / 180.0) + a_z*sin(pitch_angle*M_PI / 180.0);
  //-------------------------------------------------------------------
  //------------base_controller_data-----------------------------------
  readings.data[0] = x;
  readings.data[1] = y;
  readings.data[2] = theta;
  readings.data[3] = vx_pub;
  readings.data[4] = vth_pub;
  readings.data[5] = pitch_angle;
  readings.data[6] = gyro_rate;
  readings.data[7] = accelerometer_rate;
  readings.data[8] = dt;
  ros_pub.publish(&readings);
  //-------------------------------------------------------------------
}


void setBaseAngleAndRotation(float *base_angle, int *rot_speed) {

  L_M_prop.encoder_count_2 = L_M_prop.EncoderCount;
  R_M_prop.encoder_count_2 = R_M_prop.EncoderCount;
  time_2 = millis();

  time_diff = time_2 - time_1;

  L_M_prop.cur_speed = (((L_M_prop.encoder_count_2 - L_M_prop.encoder_count_1) / CYCLE_ENCODER_COUNT * M_PI * WHEEL_DIAMETER) / time_diff) * 1000;
  R_M_prop.cur_speed = (((R_M_prop.encoder_count_2 - R_M_prop.encoder_count_1) / CYCLE_ENCODER_COUNT * M_PI * WHEEL_DIAMETER) / time_diff) * 1000;

  double vl;
  double vr;
  if (L_M_prop.direction == -1) {
    vl = -L_M_prop.cur_speed;
  } else {
    vl = L_M_prop.cur_speed;

  }

  if (R_M_prop.direction == -1) {
    vr = -R_M_prop.cur_speed;
  } else {
    vr = R_M_prop.cur_speed;
  }

  double new_vx = (vl + vr) / 2.0;
  double new_vth = (-vl + vr) / BASE_WIDTH;

  raw_vx = new_vx;
  raw_vth = new_vth;
  
  double alpha1 = 0.2;
  double alpha2 = 0.2;
  current_vx = (1 - alpha1) * current_vx + alpha1 * new_vx;
  current_vth = (1 - alpha2) * current_vth + alpha2 * new_vth;

  
  linear_vel_error = (vx - current_vx);
  angular_vel_error = (vth - current_vth);

  float base_angle_change = linear_kp * linear_vel_error + linear_kd * (linear_vel_error - last_linear_vel_error) + linear_ki * linear_vel_error_sum;
  float rot_speed_change = angular_kp * angular_vel_error + angular_ki * angular_vel_error_sum;

  linear_vel_error_sum += linear_vel_error;
  angular_vel_error_sum += angular_vel_error;

  last_linear_vel_error = linear_vel_error;

  linear_vel_error_sum = constrain(linear_vel_error_sum, -100, 100);
  angular_vel_error_sum = constrain(angular_vel_error_sum, -1000, 1000);

  base_angle_change = constrain(base_angle_change, -5, 5);
  rot_speed_change = constrain(rot_speed_change, -100, 100);

  L_M_prop.encoder_count_1 = L_M_prop.encoder_count_2;
  R_M_prop.encoder_count_1 = R_M_prop.encoder_count_2;

  
  *base_angle = BASE_ANGLE + base_angle_change;
  
  *rot_speed = rot_speed_change;

  time_1 = time_2;
}

void setMotorProperties() {
  L_M_prop.EncoderCount = 0;
  R_M_prop.EncoderCount = 0;

  L_M_prop.cur_speed = 0;
  R_M_prop.cur_speed = 0;

  L_M_prop.direction = 1;
  R_M_prop.direction = 1;
}


void pid() {
  float current_angle_error = kalmanAngle - base_angle;
  float motor_speed_angle = kp_angle * current_angle_error + kd_angle * (current_angle_error - prev_angle_error) + ki_angle * angle_error_sum;
  float kd_error = kd_angle * (current_angle_error - prev_angle_error);
  prev_angle_error = current_angle_error;
  angle_error_sum = angle_error_sum + current_angle_error;
  angle_error_sum = constrain(angle_error_sum, -100, 100);

  int motor_speed = motor_speed_angle;

  if (abs(current_angle_error) < MAX_ANGLE && abs(current_angle_error) > MIN_ANGLE) {
    setMotorSpeeds(motor_speed - rot_speed, motor_speed + rot_speed);
  }
  else if (abs(rot_speed) > 0) {
    setMotorSpeeds(-rot_speed, rot_speed);
  }
  else {
    setMotorSpeeds(0, 0);
  }


}

void updateIMU(){
  kalmanAngle = getKalmanAngle();
  pitch_angle = kalmanAngle - BASE_ANGLE;

}

float getKalmanAngle() {
  unsigned long cur_time = millis();
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  double dt = cur_time - prev_time;
  double gyro_angle_rate = (gy / 131.0);
  prev_time = cur_time;
  double acc_angle = -atan(((double)ax) / ((double)az)) * 180.0 / M_PI;
  
  if (isnan(acc_angle)) {
    return base_angle;

  } else {
    return kalman.getAngle((float)acc_angle, (float)gyro_angle_rate, (float)dt / 1000.0);
  }
}

void initKalman() {
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  double acc_angle = -atan(((double)ax) / ((double)az)) * 180.0 / M_PI;
  
  if (isnan(acc_angle)) {
    initKalman();
  } else {
    kalman.setAngle(acc_angle);
  }
}


void setMotorSpeeds(int LM_Speed, int RM_Speed) {
  int M1_s = 0;
  int M2_s = 0;
  if (LM_Speed >= 0) {
    M1_s = LM_Speed + LEFT_BASE_MOTOR_SPEED_FWD;
    L_M_prop.direction = 1;
  } else {
    M1_s = LM_Speed - LEFT_BASE_MOTOR_SPEED_BWD;
    L_M_prop.direction = -1;
  }

  if (RM_Speed >= 0) {
    M2_s = RM_Speed + RIGHT_BASE_MOTOR_SPEED_FWD;
    R_M_prop.direction = 1;
  } else {
    M2_s = RM_Speed - RIGHT_BASE_MOTOR_SPEED_BWD;
    R_M_prop.direction = -1;
  }


  LM.setMotorSpeed(M1_s);
  RM.setMotorSpeed(M2_s);

}

void serial_pid(float * kp, float *ki, float*kd, float* base_angle) {
  float kp_step = 0.2;
  float ki_step = 0.02;
  float kd_step = 2;
  float base_angle_step = 0.1;

  if (Serial3.available()) {
    char s_r = Serial3.read();
    if (s_r == 'a') {
      *kp = *kp + kp_step;
    } else if (s_r == 'z') {
      *kp = *kp - kp_step;
    } else if (s_r == 's') {
      *ki = *ki + ki_step;
    } else if (s_r == 'x') {
      *ki = *ki - ki_step;
    } else if (s_r == 'd') {
      *kd = *kd + kd_step;
    } else if (s_r == 'c') {
      *kd = *kd - kd_step;
    } else if (s_r == 'f') {
      *base_angle = *base_angle + base_angle_step;
    } else if (s_r == 'v') {
      *base_angle = *base_angle - base_angle_step;
    }
    Serial3.print(*kp);
    Serial3.print("  ");
    Serial3.print(*ki);
    Serial3.print("  ");
    Serial3.print(*kd);
    Serial3.print("  ");
    Serial3.print(*base_angle);
    Serial3.println("  ");
  }
}


//------ Encoder Related ---------------------------------
int l_last_state = 0;
int r_last_state = 0;

void updateEncoderValues() {
  int l_read = analogRead(L_M_ENCODER);
  int r_read = analogRead(R_M_ENCODER);

  int l_state = 0;
  int r_state = 0;

  int l_valid = 0;
  int r_valid = 0;

  if (l_read > 900) {
    l_state = 1;
    l_valid = 1;
  } else if (l_read < 100) {
    l_state = 0;
    l_valid = 1;
  }

  if (r_read > 900) {
    r_state = 1;
    r_valid = 1;
  } else if (r_read < 100) {
    r_state = 0;
    r_valid = 1;
  }

  if (l_valid == 1 && l_state != l_last_state) {
    L_M_prop.EncoderCount++;
    l_last_state = l_state;
  }

  if (r_valid == 1 && r_state != r_last_state) {
    R_M_prop.EncoderCount++;
    r_last_state = r_state;
  }

}

//-----Joystick------------------------------------
void joystick_read() {
  byte command[4];
  float max_vx = 0.4;
  float max_vth = 1.0;
  
  if (Serial3.available() >= 4) {
    command[0] = Serial3.read();
    if (command[0] == 0xF1) {
      command[1] = Serial3.read();
      command[2] = Serial3.read();
      command[3] = Serial3.read();
      vx = command[1]/255.0*max_vx;
      vth = -(command[2] - 88)/23.0*max_vth;
      
    } else if (command[0] == 0xF2) {
      command[1] = Serial3.read();
      command[2] = Serial3.read();
      command[3] = Serial3.read();
      vx = -command[1]/255.0*max_vx;
      vth = -(command[2] - 88)/23.0*max_vth;
    } else if (command[0] == 0xF3) {
      command[1] = Serial3.read();
      command[2] = Serial3.read();
      command[3] = Serial3.read();
      vx = command[1]/255.0*max_vx;
      vth = -(command[2] - 88)/23.0*max_vth;
      
    }
  }
}

//---------------------------------------------------
