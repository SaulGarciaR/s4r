#include "Rover.h"

#include <roverapi/basic_psys_rover.h>

#include <stdint.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <wiringPiI2C.h>

#include <algorithm>
#include <array>
#include <endian.h>
#include <csignal>

using namespace rover;

Rover* Rover::lastInstance = NULL;

typedef struct {              // Total: 54 bytes
  uint16_t type;              // Magic identifier: 0x4d42
  uint32_t size;              // File size in bytes
  uint16_t reserved1;         // Not used
  uint16_t reserved2;         // Not used
  uint32_t offset;            // Offset to image data in bytes from beginning of file (54 bytes)
  uint32_t dib_header_size;   // DIB Header size in bytes (40 bytes)
  int32_t width_px;           // Width of the image
  int32_t height_px;          // Height of image
  uint16_t num_planes;        // Number of color planes
  uint16_t bits_per_pixel;    // Bits per pixel
  uint32_t compression;       // Compression type
  uint32_t image_size_bytes;  // Image size in bytes
  // int32_t   x_resolution_ppm; // Pixels per meter
  // int32_t   y_resolution_ppm; // Pixels per meter
  // uint32_t  num_colors;       // Number of colors
  // uint32_t  important_colors; // Important colors
} BMPHeader;

typedef struct {
  BMPHeader header;
  std::vector<uint8_t> bmpdata;
} BMPImage;

bool exitS4Rover = false;

/**
 * Signal handler for SIGINT
 * We need this to properly close the I2C file handler on exit
 * Otherwise it cannot re-open it on the next start
 *
 * This functions sets a flag to exit the program
 */
void signalHandler(int signum)
{
  ROS_INFO("SIGINT received");
  // cleanup and close up stuff here
  if (!exitS4Rover)
    exitS4Rover = true;
  else
    exit(signum);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rover");
  ros::NodeHandle nh;
  Rover rover(&nh);
  signal(SIGINT, signalHandler);  // register SIGINT handler

  // run ros, until exit flag is set (by SIGINT handler)
  while (ros::ok() && !exitS4Rover) {
    ros::Rate(20).sleep();
    ros::spinOnce();
  }
  ROS_INFO("Exiting");
  // ros::spin(); // cannot use this, because we need to check exit flag
  // closing the file handler etc. is done in Destructors
  return 0;
}

Rover::Rover(ros::NodeHandle* nh)
    : nodeHandle(*nh),
      rovUltrasonicFront(ROVER_FRONT),
      rovUltrasonicRear(ROVER_REAR),
      rovInfraredFrontLeft(ROVER_FRONT_LEFT),
      rovInfraredFrontRight(ROVER_FRONT_RIGHT),
      rovInfraredRearLeft(ROVER_REAR_LEFT),
      rovInfraredRearRight(ROVER_REAR_RIGHT),
      rovButtonUser(USER_BUTTON),  // create user and shutdown button
      rovButtonShutdown(SHUTDOWN_BUTTON)
{
  lastInstance = this;
  gy521.initialize();
  gy521.calibrate();

  // One-time must-call before every rover application
  rovBase.initialize();
  // Initialize driving
  rovDriving.initialize();
  rovDriving.setSpeed(HIGHEST_SPEED);
  rovDriving.stopRover();
  // Initialize sensors
  rovUltrasonicFront.initialize();
  rovUltrasonicRear.initialize();
  rovInfraredFrontLeft.initialize();
  rovInfraredFrontRight.initialize();
  rovInfraredRearLeft.initialize();
  rovInfraredRearRight.initialize();

  rovBuzzer.initialize();
  rovBuzzer.setBuzzerOff();
  rovButtonUser.initialize();  // also initialize normally, because we need to read the pin
  rovButtonUser.initializeInterrupt(cbButtonUser);
  rovButtonShutdown.initialize();
  rovButtonShutdown.initializeInterrupt(cbButtonShutdown);
  
  // TODO Relative path
  ROS_INFO("Display ROS logo");
  this->displayBmp("/home/pi/s4rover/src/s4rover_bridge/Ros.bmp");

  // ROS

  // Set up publishers for sensor data
  pubSonicFront = nodeHandle.advertise<sensor_msgs::Range>("sonic_front", 5, false);
  pubSonicRear = nodeHandle.advertise<sensor_msgs::Range>("sonic_rear", 5, false);
  pubIrFrontLeft = nodeHandle.advertise<sensor_msgs::Range>("ir_front_left", 5, false);
  pubIrFrontRight = nodeHandle.advertise<sensor_msgs::Range>("ir_front_right", 5, false);
  pubIrRearLeft = nodeHandle.advertise<sensor_msgs::Range>("ir_rear_left", 5, false);
  pubIrRearRight = nodeHandle.advertise<sensor_msgs::Range>("ir_rear_right", 5, false);
  pubImu = nodeHandle.advertise<sensor_msgs::Imu>("imu", 5, false);
  pubGyro = nodeHandle.advertise<std_msgs::Float32>("gyro", 5, false);

  pubButtonUser = nodeHandle.advertise<std_msgs::Bool>("button_user", 5, false);
  pubButtonShutdown = nodeHandle.advertise<std_msgs::Bool>("button_shutdown", 5, false);

  // Timers for sensor data publication
  timerPubImu = nodeHandle.createTimer(ros::Duration(1), &Rover::cbPubImu, this);
  timerPubGyro = nodeHandle.createTimer(ros::Duration(1), &Rover::cbPubGyro, this);
  timerPubSonic = nodeHandle.createTimer(ros::Duration(1), &Rover::cbPubSonic, this);
  timerPubInfrared = nodeHandle.createTimer(ros::Duration(1), &Rover::cbPubInfrared, this);
  timerToneLength =
      nodeHandle.createTimer(ros::Duration(2), &Rover::cbTimerToneLength, this, true, false);

  // Subscriber for command velocities
  subCmdVel = nodeHandle.subscribe<geometry_msgs::Twist>("cmd_vel", 5, &Rover::cbCmdVel, this);
  subBuzz = nodeHandle.subscribe<std_msgs::Int16>("cmd_buzz", 5, &Rover::cbBuzz, this);

  // Dynamic reconfigure
  cfgServer.setCallback(boost::bind(&Rover::cbConfig, this, _1, _2));

  ROS_INFO("Rover ready");
}

Rover::~Rover() { rovDriving.stopRover(); }
void Rover::cbConfig(Config& cfg, uint32_t /*level*/)
{
  ROS_INFO("Received new configuration.");

  this->cfg = cfg;
  timerPubImu.setPeriod(ros::Duration(1 / cfg.pubRateImu), false);
  timerPubGyro.setPeriod(ros::Duration(1 / cfg.pubRateGyro), false);
  timerPubSonic.setPeriod(ros::Duration(1 / cfg.pubRateSonic), false);
  timerPubInfrared.setPeriod(ros::Duration(1 / cfg.pubRateInfrared), false);
}

void Rover::cbPubImu(const ros::TimerEvent&)
{
  sensor_msgs::Imu msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "base_link";

  msg.linear_acceleration.x = gy521.getAccelX();
  msg.linear_acceleration.y = gy521.getAccelY();
  msg.linear_acceleration.z = gy521.getAccelZ();

  msg.angular_velocity.x = gy521.getGyroX();
  msg.angular_velocity.y = gy521.getGyroY();
  msg.angular_velocity.z = gy521.getGyroZ();

  // TODO Check correctness
  auto rpy = gy521.getRPY();
  // ROS_INFO_THROTTLE(0.5, "Roll=%f\tPitch=%f\tYaw=%f", rpy[0], rpy[1], rpy[2]);
  tf2::Quaternion q;
  q.setRPY(rpy[0], rpy[1], rpy[2]);
  msg.orientation = tf2::toMsg(q);

  pubImu.publish(msg);
}

void Rover::cbPubGyro(const ros::TimerEvent&)
{
  std_msgs::Float32 msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "base_link";

  msg.data = gy521.getAngleZ();

  pubGyro.publish(msg);
}

void Rover::cbPubSonic(const ros::TimerEvent&)
{
  sensor_msgs::Range msgRange;
  msgRange.radiation_type = sensor_msgs::Range::ULTRASOUND;
  msgRange.field_of_view = 0.2616;  // 15°
  msgRange.min_range = 0.02;
  msgRange.max_range = 4.0;  // according to datasheet, right now max is 0.4m

  msgRange.header.frame_id = "sonic_front";
  msgRange.header.stamp = ros::Time::now();
  msgRange.range = rovUltrasonicFront.read() / 100.0;
  pubSonicFront.publish(msgRange);

  msgRange.header.frame_id = "sonic_rear";
  msgRange.range = rovUltrasonicRear.read() / 100.0;
  pubSonicRear.publish(msgRange);
}

void Rover::cbPubInfrared(const ros::TimerEvent&)
{
  sensor_msgs::Range msgRange;
  msgRange.radiation_type = sensor_msgs::Range::INFRARED;
  msgRange.field_of_view = 0.2616;
  msgRange.min_range = 0.04;
  msgRange.max_range = 0.155;
  msgRange.header.stamp = ros::Time::now();

  msgRange.header.frame_id = "ir_front_left";
  msgRange.range = calculateInfraredRange(rovInfraredFrontLeft.read());
  pubIrFrontLeft.publish(msgRange);

  msgRange.header.frame_id = "ir_front_right";
  msgRange.range = calculateInfraredRange(rovInfraredFrontRight.read());
  pubIrFrontRight.publish(msgRange);

  msgRange.header.frame_id = "ir_rear_left";
  msgRange.range = calculateInfraredRange(rovInfraredRearLeft.read());
  pubIrRearLeft.publish(msgRange);

  msgRange.header.frame_id = "ir_rear_right";
  msgRange.range = calculateInfraredRange(rovInfraredRearRight.read());
  pubIrRearRight.publish(msgRange);
}

void Rover::cbBuzz(const std_msgs::Int16::ConstPtr& msg)
{
  int freq = msg->data;
  rovBuzzer.setBuzzerFrequency(freq);
  rovBuzzer.setBuzzerOn();
  timerToneLength.stop();
  timerToneLength.start();
}

void Rover::cbTimerToneLength(const ros::TimerEvent&) { rovBuzzer.setBuzzerOff(); }

void Rover::cbCmdVel(const geometry_msgs::Twist::ConstPtr& msg)
{
  float linear = msg->linear.x;
  float angular = msg->angular.z;

  // Apply velocity limits (std::clamp doesn't compile for some reason)
  linear = (linear <= -cfg.velMax ? -cfg.velMax : (linear <= cfg.velMax ? linear : cfg.velMax));
  angular = (angular <= -cfg.omegaMax ? -cfg.omegaMax
                                      : (angular <= cfg.omegaMax ? angular : cfg.omegaMax));
  diffDrive(linear, angular);
}

void Rover::cbButtonUser()
{
  // ROS_INFO("User Button has been pressed");
  if (lastInstance == NULL) {
    ROS_ERROR("No instance of Rover found in button interrupt handler");
  }
  else {
    std_msgs::Bool msg;
    msg.data = !lastInstance->rovButtonUser.readButton();  // invert logic (pressed = true)
    lastInstance->pubButtonUser.publish(msg);
  }
}

void Rover::cbButtonShutdown()
{
  // ROS_INFO("Shutdown button has been pressed");
  if (lastInstance == NULL) {
    ROS_ERROR("No instance of Rover found in button interrupt handler");
  }
  else {
    std_msgs::Bool msg;
    msg.data = !lastInstance->rovButtonShutdown.readButton();  // invert logic (pressed = true)
    lastInstance->pubButtonShutdown.publish(msg);
  }
}

void Rover::diffDrive(const float& v, const float& omega)
{
  // Get wheel velocities [rad/s]
  float wheelVelLeft = (v - (omega * wheelSeparation / 2)) / wheelRadius;
  float wheelVelRight = (v + (omega * wheelSeparation / 2)) / wheelRadius;

  // Extract wheel spinning direction
  int8_t dirLeft = (wheelVelLeft >= 0) ? FORWARD : BACKWARD;
  int8_t dirRight = (wheelVelRight >= 0) ? FORWARD : BACKWARD;
  wheelVelLeft = std::fabs(wheelVelLeft);
  wheelVelRight = std::fabs(wheelVelRight);

  // Convert wheel velocities to PWM value (max is equiv. to HIGHEST_SPEED=480)
  uint16_t pwmL = static_cast<uint16_t>(std::round((wheelVelLeft / maxWheelVel) * HIGHEST_SPEED));
  uint16_t pwmR = static_cast<uint16_t>(std::round((wheelVelRight / maxWheelVel) * HIGHEST_SPEED));

  // Send to PWM
  // basic_psys_rover.c has left and right mixed up... no joke.
  runside(RIGHT, dirLeft, pwmL);
  runside(LEFT, dirRight, pwmR);
}

float Rover::calculateInfraredRange(const float& raw)
{
  if (raw > 61) {
    return 0.16;
  }
  if (raw > 50.57) {
    return interpolate(50.57, 61.0, raw, 0.14, 0.155);
  }
  if (raw > 44.57) {
    return interpolate(44.57, 50.57, raw, 0.13, 0.14);
  }
  if (raw > 39.43) {
    return interpolate(39.43, 44.57, raw, 0.12, 0.13);
  }
  if (raw > 35) {
    return interpolate(35, 39.43, raw, 0.11, 0.12);
  }
  if (raw > 30.57) {
    return interpolate(30.57, 35, raw, 0.10, 0.11);
  }
  if (raw > 26.14) {
    return interpolate(26.14, 30.57, raw, 0.09, 0.10);
  }
  if (raw > 22.57) {
    return interpolate(22.57, 26.14, raw, 0.08, 0.09);
  }
  if (raw > 19.14) {
    return interpolate(19.14, 22.57, raw, 0.07, 0.08);
  }
  if (raw > 16.14) {
    return interpolate(16.14, 19.14, raw, 0.06, 0.07);
  }
  if (raw > 13.71) {
    return interpolate(13.71, 16.14, raw, 0.05, 0.06);
  }
  if (raw > 11.28) {
    return interpolate(11.28, 13.71, raw, 0.04, 0.05);
  }
  if (raw > 10) {
    return interpolate(10, 11.28, raw, 0.03, 0.04);
  }
  else {
    return 0;
  }
}

float Rover::interpolate(const float& xMin, const float& xMax, const float& x, const float& yMin,
                         const float& yMax)
{
  // TODO check if we can compile with c++20 since it offers std::lerp
  float xRange = xMax - xMin;
  float yRange = yMax - yMin;

  float deltaX = x - xMin;
  float slope = yRange / xRange;

  return deltaX * slope + yMin;
}

bool Rover::displayBmp(const char* filename)
{
  BMPImage img;

  FILE* bmp = fopen(filename, "rb");
  if (!bmp) {
    ROS_INFO("%s did not open", filename);
    return false;
  }
  else {
    // read header values
    fread((void*)&img.header.type, sizeof(uint16_t), 1, bmp);
    fread((void*)&img.header.size, sizeof(uint32_t), 1, bmp);
    fread((void*)&img.header.reserved1, sizeof(uint16_t), 1, bmp);
    fread((void*)&img.header.reserved2, sizeof(uint16_t), 1, bmp);
    fread((void*)&img.header.offset, sizeof(uint32_t), 1, bmp);
    fread((void*)&img.header.dib_header_size, sizeof(uint32_t), 1, bmp);
    fread((void*)&img.header.width_px, sizeof(uint32_t), 1, bmp);
    fread((void*)&img.header.height_px, sizeof(uint32_t), 1, bmp);
    fseek(bmp, 2, SEEK_CUR);
    fread((void*)&img.header.bits_per_pixel, sizeof(uint16_t), 1, bmp);
    fseek(bmp, 4, SEEK_CUR);
    fread((void*)&img.header.image_size_bytes, sizeof(uint32_t), 1, bmp);

    // create buffer for image data
    img.bmpdata = std::vector<uint8_t>(img.header.image_size_bytes);

    // go to where the image data starts, after header
    int setOffset = fseek(bmp, img.header.offset, SEEK_SET);

    // read image data
    int imgbytes = fread((void*)&img.bmpdata[0], 1, img.header.image_size_bytes, bmp);

    // close file
    fclose(bmp);

    // make temporary publisher to publish image
    imgTest = nodeHandle.advertise<sensor_msgs::Image>("cmd_disp_img", 1, true);
    sensor_msgs::ImagePtr msg(new sensor_msgs::Image);
    msg->data = img.bmpdata;
    msg->height = img.header.height_px;
    msg->width = img.header.width_px;
    msg->step = img.header.width_px / 8 * img.header.bits_per_pixel;
    msg->encoding = "mono1";
    imgTest.publish(msg);

    return true;
  }
}
