#ifndef ROVER_H
#define ROVER_H

#include <s4rover_bridge/DisplayText.h> // custom Display  msg type
#include <s4rover_bridge/RoverConfig.h>

// Rover
#include <roverapi/rover_api.hpp>
#include <roverapi/rover_button.hpp>
#include <roverapi/rover_buzzer.hpp>
#include <roverapi/rover_driving.hpp>
#include <roverapi/rover_grooveultrasonic.hpp>
#include <roverapi/rover_gy521.hpp>
#include <roverapi/rover_hcsr04.hpp>
#include <roverapi/rover_hmc5883l.hpp>
#include <roverapi/rover_infraredsensor.hpp>

// ROS
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>

class Rover {
 public:
  using Config = s4rover_bridge::RoverConfig;

  /** Constructor
   */
  Rover(ros::NodeHandle *nh);

  ~Rover();

 private:
  // rover-app
  rover::RoverBase rovBase;                          /// Base class of the rover
  rover::RoverDriving rovDriving;                    /// Interface for driving the rover
  rover::RoverHCSR04 rovUltrasonicFront;             /// Ultrasonic sensor in the front
  rover::RoverHCSR04 rovUltrasonicRear;              /// Ultrasonic sensor in the back
  rover::RoverInfraredSensor rovInfraredFrontLeft;   /// Infrared sensor in the front left
  rover::RoverInfraredSensor rovInfraredFrontRight;  /// Infrared sensor in the front right
  rover::RoverInfraredSensor rovInfraredRearLeft;    /// Infrared sensor in the rear left
  rover::RoverInfraredSensor rovInfraredRearRight;   /// Infrared sensor in the rear right
  rover::RoverGY521 gy521;                           /// Accelerometer/Gyroscope
  rover::RoverBuzzer rovBuzzer;                      /// Buzzer
  rover::RoverButton rovButtonUser;
  rover::RoverButton rovButtonShutdown;

  // ROS
  ros::NodeHandle nodeHandle;  /// Node handle

  ros::Publisher pubCompass;         /// Publisher for the compass
  ros::Publisher pubImu;             /// Publisher for GY521 data
  ros::Publisher pubGyro;             /// Publisher for GY521 data
  ros::Timer timerPubImu;            /// Timer for publishing GY521 data.
  ros::Timer timerPubGyro;            /// Timer for publishing GY521 data.
  ros::Publisher pubSonicFront;      /// Publisher for front ultrasonic sensor
  ros::Publisher pubSonicRear;       /// Publisher for rear ultrasonic sensor
  ros::Timer timerPubSonic;          /// Timer for publishing ultrasonic sensors.
  ros::Publisher pubIrFrontLeft;     /// Publisher for front left infrared ranges
  ros::Publisher pubIrFrontRight;    /// Publisher for front right infrared ranges
  ros::Publisher pubIrRearLeft;      /// Publisher for rear left infrared ranges
  ros::Publisher pubIrRearRight;     /// Publisher for rear right infrared ranges
  ros::Timer timerPubInfrared;       /// Timer for publishing infrared sensors.
  ros::Timer timerToneLength;        /// Timer for duration of a buzzer tone
  ros::Publisher pubButtonUser;      /// Publisher for User Button
  ros::Publisher pubButtonShutdown;  /// Publisher for Shutdown Button
  ros::Publisher imgTest;

  ros::Subscriber subCmdVel;         /// Subscriber for velocity commands
  ros::Subscriber subBuzz;           /// subscriber for buzzer commands

  dynamic_reconfigure::Server<Config> cfgServer;  /// Server for config changes
  Config cfg;                                     /// The current config from dynamic_reconfigure

  // Misc
  const float wheelRadius = 0.03125;                   /// Wheel radius [m]
  const float wheelSeparation = 0.2;                   /// Wheel separation [m]
  const float physMaxVel = 0.254;                      /// Highest possible rover speed [m/s]
  const float maxWheelVel = physMaxVel / wheelRadius;  /// Maximum wheel velocitiy [rad/s]

  /**
   * this is needed for the static button interrupt handlers
   *    The interrupt callbacks must be static, but need and instance
   *    of the rover (to read the new button state)
   *    The last Rover object created is saved in this static pointer
   *    The button callbacks use this static pointer
   */
  static Rover *lastInstance;

  /** Callback for dynamic_reconfigure updates
   * @param cfg The new config
   * @param level The level
   */
  void cbConfig(Config &cfg, uint32_t level);

  /** Read and publish GY521 data
   */
  void cbPubImu(const ros::TimerEvent &);
  
  /** Read and publish GY521 data
   */
  void cbPubGyro(const ros::TimerEvent &);

  /** Read and publish ultrasonic sensors
   */
  void cbPubSonic(const ros::TimerEvent &);

  /** Read and publish ultrasonic sensors
   */
  void cbPubInfrared(const ros::TimerEvent &);

  /** Callback for command velocities
   * @param msg The message.
   */
  void cbCmdVel(const geometry_msgs::Twist::ConstPtr &msg);

  /**
   * Callback for buzzer commands
   * @param msg The message; a 16 bit int: frequency for buzzer
   */
  void cbBuzz(const std_msgs::Int16::ConstPtr &msg);

  /**
   * Timer callback to stop Buzzer
   * @param Timer event
   */
  void cbTimerToneLength(const ros::TimerEvent &);

  /**
   * Callback when user button is pressed (rising or falling edge)
   * publishes new button state to 'button_user'
   */
  static void cbButtonUser();

  /**
   * Callback when shutdown button is pressed (rising or falling edge)
   * publishes new button state to 'button_shutdown'
   */
  static void cbButtonShutdown();

  /**
   * Controls the robot's motors.
   * The implementation in rover-api/basic_psys_rover is not that good,
   * so here is a better one.
   *
   * @param v Desired linear velocity [m/s]
   * @param omega Desired angular velocity [rad/s]
   */
  void diffDrive(const float &v, const float &omega);

  /**
   * Estimate an infrared reading that is closer to the truth.
   *
   * @param raw original reading of the infrared sensor [cm]
   * @return estimated real distance [m]
  */
  float calculateInfraredRange(const float &raw);

  /**
   * Linear 2D interpolation for correction of infrared readings.
   *
   * @param xMin Lower input bound
   * @param xMax Upper input bound
   * @param x Input value
   * @param yMin Lower target bound
   * @param yMax Upper target bound
   * @return Interpolated value
  */
  float interpolate(const float &xMin, const float &xMax, const float &x, const float &yMin,
                    const float &yMax);

  /**
   * shows a Bitmap file on the display
   * The BMP must have the correct format. The display size is 128*64 pixels
   * Each pixel is one bit (1: WHITE, 0: BLACK)
   * *Note*: The image is currently shown upside down, because this
   *          function reads the file from front to back, while most
   *          BMPs are stored back to front. Use upside down images as
   *          a workaround.
   * @param filename (absolute) Path to BMP-file
   */
  bool displayBmp(const char *filename);

};

#endif  // ROVER_H
