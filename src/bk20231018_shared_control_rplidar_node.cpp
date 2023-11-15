/* Obstacle avoidance shared control algo for usage on teleoperated robots
Inputs:
- lidar: 2D rplidar
- joypad: Sony DS4: right joystick data
- odometry: linear velocity, angular velocity

Tested on Ubuntu 20.04.1 + ROS Noetic

*/ 

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

#include <vector>

// a point on an obstacle identified by lidar
struct PointObst {
   double xUbase_linkLobst, yUbase_linkLobst, distUbase_linkLobst, angleUbase_linkLobst, intensity;
}; //struct PointObst

class SharedControl_rplidar {
   private:
   // // Part 1: shared control
   // choose: tuning param 
   double RMmin = 0.01/10;  // [m/s / rad/s = m]
   double RMmax = 0.98/0.1; // [m/s / rad/s = m]
   
   // // Part 2: physical qtts
   const double ROSbot_absvmax_fwd {.1}; //[m/s] robot maximum linear velocity when advancing forward
   const double ROSbot_absvmax_bwd  {ROSbot_absvmax_fwd}; //m
   const double ROSbot_chassisWidth {0.24}; //[m]
   const double ROSbot_absomegamax     {3.14/6.0}; //[rad/s]
   const double ROSbot_distSensorToEdgeOfRobot {0.08} ;//m

   // // Part3: data members
   double vd_nz {0.0}, omegad_nz {0.0}; //normalized values
   double v {0.0},omega {0.0}; //odometry: v [m/s]; omega [rad/s]

   // define a ring of laser data
   std::vector<PointObst> scan; //scan[id]

   // ROS
   ros::NodeHandle nh_; // in C++ the naming convention of a variable with an underscore usually indicates a private member variable: see B2017Newman, Comment p55
   ros::Subscriber rplidar_sub_, joypad_sub_, odom_sub_;
   ros::Publisher  cmd_vel_pub_;

   // create a temporary 'empty' PointObst
   PointObst temp_onePointObst {nan(""),nan(""),nan(""),nan(""),nan("")};

   // member function
   void joypadSubscCallback(const sensor_msgs::JoyConstPtr& msg) {
      // using axis convention (v_joy,omega_joy) from article [TeZhCa2020] i.e. art_whc2
      double x_joy = msg->axes[4]; //[-] \in [-1,1] the 5th element inside array axes represents advancing bwd/fwd user intention (max fwd = +1; max bwd = -1)
      double y_joy = msg->axes[3]; // \in [-1,1]  the 4th element inside array axes represents turning right/left user intention (max left = +1; max right=-1)

      // sanity check 
      if (isSain(vd_nz) && isSain(omegad_nz))  {    
	  // // implement the same logic as in $rosrun teleop_twist_keyboard teleop_twist_keyboard.py 
	  // Q1 + Q2
	  if (x_joy>=0) {
	     vd_nz     = x_joy; //[-]
	     omegad_nz = y_joy; //[-]
	  } else {// i.e. if (x_joy<0)  covering Q3 + Q4
	     vd_nz     =  x_joy; //[-]
	     omegad_nz = -y_joy; //[-]
	  } // if (.)

	  ROS_INFO("Joypad: vd_nz = %.2f [-]; omegad_nz = %.2f [-]", vd_nz,omegad_nz); //V&V
      } // if (.)
      //otherwise decision to keep the previous meaningful values and wait until we receive new meaningful data


   } // void joypadSubscCallback(.)

   // member function
   void odometrySubscCallback(const nav_msgs::Odometry::ConstPtr& msg) {
      double v_temp = msg->twist.twist.linear.x;  //[m/s]
      double omega_temp = msg->twist.twist.angular.z; //[rad/s]
      //ROS_INFO("Raw     : Lin velo: v = %.2f [m/s]; Angular velo: omega = %.2f [rad/s]",v,omega); //V&V

      // sanity check + postprocess: saturate: safety reasons (data consistency)
      if (isSain(v_temp) && isSain(omega_temp))  {
	  v = std::min(ROSbot_absvmax_fwd, std::max(ROSbot_absvmax_bwd, v_temp)); //overwrite
          omega = std::min(ROSbot_absomegamax, std::max(-ROSbot_absomegamax, omega_temp)); //overwrite
      } // if (.)
      //otherwise decision to keep the previous meaningful values and wait until we receive new meaningful data

      //ROS_INFO("Odometry: v = %.2f [m/s]; omega = %.2f [rad/s]",v,omega); //V&V
   } //void odometrySubscCallback(.)

   // member function: helper/utility function
   void publish_cmd_vel(double vr_nz, double omegar_nz) {
      geometry_msgs::Twist cmd_vel_msg;

      // ROS
      cmd_vel_msg.linear.x  = vr_nz > 0.0 ? vr_nz*ROSbot_absvmax_fwd : vr_nz*ROSbot_absvmax_bwd;   //[m/s]
      cmd_vel_msg.angular.z = omegar_nz*ROSbot_absomegamax;  //[rad/s] symmetrical max angular velocity i.e. there is no distinction between max left and max right

      // conclude
      //ROS_INFO("Will publish cmd_vel_msg.linear.x=%f, cmd_vel_msg.angular.z=%f", cmd_vel_msg.linear.x, cmd_vel_msg.angular.z);
      cmd_vel_pub_.publish(cmd_vel_msg);
    } //void publish_cmd_vel(.)

   


   // member function: helper/utility function
   bool isSain(double temp) {
      if (std::isnan(temp) || std::isinf(temp)) return false;
	
      return true;  
   } // void isSain(.)

   // member function
   void sharedControlAlgo() {
     // Output: (vr_nz,omegar_nz)
     ROS_INFO("Entered sharedControlAlgo()");

     // ini: by default assume no need to enable ASC
     double vr_nz {vd_nz};
     double omegar_nz {omegad_nz};

     // // case1. vehicle advancing straight ahead wo rotation
     // // case2. vehicle rotating
     // simplyfication hypothesis for the sake of simplifying the code: treat case1 as a particular situation of case2 by clipping omegad to a small non-zero value

     // define situation when to apply SC
     double RM = abs(vr_nz/omegar_nz); //[m]
     if (isSain(RM)) if (RM>=RMmin) { // then eligible to apply SC
 	
	// handle case1 as a particular situation of case2: adhoc clipping     
        if (RM>=RMmax) {
           // clip omegar_nz
	   omegar_nz = signNotZero(omegar_nz) * abs(vr_nz)/RMmax; //overwrite
	} // if (.)

        // // find nearest obstacle
        temp_dist_nearest_obstacle = RMmax; // a large number


     } // if (.)	     
     // // conclude
     publish_cmd_vel(vr_nz, omegar_nz);

   } // void sharedControlAlgo() 

   // member function: helper/utility function
   double signNotZero(double temp) {
	   return (temp >= 0.0) ? 1.0 : -1.0;
   } // double signNotZero(.)


   // member function: helper/utility function
   void rplidarSubscCallback(const sensor_msgs::LaserScanConstPtr& msg) { // code highly inspired from ../sync/code_UoM/ROS/rviz_markers/../tf_laserscanReexprWrtMap_fasterCode.cpp
    // choose
    bool doDebug = false;

    // // process laserscan data
    // ini
    size_t id = -1;

    for (auto elem:msg->ranges) { // reason for using foreach loop is because I don't know the size of msg->intensities[]
       id++; //reconstruct index
  
       // express the location of the obstacle point pLpoint  as (alphaUlaser_frameLpoint, dUlaser_frameLpoint)
      double alphaUlaser_frameLpoint = msg->angle_min + id*msg->angle_increment; //[rad]
      double dUlaser_frameLpoint = msg->ranges[id];//[m] distance

      // sanity check
      if (!isSain(alphaUlaser_frameLpoint) || !isSain(dUlaser_frameLpoint)) continue; // cannot make use of meaningless data

if (doDebug) {      
ROS_INFO("alphaUlaser_frameLpoint = %.2f [rad]; dUlaser_frameLpoint = %.2f [m]", alphaUlaser_frameLpoint,dUlaser_frameLpoint); //sleep(1);
} // if (.)

      // reexpress/convert the obstacle point pLpoint  to/as (xUlaser_frameLpoint, yUlaser_frameLpoint)
      double xUlaser_frameLpoint = cos(alphaUlaser_frameLpoint) * dUlaser_frameLpoint;
      double yUlaser_frameLpoint = sin(alphaUlaser_frameLpoint) * dUlaser_frameLpoint;

if (doDebug) {
ROS_INFO("xUlaser_frameLpoint = %.2f [m]; yUlaser_frameLpoint = %.2f [m]", xUlaser_frameLpoint,yUlaser_frameLpoint); // sleep(1);
} // if (.)

      // transform the point p expressed wrt/ito "base_link" frame instead of laser_frame: apply formula pUbase_linkLpoint := [xUbase_linkLpoint; yUbase_linkLpoint] = RUbase_linkLlaser_frame * pUlaser_frame, where origins coincide of "base_link" and "laser_frame"
      double theta_temp = M_PI;
      double xUbase_linkLpoint = cos(theta_temp)*xUlaser_frameLpoint - sin(theta_temp)*yUlaser_frameLpoint;
      double yUbase_linkLpoint = sin(theta_temp)*xUlaser_frameLpoint + cos(theta_temp)*yUlaser_frameLpoint;
       
if (doDebug) {
ROS_INFO("xUbase_linkLpoint = %.2f [m]; yUbase_linkLpoint = %.2f [m]", xUbase_linkLpoint,yUbase_linkLpoint ); //sleep(1);
} // if (.)

    // conclude: store
    if (isSain(xUbase_linkLpoint) && isSain(yUbase_linkLpoint))  {
        PointObst p;
	p.xUbase_linkLobst = xUbase_linkLpoint; //[m]
	p.yUbase_linkLobst = yUbase_linkLpoint; //[m]

        p.distUbase_linkLobst = dUlaser_frameLpoint; //because frames "base_link" and "laser_frame" share the same origin
	
	p.angleUbase_linkLobst = M_PI + alphaUlaser_frameLpoint; //
	//ensure angle is between [-M_PI,M_PI)
        if (p.angleUbase_linkLobst>=M_PI) p.angleUbase_linkLobst = p.angleUbase_linkLobst - 2*M_PI;
	if (p.angleUbase_linkLobst<-M_PI) p.angleUbase_linkLobst = p.angleUbase_linkLobst + 2*M_PI;
 
	p.intensity = msg->intensities[id];

if (doDebug) {
ROS_INFO("p.xUbase_linkLobst = %.2f [m]; p.yUbase_linkLobst = %.2f [m]; p.distUbase_linkLobst = %.2f [m]; p.angleUbase_linkLobst = %.2f [deg]; p.intensity = %.2f", p.xUbase_linkLobst,p.yUbase_linkLobst,p.distUbase_linkLobst,p.angleUbase_linkLobst,p.intensity);	
ROS_INFO("======="); sleep(1);
} // if (.)

        // conclude
	scan.push_back(p);

    } // if

    } // for (.)

    // call: as soon as we have new info about obstacles
    sharedControlAlgo();

   } // void rplidarSubscCallback(.)





   public:
   // constructor
   explicit SharedControl_rplidar(ros::NodeHandle& nh): nh_(nh), vd_nz(0.0), omegad_nz(0.0) { //wo 'explicit' keyword, single-argument constructors have the potential to be wrongly used as conversion constructors, see B2022Deitel, C++ (3rd), section 11.9
      
      // // ROS subscribers and publishers
      // subscribe to topic /scan  
      rplidar_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan",1, &SharedControl_rplidar::rplidarSubscCallback, this); 

      joypad_sub_   = nh_.subscribe<sensor_msgs::Joy>("/joy", 1, &SharedControl_rplidar::joypadSubscCallback, this);
      odom_sub_     = nh_.subscribe<nav_msgs::Odometry>("/odom", 1, &SharedControl_rplidar::odometrySubscCallback, this);
      cmd_vel_pub_  = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",1);

   } // explicit SharedControl_rplidar(.)

}; // class SharedControl_rplidar


// ++++++++++++++++++++++++++++++++++++++
int main(int argc, char** argv) {

    ros::init(argc, argv, "shared_control_rplidar_node");
    ros::NodeHandle nh;

    SharedControl_rplidar sc(nh);
    ros::spin();

    return EXIT_SUCCESS;
} // int main(.)

