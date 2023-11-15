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
   // // Part0: rplidar A3 properties
   const double rplidar_maxDistRange = 25.0; //[m] cf rplidar A3 datasheet
   const double rplidar_minDistRange = 0.2; //[m] cf rplidar A3 datasheet

   // // Part 1: shared control
   // choose: tuning param 
   const double RMdmin_nz = 0.2/0.9;  //[-]
   
   // // Part 2: physical qtts
   const double ROSbot_absvmax_fwd {0.5}; //[m/s] robot maximum linear velocity when advancing forward
   const double ROSbot_absvmax_bwd  {ROSbot_absvmax_fwd}; //m
   const double ROSbot_chassisWidth {0.24 + 0.08}; //[m] real dimension + inflate for obsta avoidance purpose
   const double ROSbot_absomegamax     {3.14/2.0}; //[rad/s]
   const double ROSbot_distSensorToEdgeOfRobot_fwd {0.08} ;//m
   const double ROSbot_distSensorToEdgeOfRobot_bwd {0.12} ;//m

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

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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
      } else { 
      //otherwise decision to keep the previous meaningful values and wait until we receive new meaningful data
         ROS_INFO("Fct joypadSubscCallback(.): received inconsistent data");
      } // if (.)

   } // void joypadSubscCallback(.)

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   // member function
   void odometrySubscCallback(const nav_msgs::Odometry::ConstPtr& msg) {
      double v_temp = msg->twist.twist.linear.x;  //[m/s]
      double omega_temp = msg->twist.twist.angular.z; //[rad/s]
      //ROS_INFO("Raw     : Lin velo: v = %.2f [m/s]; Angular velo: omega = %.2f [rad/s]",v,omega); //V&V

      // sanity check + postprocess: saturate: safety reasons (data consistency)
      if (isSain(v_temp) && isSain(omega_temp))  {
	  v = std::min(ROSbot_absvmax_fwd, std::max(-ROSbot_absvmax_bwd, v_temp)); //overwrite
          omega = std::min(ROSbot_absomegamax, std::max(-ROSbot_absomegamax, omega_temp)); //overwrite
      } else { 
      //otherwise decision to keep the previous meaningful values and wait until we receive new meaningful data
         ROS_INFO("Fct odometrySubscCallback(.): received inconsistent data");      
      } // if (.)

      //ROS_INFO("Odometry: v = %.2f [m/s]; omega = %.2f [rad/s]",v,omega); //V&V
   } //void odometrySubscCallback(.)

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   // member function: helper/utility function
   double calc_abs_vrMax_nz(double xUobstLco) {  // relation between axes: xUbase_link = xUobst + distSensorToEdgeOfRobot ; 'co' stands for 'closest obstacle'
        // Models the relation between vrUmax_nz and xUobstLco, as a mathematical function
        // Output: vrUmax_nz [-]

        // //[var1] linear fct fct(xUobst)=a*xUobst+b, intersecting the points (xUobst=d0,vrMax=0) and (xUobst=dmax,vrMax=1)
        // Choose
        double d0   = 0.20; //[m] d0 is where fct=fct(xUobst) starts ramping up; d0 defined along xUobst
        double dmax = 0.62; //[m] dmax is the distance-to-obstacle where the SC kicks in; dmax defined along xUobst

        // ini: safe value
        double vrMax_nz = nan("");

        if (xUobstLco<0.0) {
        ROS_ERROR("Please check xUobstLco");
        } else if (xUobstLco<=d0) {
        vrMax_nz = 0.0;
        } else if (xUobstLco<dmax) {
        vrMax_nz = (xUobstLco-d0)/(dmax-d0);
        } else {
        vrMax_nz = 1.0;
        } //if (.)

        //conclude
        return vrMax_nz;
   } // double calc_abs_vrMax_nz(.)


// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   // member function: helper/utility function
   bool isSain(double temp) {
      if (std::isnan(temp) || std::isinf(temp)) return false;
	
      return true;  
   } // void isSain(.)

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   // member function
   void sharedControlAlgo() {
     // Output: (vr_nz,omegar_nz)
     
     // display purpose	   
     ROS_INFO("Entered sharedControlAlgo()");

     // ini: by default assume no need to enable ASC
     double vr_nz {vd_nz};
     double omegar_nz {omegad_nz};


// Testing  fcf nearest_obst_arc_circ(.) on the 4 quadrants joypad
//ROS_INFO("Q1: Nearest obstacle is located at l_arc_nearest_obst=%.2f [m]", nearest_obst_arc_circ(1.0, 1.0) ); sleep(1);// joypad Q1
//ROS_INFO("Q2: Nearest obstacle is located at l_arc_nearest_obst=%.2f [m]", nearest_obst_arc_circ(1.0, -1.0) ); sleep(1); // joypad Q2
//ROS_INFO("Q3: Nearest obstacle is located at l_arc_nearest_obst=%.2f [m]", nearest_obst_arc_circ(-1.0, 1.0) ); sleep(1); // joypad Q3
//ROS_INFO("Q4: Nearest obstacle is located at l_arc_nearest_obst=%.2f [m]", nearest_obst_arc_circ(-1.0, -1.0) ); sleep(1);// joypad Q4

// Testing fct nearest_obst_straight_line() on fwd-bwd motion
//ROS_INFO("fwd: Nearest obstacle is located at d_nearest_obst=%.2f [m]", nearest_obst_straight_line(1.0)); sleep(1);// vehicle moving fwd    
//ROS_INFO("bwd: Nearest obstacle is located at d_nearest_obst=%.2f [m]", nearest_obst_straight_line(-1.0)); sleep(1); // vehicle moving bwd    

// Testing SC algo: by-passing joy and odometry
//omega=0; v=0; vd_nz= 1; omegad_nz=0;// case IB1) robot adv in straight line fwd
//omega=0; v=0; vd_nz=-1; omegad_nz=0;// case IB2) robot adv in straight line bwd
// omega=0; v=0; vd_nz= 1; omegad_nz=1;// case ID) robot adv in circle fwd towards left (joypad Q1)
// omega=0; v=0; vd_nz= 1; omegad_nz=-1;// case ID) robot adv in circle fwd towards right (joypad Q2)
// omega=0; v=0; vd_nz= -1; omegad_nz=1;// case ID) robot adv in circle bwd towards right (joypad Q3)
// omega=0; v=0; vd_nz= -1; omegad_nz=-1;// case ID) robot adv in circle bwd towards right (joypad Q4)
//omega=0; v=1; vd_nz= 1; omegad_nz=1;// case II1) robot adv in straight line fwd, user expressing intention to advance towards the obstacle 
//omega=0; v=-1; vd_nz=-1; omegad_nz=1;// case II2) robot adv in straight line bwd, user expressing intention to advance towards the obstacle 
// omega=2; v=1; vd_nz=1; omegad_nz=1;// case IV with (v,omega) in Q1: robot adv in circle fwd, user expressing intention to advance towards the obstacle 
// omega=-2; v=1; vd_nz=1; omegad_nz=1;// case IV with (v,omega) in Q2: robot adv in circle fwd, user expressing intention to advance towards the obstacle 
// omega=2; v=-1; vd_nz=-1; omegad_nz=1;// case IV with (v,omega) in Q4 corresp to the steady-state motion of a joypad in Q3: robot adv in circle bwd, user expressing intention to advance towards the obstacle 
// omega=-2; v=-1; vd_nz=-1; omegad_nz=1;// case IV with (v,omega) in Q3 corresp to the steady-state motion of a joypad in Q4: robot adv in circle bwd, user expressing intention to advance towards the obstacle 

     // treat first exceptional cases that fall outside of the scope of SC
     if (omegad_nz!=0 && abs(vd_nz/omegad_nz) < RMdmin_nz) { // recovery mode area; decision to modify user input: make a pure rotation instead of what the user requests 
	vr_nz = 0;
        omegar_nz = omegad_nz > 0 ? 1 : -1;	
     } else {
     // // // Part1: compute the distance to the nearest obstacle dist_nearest_obst_temp: in this code, decisions to compute the predicted path/traj using the current (v,omega); exceptionally, from standstill, compute the predicted path/traj using the user input (vd,omegad) 
     // ini
     bool doEnableSC {false}; 
     double dist_nearest_obst_temp {rplidar_maxDistRange};

     // // Below, the following if-cases highlight situations when we want to enable SC and by default leave all other cases outside SC i.e. preserve user's original intention
     // case IB
     if ((omega==0 && v==0) && // i.e. robot at standstill
	(vd_nz !=0 && omegad_nz==0)) { // i.e. user expresses intention to advance in straight line fwd-bwd    
	   // // seek/identify the nearest obstacle ahead 	   
	   double d_nearest_obst = nearest_obst_straight_line(vd_nz);

           // record
	   dist_nearest_obst_temp = d_nearest_obst;

	   doEnableSC = true;
	} // if (vd_nz !=0 && omegad_nz==0)
     // case ID 
     else if ((omega==0 && v==0) && // i.e. robot at standstill
	(vd_nz !=0 && omegad_nz!=0)) { // i.e. user expresses intention to advance in circle fwd-bwd    
	   // // seek/identify the nearest obstacle ahead 	   
	   double l_arc_nearest_obst = nearest_obst_arc_circ(vd_nz, omegad_nz);;
           
	   // record
           dist_nearest_obst_temp = l_arc_nearest_obst;

	   doEnableSC = true;
	} // if (vd_nz !=0 && omegad_nz!=0)
     // case II
     else if ((omega==0 && v!=0) &&  // i.e. robot advancing on straight line 
        (v*vd_nz>0)) { // i.e. signs of vd and v coincide, meaning user expresses intention to advance towards the obstacle, be it in straight line (omegad==0) or in circle (omegad!=0) 
           // // seek/identify the nearest obstacle ahead          
           double d_nearest_obst = nearest_obst_straight_line(v); 

	   // record
           dist_nearest_obst_temp = d_nearest_obst;

           doEnableSC = true;
     } // if (omega==0) && v!=0)
     // case IV
     else if ((omega!=0 && v!=0) && // i.e. robot advancing on circle
        (v*vd_nz>0) &&  // i.e. signs of vd and v coincide, meaning user expresses intention to advance towards the obstacle, be it in straight line (omegad==0) or in circle (omegad!=0) 
	((omegad_nz == 0) || abs(vd_nz/omegad_nz) >= RMdmin_nz)) { // outside recovery area where decision to keep SC disabled iot avoid getting trapped between obstacles wo possibility to escape
	   // // seek/identify the nearest obstacle ahead
           double l_arc_nearest_obst = nearest_obst_arc_circ(v, omega);
//debug
ROS_INFO("  Fct sharedControlAlgo(.): case IV: RM:=v/omega=%.2f [m]; l_arc_nearest_obst=%.2f [m]", v/omega,l_arc_nearest_obst);

           // record
           dist_nearest_obst_temp = l_arc_nearest_obst;

	   doEnableSC = true;
     } // if (omega!=0 && v!=0) 


     // // // Part2: compute/calc (vr_nz,omega_nz) = fct(distUbLobst)
     if (doEnableSC) {
           // // calc (vr_nz,omega_nz) = fct(distUbLobst)
           vr_nz     = std::min(calc_abs_vrMax_nz(dist_nearest_obst_temp - ROSbot_distSensorToEdgeOfRobot_fwd),vd_nz); 

	   // account for the complementary situation possible, namely case IB2) i.e. vd_nz<0
	   if (vd_nz<0)   vr_nz = std::max(-calc_abs_vrMax_nz(dist_nearest_obst_temp - ROSbot_distSensorToEdgeOfRobot_bwd),vd_nz); 

	   // adjust omegad accordingly
           omegar_nz = omegad_nz * vr_nz/vd_nz;

//debug
ROS_INFO("  Fct sharedControlAlgo(.): SC enabled: User input (vd_nz=%.2f,omegad_nz=%.2f); Robot status (v=%.2f,omega=%.2f); SC output (vr_nz=%.2f,omegar_nz=%.2f)", vd_nz,omegad_nz, v,omega, vr_nz,omegar_nz);

     } // if (doEnableSC)
     else ROS_INFO("  Fct sharedControlAlgo(.): SC not enabled: User input (vd_nz=%.2f,omegad_nz=%.2f). Robot status (v=%.2f,omega=%.2f)",vd_nz,omegad_nz, v,omega);
     } // if (omegad_nz!=0 && abs(vd_nz/omegad_nz) < RMdmin_nz) 

     // // conclude
     publish_cmd_vel(vr_nz, omegar_nz);

     // display purpose	   
     ROS_INFO("Exit sharedControlAlgo()"); ROS_INFO("=========================");

   } // void sharedControlAlgo() 

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   // member function: helper/utility function
   double signNotZero(double temp) {
	   return (temp >= 0.0) ? 1.0 : -1.0;
   } // double signNotZero(.)


// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   // member function: helper/utility function
   void rplidarSubscCallback(const sensor_msgs::LaserScanConstPtr& msg) { // code highly inspired from ../sync/code_UoM/ROS/rviz_markers/../tf_laserscanReexprWrtMap_fasterCode.cpp
    // choose
    bool doDebug = false;

    // clear whatever was stored previously
    scan.clear();

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

      // check data consistency in line with manufacturer specs
      if (dUlaser_frameLpoint<rplidar_minDistRange || dUlaser_frameLpoint>rplidar_maxDistRange) continue; // cannot conclude that such data is valid as beyond manufacturer specs

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
ROS_INFO("p.xUbase_linkLobst = %.2f [m]; p.yUbase_linkLobst = %.2f [m]; p.distUbase_linkLobst = %.2f [m]; p.angleUbase_linkLobst = %.2f [deg]; p.intensity = %.2f", p.xUbase_linkLobst,p.yUbase_linkLobst,p.distUbase_linkLobst,rad2deg(p.angleUbase_linkLobst),p.intensity);	
ROS_INFO("======="); sleep(1);
} // if (.)

        // conclude
	scan.push_back(p);

    } // if

    } // for (.)

    // call: as soon as we have new info about obstacles
    sharedControlAlgo();

   } // void rplidarSubscCallback(.)

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   // member function: helper/utility function
   double nearest_obst_straight_line(double v) {
      // Implicit expectation when calling this function: omega=0 and v\neq0, otherwise results are meaningless; actually this function only needs as input the sign of v, it does not really matter the magnitude of v
      // Returns the distance in metres to the nearest obstacle d_nearest_obst, located ahead or behind, depending on the travel direction (v>0 or v<0, respectively)

      // ini
      double d_nearest_obst {rplidar_maxDistRange}; // ini with a large number

      // Check expectation
      if (v!=0) { // then we can proceed to calc d_nearest_obst
         // seek candidates for nearest obstacle
	 for (auto p:scan) {
             // case1: v>0
	     if ((v>0) && 
                 (abs(p.yUbase_linkLobst) <= ROSbot_chassisWidth/2.0 && p.xUbase_linkLobst > 0) &&
                 (p.distUbase_linkLobst < d_nearest_obst)) 
		    d_nearest_obst = p.distUbase_linkLobst; // assumption that p.angleUbase_linkLobst = 0 otherwise we'd need to calc the projection of p onto yUbase_link-axis				
             // case2: v<0
	     else if ((v<0) &&
                 (abs(p.yUbase_linkLobst) <= ROSbot_chassisWidth/2.0 && p.xUbase_linkLobst < 0) &&
                 (p.distUbase_linkLobst < d_nearest_obst)) 
		     d_nearest_obst = p.distUbase_linkLobst; // assumption that p.angleUbase_linkLobst = 0 otherwise we'd need to calc the projection of p onto yUbase_link-axis 

	 } // for (.)
      } // if (.)       
      else ROS_ERROR("ctdr: Error: Please check nearest_obst_straight_line(.)"); 

      return d_nearest_obst; 
   } // double nearest_obst_straight_line(.) 

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   // member function: helper/utility function 
   double nearest_obst_arc_circ(double v, double omega) {
      // Implicit expectation when calling this function: v \neq0 and omega \neq0, otherwise results might be meaningless
      // Returns the length of an arc to the nearest obstacle l_arc_nearest_obst, depending on quadrant=quadrant(v,omega) 

// choose	   
bool doDebug = false;

      // ini	
      double l_arc_nearest_obst {M_PI*rplidar_maxDistRange/2.0}; // ini with a large number 
   
      // Check expectation
      if (v !=0 && omega !=0 ) {
        // seek candidates for nearest obstacle
         for (auto p:scan) {

             // case joypad in Q1: v>0 and omega>0
             if (v>0 && omega>0) { 
                if ((pow(v/omega - ROSbot_chassisWidth/2.0, 2.0) <= pow(v/omega-p.yUbase_linkLobst, 2.0) + pow(p.xUbase_linkLobst, 2.0)) && (pow(v/omega-p.yUbase_linkLobst, 2.0) + pow(p.xUbase_linkLobst, 2.0) <=  pow(v/omega + ROSbot_chassisWidth/2.0, 2.0))) {
if (doDebug)
ROS_INFO("  Fct nearest_obst_arc_circ(.): Q1: investigating the point: xUbase_linkLobst=%.2f; yUbase_linkLobst=%.2f; distUbase_linkLobst=%.2f; angleUbase_linkLobst=%.2f [deg]", p.xUbase_linkLobst, p.yUbase_linkLobst, p.distUbase_linkLobst, rad2deg(p.angleUbase_linkLobst));			
                   // def
		   double RM = v/omega; //[m] radius of the circle travelled by the robot
                   
		   // convert/transform from "link_base" frame into "circ" frame	
		   double xUcircLobst = RM - p.yUbase_linkLobst; //[m]
	           double yUcircLobst = p.xUbase_linkLobst; //[m]
if (doDebug)
ROS_INFO("   for which: xUcircLobst=%.2f [m]; yUcircLobst=%.2f [m]", xUcircLobst,yUcircLobst);

                   // conseq
		   double theta = atan2(yUcircLobst, xUcircLobst); // atan2 returns theta in (-pi,pi)  
if (doDebug)
ROS_INFO("              theta=%.2f [deg] issued by atan2(.)", rad2deg(theta));

                   // post-process: we need theta continuous in [0,2*pi)
		   if (theta < 0) theta = theta + 2*M_PI; // overwrite
if (doDebug)
ROS_INFO("              theta=%.2f [deg] after post-processing", rad2deg(theta));

                   // calc
		   double p_l_arc_nearest_obst = theta*RM; //[m]
if (doDebug)
ROS_INFO("              p_l_arc_nearest_obst=%.2f [m]", p_l_arc_nearest_obst);

                   // check whether this candidate is better than the stored closest obstacle
		   if (p_l_arc_nearest_obst<l_arc_nearest_obst) l_arc_nearest_obst = p_l_arc_nearest_obst;
		}} // if-case-Q1
	     //	   	
             // case joypad in Q2: v>0 and omega<0
	     else if (v>0 && omega<0) {
		if ((pow(v/omega + ROSbot_chassisWidth/2.0, 2.0) <= pow(-v/omega+p.yUbase_linkLobst, 2.0) + pow(p.xUbase_linkLobst, 2.0)) && (pow(-v/omega+p.yUbase_linkLobst, 2.0) + pow(p.xUbase_linkLobst, 2.0) <=  pow(-v/omega + ROSbot_chassisWidth/2.0, 2.0))) {
if (doDebug)
ROS_INFO("  Fct nearest_obst_arc_circ(.): Q2: investigating the point: xUbase_linkLobst=%.2f; yUbase_linkLobst=%.2f; distUbase_linkLobst=%.2f; angleUbase_linkLobst=%.2f [deg]", p.xUbase_linkLobst, p.yUbase_linkLobst, p.distUbase_linkLobst, rad2deg(p.angleUbase_linkLobst));  
                   // def
                   double RM = -v/omega; //[m] positive value; radius of the circle travelled by the robot

                   // convert/transform from "link_base" frame into "circ" frame        
                   double xUcircLobst = -RM - p.yUbase_linkLobst; //[m]
                   double yUcircLobst = p.xUbase_linkLobst; //[m]
if (doDebug)
ROS_INFO("   for which: xUcircLobst=%.2f [m]; yUcircLobst=%.2f [m]", xUcircLobst,yUcircLobst);

                   // conseq
                   double theta = atan2(yUcircLobst, xUcircLobst); // atan2 returns theta in (-pi,pi)  
if (doDebug)
ROS_INFO("              theta=%.2f [deg] issued by atan2(.)", rad2deg(theta));

                   // post-process: we need theta continuous in [0,2*pi); actually we are interested in the complement angle to 180 deg
                   theta = M_PI - theta; // overwrite
if (doDebug)
ROS_INFO("              theta=%.2f [deg] after post-processing", rad2deg(theta));

                   // calc
                   double p_l_arc_nearest_obst = theta*RM; //[m]
if (doDebug)
ROS_INFO("              p_l_arc_nearest_obst=%.2f [m]", p_l_arc_nearest_obst);

                   // check whether this candidate is better than the stored closest obstacle
                   if (p_l_arc_nearest_obst<l_arc_nearest_obst) l_arc_nearest_obst = p_l_arc_nearest_obst;
		}} // if-case-Q2
	    // 
            // case joypad in Q3: v<0 and omega>0
	    else if (v<0 && omega>0) {
               if ((pow(v/omega + ROSbot_chassisWidth/2.0, 2.0) <= pow(-v/omega+p.yUbase_linkLobst, 2.0) + pow(p.xUbase_linkLobst, 2.0)) && (pow(-v/omega+p.yUbase_linkLobst, 2.0) + pow(p.xUbase_linkLobst, 2.0) <=  pow(-v/omega + ROSbot_chassisWidth/2.0, 2.0))) {
if (doDebug)
ROS_INFO("  Fct nearest_obst_arc_circ(.): Q3: investigating the point: xUbase_linkLobst=%.2f; yUbase_linkLobst=%.2f; distUbase_linkLobst=%.2f; angleUbase_linkLobst=%.2f [deg]", p.xUbase_linkLobst, p.yUbase_linkLobst, p.distUbase_linkLobst, rad2deg(p.angleUbase_linkLobst));

                   // def
                   double RM = -v/omega; //[m] positive value; radius of the circle travelled by the robot

                   // convert/transform from "link_base" frame into "circ" frame        
                   double xUcircLobst = RM + p.yUbase_linkLobst; //[m]
                   double yUcircLobst = -p.xUbase_linkLobst; //[m]
if (doDebug)
ROS_INFO("   for which: xUcircLobst=%.2f [m]; yUcircLobst=%.2f [m]", xUcircLobst,yUcircLobst);

                   // conseq
                   double theta = atan2(yUcircLobst, xUcircLobst); // atan2 returns theta in (-pi,pi)  
if (doDebug)
ROS_INFO("              theta=%.2f [deg] issued by atan2(.)", rad2deg(theta));

                   // post-process: we need theta continuous in [0,2*pi); actually we are interested in the complement angle to 180 deg
                   if (theta < 0) theta = theta + 2*M_PI; // overwrite
if (doDebug)
ROS_INFO("              theta=%.2f [deg] after post-processing", rad2deg(theta));

                   // calc
                   double p_l_arc_nearest_obst = theta*RM; //[m]
if (doDebug)
ROS_INFO("              p_l_arc_nearest_obst=%.2f [m]", p_l_arc_nearest_obst);

                   // check whether this candidate is better than the stored closest obstacle
                   if (p_l_arc_nearest_obst<l_arc_nearest_obst) l_arc_nearest_obst = p_l_arc_nearest_obst;
                }} // if-case-Q3
            // 
	    // case joypad in Q4: v<0 and omega<0
	    else if (v<0 && omega<0) {
               if ((pow(v/omega - ROSbot_chassisWidth/2.0, 2.0) <= pow(v/omega-p.yUbase_linkLobst, 2.0) + pow(p.xUbase_linkLobst, 2.0)) && (pow(v/omega-p.yUbase_linkLobst, 2.0) + pow(p.xUbase_linkLobst, 2.0) <=  pow(v/omega + ROSbot_chassisWidth/2.0, 2.0))) {
if (doDebug)
ROS_INFO("  Fct nearest_obst_arc_circ(.): Q4: investigating the point: xUbase_linkLobst=%.2f; yUbase_linkLobst=%.2f; distUbase_linkLobst=%.2f; angleUbase_linkLobst=%.2f [deg]", p.xUbase_linkLobst, p.yUbase_linkLobst, p.distUbase_linkLobst, rad2deg(p.angleUbase_linkLobst)); 

                   // def
                   double RM = v/omega; //[m] positive value; radius of the circle travelled by the robot

                   // convert/transform from "link_base" frame into "circ" frame        
                   double xUcircLobst = RM - p.yUbase_linkLobst; //[m]
                   double yUcircLobst = p.xUbase_linkLobst; //[m]
if (doDebug)
ROS_INFO("   for which: xUcircLobst=%.2f [m]; yUcircLobst=%.2f [m]", xUcircLobst,yUcircLobst);

                   // conseq
                   double theta = atan2(yUcircLobst, xUcircLobst); // atan2 returns theta in (-pi,pi)  
if (doDebug)
ROS_INFO("              theta=%.2f [deg] issued by atan2(.)", rad2deg(theta));

                   // post-process: we need theta continuous in [0,2*pi); actually we are interested in the complement angle to 180 deg
		   theta = -theta; //overwrite, reverse direction
                   if (theta < 0) theta = theta + 2*M_PI; // overwrite
if (doDebug)
ROS_INFO("              theta=%.2f [deg] after post-processing", rad2deg(theta));

                   // calc
                   double p_l_arc_nearest_obst = theta*RM; //[m]
if (doDebug)
ROS_INFO("              p_l_arc_nearest_obst=%.2f [m]", p_l_arc_nearest_obst);

                   // check whether this candidate is better than the stored closest obstacle
                   if (p_l_arc_nearest_obst<l_arc_nearest_obst) l_arc_nearest_obst = p_l_arc_nearest_obst;
	    }} // if-case-Q4
            else ROS_ERROR("ctdr: Error: Please check nearest_obst_arc_circ(.)"); 


	 } // for (auto p:scan)	 
      } // if (.) 
  
   return l_arc_nearest_obst; 
   } // double nearest_obst_arc_circ(.) 

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   // member function: helper/utility function
   constexpr static inline double rad2deg(double rad) {
      return rad*180.0/M_PI;
   } //double rad2deg(double rad)

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   // member function: helper/utility function
   constexpr static inline double deg2rad(double deg) {
      return deg*M_PI/180.0;
   } //double deg2rad(double deg)



   public:
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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


// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int main(int argc, char** argv) {

    ros::init(argc, argv, "shared_control_rplidar_node");
    ros::NodeHandle nh;

    SharedControl_rplidar sc(nh);
    ros::spin();

    return EXIT_SUCCESS;
} // int main(.)

