/**
 * @brief Flappy automation code
 * @file flappy_automation_code.hpp
 * @author Ricardo Marques <marques.ricardo17@gmail.com>
 *
 */

#ifndef FLAPPY_AUTOMATION_CODE_H_
#define FLAPPY_AUTOMATION_CODE_H_

#include "flappy_automation_code/FlappyData.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/LaserScan.h"
#include <eigen3/Eigen/Dense> /* Matrix */
#include <ros/ros.h>

/**
 * Flappy Bird class.
 * Handles the control of the flappy bird challenge.
 */
class flappyBird
{
public:
    flappyBird();
    ~flappyBird();

    /**
     * Flappy velocity data callback.
     * 
     * Handles the position estimate via odometry.
     *
     * @param msg Velocity data message.
     */
    void velCallback(const geometry_msgs::Vector3::ConstPtr &msg);

    /**
     * Laser scan data callback.
     *
     * @param msg Laser scan data.
     */
    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg);

    /**
     * Checks the running state of the flappy bird.
     *
     * @return true if game is running, false if otherwise.
     */
    bool flappyValid();

    /**
     * Main control module.
     * 
     */
    void flappyControl();

    /**
     * PD X Velocity controller.
     *
     * @param x_vel_sp X velocity setpoint.
     * @return X acceleration command.
     */
    float xVelocityController(float max_x_vel);

    /**
     * PD Y Position controller.
     *
     * @return y acceleration setpoint.
     */
    float yPositionController();

    /**
     * Gate detector function. Updates the value of y position setpoint
     *
     * @param gap_p_threshold The desired gap probability threshold.
     * @return Boolean with Gate Found/Not Found
     */
    bool detectGate(const float gap_p_threshold);

    const int NODE_RATE = 30;

private:
    //Ros nodehandle
    ros::NodeHandle *nh_ = NULL;
    //Publisher for acceleration command
    ros::Publisher pub_acc_cmd_;
    //Publisher for Flappy data, including position estimate and setpoints
    ros::Publisher pub_flappy_data_;
    //Subscriber for velocity
    ros::Subscriber sub_vel_;
    //Subscriber for laser scan
    ros::Subscriber sub_laser_scan_;

    sensor_msgs::LaserScan laser_scan_;

    flappy_automation_code::FlappyData flappy_data_;

    ros::Time last_vel_msg_time_;
    ros::Time last_laser_scan_time_;

    geometry_msgs::Vector3 flappy_pos_;
    geometry_msgs::Vector3 flappy_vel_;
    geometry_msgs::Vector3 previous_flappy_vel_;

    int state_;
    enum STATE
    {
        MOVING = 0,
        SCANNING = 1
    };

    const float ASTEROID_WIDTH = 0.50f;
    const float FLAPPY_HEIGHT = 0.25f;
    const float FLAPPY_WIDTH = 0.34f;

    const float MAX_X_VEL = 3.0;
    const float MAX_X_ACCEL = 3.0;
    const float MAX_Y_ACCEL = 35.0;

    const float CEILLING_HEIGHT = 2.51f;
    const float FLOOR_HEIGHT = -1.43f;

    const float OCCUPANCY_GRID_FREE = 0.0f;
    const float OCCUPANCY_GRID_UNKNOWN = 0.50f;
    const float OCCUPANCY_GRID_OCCUPIED = 1.0f;

    float max_gap_y_;
    float min_gap_y_;

    Eigen::Matrix<float, 30, 2> occupancy_grid_;
    Eigen::Vector2f occupancy_grid_x_;

    int obstacle_counter_ = 0;

    float y_pos_sp_;
};

#endif
