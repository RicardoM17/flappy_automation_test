#include "flappy_automation_code/flappy_automation_code.hpp"

flappyBird::flappyBird()
{
    //Initialization of nodehandle
    nh_ = new ros::NodeHandle();
    //Init publishers and subscribers
    pub_acc_cmd_ = nh_->advertise<geometry_msgs::Vector3>("/flappy_acc", 1);
    pub_flappy_data_ = nh_->advertise<flappy_automation_code::FlappyData>("/flappy_data", 1);
    sub_vel_ = nh_->subscribe<geometry_msgs::Vector3>("/flappy_vel", 1, &flappyBird::velCallback, this);
    sub_laser_scan_ = nh_->subscribe<sensor_msgs::LaserScan>("/flappy_laser_scan", 1, &flappyBird::laserScanCallback, this);
}

flappyBird::~flappyBird()
{
}

void flappyBird::velCallback(const geometry_msgs::Vector3::ConstPtr &msg)
{
    double vel_msg_time_diff = ros::Time::now().toSec() - last_vel_msg_time_.toSec();
    flappy_vel_ = *msg;

    // Update position estimate based on average velocity
    // x = (vxi + vxf) * t / 2;
    flappy_pos_.x += (flappy_vel_.x + previous_flappy_vel_.x) * vel_msg_time_diff / (2);
    flappy_pos_.y += (flappy_vel_.y + previous_flappy_vel_.y) * vel_msg_time_diff / (2);

    // Store velocity measurements for next
    previous_flappy_vel_.x = flappy_vel_.x;
    previous_flappy_vel_.y = flappy_vel_.y;

    last_vel_msg_time_ = ros::Time::now();
}

void flappyBird::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    laser_scan_ = *msg;
    last_laser_scan_time_ = msg->header.stamp;
}

bool flappyBird::flappyValid()
{

    // If not receiving any laser scan messages then game is not running.
    double time_since_last_scan = ros::Time::now().toSec() - last_laser_scan_time_.toSec();
    if (time_since_last_scan > 0.5 || laser_scan_.ranges.empty())
    {

        // Reset state
        state_ = MOVING;

        // Reset position estimate and setpoint
        flappy_pos_.x = 0.0;
        flappy_pos_.y = 0.0;
        y_pos_sp_ = 0.0;

        // Reset occupancy grid maps
        occupancy_grid_.setConstant(0.5f);
        occupancy_grid_x_.setConstant(0.0f);

        // Reset obstacle counter
        obstacle_counter_ = 0;

        return false;
    }
    else
    {
        return true;
    }
}

float flappyBird::xVelocityController(float x_vel_sp)
{
    float x_acc_cmd;

    x_vel_sp = std::min(x_vel_sp, 3.0f);

    float kp_vel = 4.0f;
    float kd_vel = 0.02f;

    float velocity_error = x_vel_sp - flappy_vel_.x;

    x_acc_cmd = kp_vel * velocity_error + kd_vel * (flappy_vel_.x - previous_flappy_vel_.x);

    return x_acc_cmd;
}

float flappyBird::yPositionController()
{
    float y_acc_cmd;

    // P: 30 D: 10 is quite aggressive
    // P: 20 D: 8 is more relaxed
    float kp_y = 25.0f;
    float kd_y = 9.0f;

    float y_error = y_pos_sp_ - flappy_pos_.y;

    y_acc_cmd = kp_y * y_error + kd_y * (0.0f - flappy_vel_.y);

    return y_acc_cmd;
}

bool flappyBird::detectGate(const float gap_p_threshold)
{
    // Initialize variables.
    int gap_counter = 0;
    int gap_start = -1;
    int gap_end = -1;
    float temp_y;
    float gap_middle = 0;
    bool gate_found = false;

    // Maximum distance for a gap to exist in
    float max_gap_distance = max_gap_y_ - min_gap_y_;

    // Ignore the first and last two grid values as they don't contain gaps
    for (int8_t i = 2; i < occupancy_grid_.rows() - 2; i++)
    {
        // If gap is free increase counter. Else reset counter as we're looking for consecutive gaps and some small holes may exist.
        if (occupancy_grid_(i, 0) < gap_p_threshold)
        {
            if (gap_counter == 0)
            {
                gap_start = i;
            }

            gap_counter++;
        }
        else
        {
            gap_counter = 0;
        }

        // If there are more than 2 consecutive gaps we found a gate
        if (gap_counter > 1)
        {

            gap_end = i;
            // Compute the gate location based on the average of the first and last grid indexes.
            gap_middle = (gap_start + gap_end) / 2;
            temp_y = (max_gap_distance / (occupancy_grid_.rows() - 1)) * gap_middle;

            // Update the y position setpoint
            y_pos_sp_ = max_gap_y_ - temp_y;

            gate_found = true;
        }
    }

    return gate_found;
}

void flappyBird::flappyControl()
{
    // Initialize variables
    geometry_msgs::Vector3 acc_cmd;
    std::vector<float> scan_x(9);
    std::vector<float> scan_y(9);

    max_gap_y_ = CEILLING_HEIGHT - 3 * FLAPPY_HEIGHT;
    min_gap_y_ = FLOOR_HEIGHT + 3 * FLAPPY_HEIGHT;

    // Maximum distance for a gap to exist in
    float max_gap_distance = max_gap_y_ - min_gap_y_;

    float y_sp_correction = 0.0f;
    float nearest_y = INFINITY;

    // Parse all lasers
    for (int8_t i = 0; i < laser_scan_.ranges.size(); i++)
    {
        // Calculate the angle of the laser
        float scan_angle = laser_scan_.angle_min + laser_scan_.angle_increment * i;

        // Calculate the x and y distance of the obstacle to flappy (body frame)
        scan_x[i] = laser_scan_.ranges[i] * cos(scan_angle);
        scan_y[i] = laser_scan_.ranges[i] * sin(scan_angle);

        // Calculate x and y location of object in local frame
        float obstacle_x = scan_x[i] + flappy_pos_.x;
        float obstacle_y = scan_y[i] + flappy_pos_.y;

        int obstacle_col_index = -1;
        int obstacle_row_index;

        // Correct y position estimate based on ceiling and floor measurements
        // Logic: The location of the ceiling is known. If a measurement indicantes an obstacle past the ceiling
        // then we know we've drifted upwards. Vice-versa for the floor.
        // This is valid and true given the project assumptions
        if (obstacle_y > CEILLING_HEIGHT)
        {
            flappy_pos_.y -= obstacle_y - CEILLING_HEIGHT;
        }
        else if (obstacle_y < FLOOR_HEIGHT)
        {
            flappy_pos_.y += FLOOR_HEIGHT - obstacle_y;
        }

        // Obstacle detected is neither floor nor ceiling and is an object.
        if (obstacle_y < max_gap_y_ && obstacle_y > min_gap_y_ && laser_scan_.intensities[i])
        {

            // If we're past the obstacle wall
            if (flappy_pos_.x > occupancy_grid_x_(0) + ASTEROID_WIDTH / 2)
            {
                // Move furthest wall to nearest and set a hypothetical far enough distance for furthest wall
                occupancy_grid_x_(0) = occupancy_grid_x_[1];
                occupancy_grid_x_(1) += 1.0f;

                // Do the same for occupancy grid and create a new one full of unknown fields for furthest wall
                occupancy_grid_.col(0) = occupancy_grid_.col(1);
                occupancy_grid_.col(1).setConstant(OCCUPANCY_GRID_UNKNOWN);
            }

            // If found a new obstacle wall and we're not passing a wall
            if (obstacle_x > occupancy_grid_x_.maxCoeff() + ASTEROID_WIDTH && flappy_pos_.x < occupancy_grid_x_(0))
            {
                obstacle_counter_++;

                // Update new wall location
                occupancy_grid_x_(1) = obstacle_x;
            }

            // Check if obstacle belongs to nearest or furthest wall
            if (abs(obstacle_x - occupancy_grid_x_[0]) < abs(obstacle_x - occupancy_grid_x_[1]))
            {
                obstacle_col_index = 0;
            }
            else
            {
                obstacle_col_index = 1;
            }

            // Compute obstacle index in the grid and update it
            float temp_y = abs(obstacle_y - max_gap_y_);
            obstacle_row_index = std::round((temp_y / max_gap_distance) * (occupancy_grid_.rows() - 1));

            occupancy_grid_(obstacle_row_index, obstacle_col_index) = OCCUPANCY_GRID_OCCUPIED;
        }

        // Compute intersections between laser scan and gaps in obstacle walls
        // Because walls are roughly vertical lines the interception can be found by:
        // y = mx + b
        // m = tan(scan_angle)
        // x = occupancy_grid_x_(0) or occupancy_grid_x_(1) - flappy_pos_.x
        // b = flappy_pos_.y;

        // Current laser is detecting an obstacle past the nearest wall
        // Fill occupancy grid for nearest wall.
        // Note: The furthest wall has already been updated above.
        if (obstacle_col_index == 1)
        {
            float gap_y = std::tan(scan_angle) * (occupancy_grid_x_(0) - flappy_pos_.x) + flappy_pos_.y;

            // Discard intersections outside map
            if (gap_y < max_gap_y_ && gap_y > min_gap_y_)
            {
                float temp_y = abs(gap_y - max_gap_y_);

                obstacle_row_index = std::round((temp_y / max_gap_distance) * (occupancy_grid_.rows() - 1));

                // Do not update if this field has already been previously defined as occupied.
                // This is to avoid small gaps in the rocks from providing wrong info.
                if (!(occupancy_grid_(obstacle_row_index, 0) > 0.95 * OCCUPANCY_GRID_OCCUPIED))
                {
                    occupancy_grid_(obstacle_row_index, 0) = OCCUPANCY_GRID_FREE;
                }
            }
        }
        else if (!laser_scan_.intensities[i]) // Laser is not hitting any object. Fill occupancy grid for both walls
        {
            for (int8_t j = 0; j < 2; j++)
            {
                float gap_y = std::tan(scan_angle) * (occupancy_grid_x_(j) - flappy_pos_.x) + flappy_pos_.y;

                // Discard intersections if outside interest zone
                if (gap_y < max_gap_y_ && gap_y > min_gap_y_)
                {
                    float temp_y = abs(gap_y - max_gap_y_);
                    obstacle_row_index = std::round((temp_y / max_gap_distance) * (occupancy_grid_.rows() - 1));

                    // Set field to empty only if it hasn't been set as occupied before
                    // This is to avoid small gaps in the rocks from providing wrong info.
                    if (!(occupancy_grid_(obstacle_row_index, j) > 0.95 * OCCUPANCY_GRID_OCCUPIED))
                    {
                        occupancy_grid_(obstacle_row_index, j) = OCCUPANCY_GRID_FREE;
                    }
                }
            }
        }

        // Find the closest object w.r.t. to y axis, within a certain distance from flappy using only the 2 top and 2 bottom lasers
        if ((laser_scan_.ranges[i] < FLAPPY_HEIGHT) && (std::abs(scan_y[i]) < std::abs(nearest_y)) && (i < 2 || i > laser_scan_.ranges.size() - 3))
        {
            // Compute a y setpoint correction based on the distance of the object to act as collision avoidance.
            nearest_y = scan_y[i];
            y_sp_correction = (nearest_y > 0) ? -(FLAPPY_HEIGHT - nearest_y) : (FLAPPY_HEIGHT + nearest_y);
        }
    }

    float max_x_vel = MAX_X_VEL;

    // Minimum braking distance without crashing at current speed is given by
    // vf^2 = vi^2 + 2 * a * dx
    // dx = vi^2 / (- 2 * a) + 0^2
    // brake_distance = dx + buffer
    float brake_distance = (flappy_vel_.x * flappy_vel_.x) / (2 * MAX_X_ACCEL) + 0.65;

    // Check that there's any object directly in front of flappy and within braking distance.
    // Break if that's the case
    if (!(scan_x[3] > brake_distance * 0.95f && scan_x[4] > brake_distance && scan_x[5] > brake_distance * 0.95f))
    {
        // Set max velocity as very large negative number to ensure max braking.
        max_x_vel = -100.0;
    }

    // Check if there's a confirmed gate
    bool gate_found = detectGate(OCCUPANCY_GRID_FREE + 0.05f);

    // If we're in scanning mode and we found a confirmed gate, change mode.
    if (gate_found && state_ == SCANNING)
    {
        state_ = MOVING;
    }

    // Flappy is stopped and past the original point
    // Change to scanning mode
    if ((abs(flappy_vel_.x) < 0.01 && abs(flappy_vel_.y) < 0.01 && flappy_pos_.x > 0.01))
    {
        state_ = SCANNING;
    }

    if (state_ == SCANNING)
    {
        // Rerun gate detection not excluding blocks that are currently unknown.
        detectGate(OCCUPANCY_GRID_UNKNOWN + 0.05f);
    }

    // Y Collision avoidance for nearby objects. Clamped for robustness
    y_pos_sp_ += std::min(FLAPPY_HEIGHT, std::max(y_sp_correction, -FLAPPY_HEIGHT));

    // Compute the acceleration commands by calling respective controllers
    acc_cmd.x = xVelocityController(max_x_vel);
    acc_cmd.y = yPositionController();

    // Clamp acceleration values according to max specs
    acc_cmd.x = std::min(static_cast<double>(MAX_X_ACCEL), std::max(acc_cmd.x, static_cast<double>(-MAX_X_ACCEL)));
    acc_cmd.y = std::min(static_cast<double>(MAX_Y_ACCEL), std::max(acc_cmd.y, static_cast<double>(-MAX_Y_ACCEL)));

    // Publish acceleration commands
    pub_acc_cmd_.publish(acc_cmd);

    // Fill flappy data
    flappy_data_.header.stamp = ros::Time::now();
    flappy_data_.position = flappy_pos_;
    flappy_data_.setpoint.x = max_x_vel;
    flappy_data_.setpoint.y = y_pos_sp_;
    flappy_data_.score = std::max(obstacle_counter_ - 2, 0);

    // Publish flappy data
    pub_flappy_data_.publish(flappy_data_);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "flappy_automation_code");

    flappyBird myFlappyBird;

    ros::Rate rate(myFlappyBird.NODE_RATE);

    while (ros::ok())
    {
        bool flappy_validity = myFlappyBird.flappyValid();

        // While flappy is alive run main module
        if (flappy_validity)
        {
            myFlappyBird.flappyControl();
        }
        // Ros spin to prevent program from exiting
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
