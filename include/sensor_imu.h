#ifndef SENSOR_IMU_H_
#define SENSOR_IMU_H_

#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <string>
#include <regex>

enum IMU_BAUD_RATE
{
    sb1 = 9600,
    sb2 = 19200,
    sb3 = 38400,
    sb4 = 57600,
    sb5 = 115200,
    sb6 = 230400,
    sb7 = 460800,
    sb8 = 921600,
};

class SensorIMU
{
private:
    ros::NodeHandle _nh;
    ros::Publisher _pub_imu_cmd;
    ros::Publisher _pub_imu_data;
    ros::Subscriber _sub;
    ros::Publisher _pub_cmd;

    int _ok_count = 0;

    serial::Serial _serial;

    std::string _port;
    int _baud_rate, _output_rate, _output_format;
    int _output_gyro, _output_accelero;
    // int _output_distance;
    // int _sens_gyro, _sens_accelero;
    // int _low_pass_filter_gyro, _low_pass_filter_accelero;
    // int _power_on_start;
    std::string _regex_pattern;

    int _sleep_sec;
    int _ros_rate;

    std::string _output_ok = "<ok>";

    std_msgs::String _msg_start_bit;
    std_msgs::String _msg_error;

    sensor_msgs::Imu _imu_msg;

    void init_param();
    void callback_writeToSerial(const std_msgs::String::ConstPtr &msg);
    void start_serial_comm();
    bool initialize_IMU();
    bool check_output_ok(std::string serial_read);
    void imu_data_publisher(std::string serial_data);

public:
    SensorIMU();
    ~SensorIMU();
};

#endif