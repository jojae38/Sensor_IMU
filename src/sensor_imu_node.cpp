#include "sensor_imu.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sensor_imu_node");
    SensorIMU sensor_IMU;
    ros::spin();
    
    return 0;
}