#include "sensor_imu.h"

SensorIMU::SensorIMU()
{
    init_param();
    _pub_imu_cmd = _nh.advertise<std_msgs::String>("/imu_cmd_read", 1);
    _pub_imu_data = _nh.advertise<sensor_msgs::Imu>("/imu_read", 1);
    _sub = _nh.subscribe("/imu_write", 1, &SensorIMU::callback_writeToSerial, this);
    _pub_cmd = _nh.advertise<std_msgs::String>("/imu_write", 1);
    _msg_start_bit.data = "*";
    start_serial_comm();
}

SensorIMU::~SensorIMU()
{
}

void SensorIMU::init_param()
{
    ros::NodeHandle _nh("~");
    std::string port = "/dev/ttyUSB0";
    std::string regex_pattern = "(([-+]?\\d*\\.\\d+)|\\d+)";
    _nh.param("PORT", _port, port);
    _nh.param("SET_BAUDRATE", _baud_rate, 5);
    _nh.param("SET_OUTPUT_RATE", _output_rate, 10);
    _nh.param("SET_OUTPUT_FORMAT", _output_format, 1);
    _nh.param("SET_OUTPUT_GYRO", _output_gyro, 0);
    _nh.param("SET_OUTPUT_ACCELERO", _output_accelero, 0);
    // _nh.param("SET_OUTPUT_DISTANCE", _output_distance, 0);
    // _nh.param("SET_SENS_GYRO", _sens_gyro, 5);
    // _nh.param("SET_SENS_ACCELERO", _sens_accelero, 3);
    // _nh.param("SET_Low_Pass_Filter_Gyroscope", _low_pass_filter_gyro, 3);
    // _nh.param("SET_Low_Pass_Filter_Accelerometer", _low_pass_filter_accelero, 3);
    // _nh.param("POWER_ON_START", _power_on_start, 1);
    _nh.param("sleep_sec", _sleep_sec, 0);
    _nh.param("ros_rate", _ros_rate, 10);
    _nh.param("regex_pattern", _regex_pattern, regex_pattern);
    _msg_error.data = "::: <err> occurred ::: check sensor_imu.launch & parameters";
}

void SensorIMU::callback_writeToSerial(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO_STREAM("Writing to serial port" << msg->data);
    _serial.write(msg->data);
}

void SensorIMU::start_serial_comm()
{
    std::cout << "start_serial_comm" << std::endl;
    try
    {
        _serial.setPort(_port);
        _serial.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(100);
        _serial.setTimeout(to);
        _serial.open();
        bool initialized = initialize_IMU();

        while (initialized == false)
        {
            sleep(_sleep_sec);
            _pub_imu_cmd.publish(_msg_error);
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        return;
    }

    if (_serial.isOpen())
    {
        ROS_INFO_STREAM("IMU sensor initialized");
    }
    else
    {
        return;
    }

    sleep(_sleep_sec);

    ros::Rate loop_rate(_ros_rate);
    int i = 0;
    while (ros::ok())
    {
        ros::spinOnce();
        _pub_cmd.publish(_msg_start_bit);
        if (_serial.available())
        {
            std_msgs::String result;
            ROS_INFO_STREAM("serial ::: * ::: available : " << _serial.available());
            result.data = _serial.read(_serial.available());
            imu_data_publisher(result.data);
            // ROS_INFO_STREAM("vector : " << data_imu);
            // for (std::string i : data_imu)
            // {
                // ROS_INFO_STREAM("vector :: " << i << "\n");
            // }
            ROS_INFO_STREAM("Read: \n"
                            << result.data);
            _pub_imu_cmd.publish(result);
        }
        loop_rate.sleep();
    }
}

bool SensorIMU::initialize_IMU()
{
    _serial.flush();
    _serial.flushOutput();
    _serial.flushInput();
    ROS_INFO_STREAM(":: ...Initializing IMU... ::");
    if (_baud_rate != 5)
    {
        std::string set_baud_rate = "<sb" + std::to_string(_baud_rate) + ">";
        _serial.write(set_baud_rate);
        sleep(_sleep_sec);
        if (!check_output_ok(_serial.read(_serial.available())))
        {
            return false;
        }
    }

    if (_output_rate != 10)
    {
        std::string set_output_rate = "<sor" + std::to_string(_output_rate) + ">";
        _serial.flush();
        _serial.write(set_output_rate);
        sleep(_sleep_sec);
        if (!check_output_ok(_serial.read(_serial.available())))
        {
            return false;
        }
    }

    if (_output_format != 1)
    {
        std::string set_output_format = "<sof" + std::to_string(_output_format) + ">";
        _serial.write(set_output_format);
        sleep(_sleep_sec);
        if (!check_output_ok(_serial.read(_serial.available())))
        {
            return false;
        }
    }

    if (_output_gyro != 0)
    {
        std::string set_output_gyro = "<sog" + std::to_string(_output_gyro) + ">";
        _serial.write(set_output_gyro);
        sleep(_sleep_sec);
        if (!check_output_ok(_serial.read(_serial.available())))
        {
            return false;
        }
    }

    if (_output_accelero != 0)
    {
        std::string set_output_accelero = "<soa" + std::to_string(_output_accelero) + ">";
        _serial.write(set_output_accelero);
        sleep(_sleep_sec);
        if (!check_output_ok(_serial.read(_serial.available())))
        {
            return false;
        }
    }

    // if (_output_distance != 0)
    // {
    //     std::string set_output_distance = "<sod" + std::to_string(_output_distance) + ">";
    //     _serial.write(set_output_distance);
    //     sleep(_sleep_sec);
    //     if (!check_output_ok(_serial.read(_serial.available())))
    //     {
    //         return false;
    //     }
    // }

    // if (_sens_gyro != 5)
    // {
    //     std::string set_sens_gyro = "<ssg" + std::to_string(_sens_gyro) + ">";
    //     _serial.write(set_sens_gyro);
    //     sleep(_sleep_sec);
    //     if (!check_output_ok(_serial.read(_serial.available())))
    //     {
    //         return false;
    //     }
    // }

    // if (_sens_accelero != 3)
    // {
    //     std::string set_sens_accelero = "<ssa" + std::to_string(_sens_accelero) + ">";
    //     _serial.write(set_sens_accelero);
    //     sleep(_sleep_sec);
    //     if (!check_output_ok(_serial.read(_serial.available())))
    //     {
    //         return false;
    //     }
    // }

    // if (_low_pass_filter_gyro != 3)
    // {
    //     std::string set_low_pass_filter_gyro = "<lpfg" + std::to_string(_low_pass_filter_gyro) + ">";
    //     _serial.write(set_low_pass_filter_gyro);
    //     sleep(_sleep_sec);
    //     if (!check_output_ok(_serial.read(_serial.available())))
    //     {
    //         return false;
    //     }
    // }

    // if (_low_pass_filter_accelero != 5)
    // {
    //     std::string set_low_pass_filter_accelero = "<lpfa" + std::to_string(_low_pass_filter_accelero) + ">";
    //     _serial.write(set_low_pass_filter_accelero);
    //     sleep(_sleep_sec);
    //     if (!check_output_ok(_serial.read(_serial.available())))
    //     {
    //         return false;
    //     }
    // }

    // if (_power_on_start != 1)
    // {
    //     std::string set_power_on_start = "<pons" + std::to_string(_power_on_start) + ">";
    //     _serial.write(set_power_on_start);
    //     sleep(_sleep_sec);
    //     if (!check_output_ok(_serial.read(_serial.available())))
    //     {
    //         return false;
    //     }
    // }
    return true;
}

bool SensorIMU::check_output_ok(std::string serial_read)
{
    std::cout << serial_read << std::endl;
    if (_output_ok.compare(serial_read) == 0)
    {
        return true;
    }
    else 
    {
        return false;
    }
}

void SensorIMU::imu_data_publisher(std::string serial_data)
{
    std::regex reg{ _regex_pattern };
    std::sregex_token_iterator iter(serial_data.begin(), serial_data.end(), reg), end;
    std::vector<std::string> result = std::vector<std::string>(iter, end);
    if (!result.empty())
    {
        _imu_msg.header.frame_id = "/imu_read";
        _imu_msg.header.stamp = ros::Time::now();

        _imu_msg.orientation.z = std::stof(result[0]);
        _imu_msg.orientation.y = std::stof(result[1]);
        _imu_msg.orientation.x = std::stof(result[2]);
        _imu_msg.orientation.w = std::stof(result[3]);

        _imu_msg.angular_velocity.x = std::stof(result[4]);
        _imu_msg.angular_velocity.y = std::stof(result[5]);
        _imu_msg.angular_velocity.z = std::stof(result[6]);

        _imu_msg.linear_acceleration.x = std::stof(result[7]);
        _imu_msg.linear_acceleration.y = std::stof(result[8]);
        _imu_msg.linear_acceleration.z = std::stof(result[9]);

        _pub_imu_data.publish(_imu_msg);
    }
}