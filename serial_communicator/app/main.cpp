#include <serial_communicator/serial.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <functional>
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

union AngleData
{
    float angles[12];  // 4つのARM × 3つの角度
    uint8_t bytes[48]; // 12 * sizeof(float)
};

union PosData
{
    uint16_t positions[12];
    uint8_t bytes[24];
};

AngleData angle_data;
AngleData angle_data2;
AngleData received_angle_data;

template <typename T, typename U>
U mapValue(T x, T in_min, T in_max, U out_min, U out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double normalizeAngle(double angle)
{
    while (angle > 180.0)
        angle -= 360.0;
    while (angle < -180.0)
        angle += 360.0;
    return angle;
}

PosData send_pos_data;
PosData send_pos_data2;

uint8_t header[2] = {0xFF, 0xFF};
uint8_t footer[2] = {0xFE, 0xFD};

const int arm_num = 4;

void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg, SerialCommunication* serial)
{
    if (msg->name.size() != arm_num * 3)
    {
        RCLCPP_ERROR(rclcpp::get_logger("serial_communicator"), "Received joint state does not contain enough data for all arms.");
        return;
    }

    for (int i = 0; i < arm_num; i++)
    {
        double x_offset = 0.0;
        double isInverse = 1.0;
        if (i % 2 == 0)
        {
            x_offset = -90.0;
            isInverse = -1.0;
        }
        else
        {
            x_offset = 90.0;
            isInverse = 1.0;
        }

        angle_data.angles[i * 3 + 0] = static_cast<float>(msg->position[i * 3 + 0] * 180 / M_PI + x_offset);
        angle_data.angles[i * 3 + 1] = static_cast<float>(msg->position[i * 3 + 1] * 180 / M_PI * isInverse);
        angle_data.angles[i * 3 + 2] = static_cast<float>(msg->position[i * 3 + 2] * 180 / M_PI * isInverse);

        send_pos_data.positions[i * 3 + 0] = static_cast<uint16_t>(mapValue(normalizeAngle(-msg->position[i * 3 + 0] * 180 / M_PI + x_offset),  -180.0, 180.0, 0, 4095));
        send_pos_data.positions[i * 3 + 1] = static_cast<uint16_t>(mapValue(normalizeAngle(msg->position[i * 3 + 1] * 180 / M_PI * (-isInverse)), -180.0, 180.0, 0, 4095));
        send_pos_data.positions[i * 3 + 2] = static_cast<uint16_t>(mapValue(normalizeAngle(msg->position[i * 3 + 2] * 180 / M_PI * isInverse), -180.0, 180.0, 0, 4095));

        // std::cout << "ARM" << i << ": "
        //           << "Angle1: " << angle_data.angles[i * 3 + 0] << ", "
        //           << "Angle2: " << angle_data.angles[i * 3 + 1] << ", "
        //           << "Angle3: " << angle_data.angles[i * 3 + 2] << ", " << std::endl;
                //   << "Position1: " << send_pos_data.positions[i * 3 + 0] << ", "
                //   << "Position2: " << send_pos_data.positions[i * 3 + 1] << ", "
                //   << "Position3: " << send_pos_data.positions[i * 3 + 2] << std::endl;
    }
}

void jointState2Callback(const sensor_msgs::msg::JointState::SharedPtr msg, SerialCommunication* serial)
{
    if (msg->name.size() != arm_num * 3)
    {
        RCLCPP_ERROR(rclcpp::get_logger("serial_communicator"), "Received joint state does not contain enough data for all arms.");
        return;
    }

    for (int i = 0; i < arm_num; i++)
    {
        double x_offset = 0.0;
        double isInverse = 1.0;
        if (i % 2 == 0)
        {
            x_offset = -90.0;
            isInverse = -1.0;
        }
        else
        {
            x_offset = 90.0;
            isInverse = 1.0;
        }

        angle_data2.angles[i * 3 + 0] = static_cast<float>(msg->position[i * 3 + 0]);
        angle_data2.angles[i * 3 + 1] = static_cast<float>(msg->position[i * 3 + 1]);
        angle_data2.angles[i * 3 + 2] = static_cast<float>(msg->position[i * 3 + 2]);

        send_pos_data2.positions[i * 3 + 0] = static_cast<uint16_t>(mapValue(normalizeAngle(msg->position[i * 3 + 0]), -180.0, 180.0, 0, 4095));
        send_pos_data2.positions[i * 3 + 1] = static_cast<uint16_t>(mapValue(normalizeAngle(-msg->position[i * 3 + 1]), -180.0, 180.0, 0, 4095));
        send_pos_data2.positions[i * 3 + 2] = static_cast<uint16_t>(mapValue(normalizeAngle(msg->position[i * 3 + 2]), -180.0, 180.0, 0, 4095));
    }
}

sensor_msgs::msg::Joy current_joy_msg;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("serial_communicator");

    std::string serial_port = "/dev/ttyUSB0";
    int serial_baudrate = 115200;
    std::string angle_topic_name = "angle_data";
    
    // パラメータの取得
    node->declare_parameter<std::string>("port", serial_port);
    node->declare_parameter<int>("baudrate", serial_baudrate);
    node->declare_parameter<std::string>("angle_topic_name", angle_topic_name);
    
    node->get_parameter("port", serial_port);
    node->get_parameter("baudrate", serial_baudrate);
    node->get_parameter("angle_topic_name", angle_topic_name);
    
    SerialCommunication serial(serial_port);

    auto pub = node->create_publisher<std_msgs::msg::Float64MultiArray>(angle_topic_name, 10);

    auto joint_state_sub = node->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 1,
        [&serial](const sensor_msgs::msg::JointState::SharedPtr msg) {
            jointStateCallback(msg, &serial);
        });
        
    auto joint_state_sub2 = node->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states2", 1,
        [&serial](const sensor_msgs::msg::JointState::SharedPtr msg) {
            jointState2Callback(msg, &serial);
        });

    current_joy_msg.buttons.resize(32, 0);
    current_joy_msg.axes.resize(32, 0.0);
    
    auto joy_sub = node->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 1,
        [](const sensor_msgs::msg::Joy::SharedPtr msg) {
            current_joy_msg = *msg;
        });

    if (!serial.open_port(serial_baudrate))
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to open serial port");
        return 1;
    }

    rclcpp::sleep_for(1s);

    serial.receive_bytes();

    // 送信データの初期化
    for (int i = 0; i < arm_num * 3; ++i)
    {
        send_pos_data.positions[i] = 2048;
        send_pos_data2.positions[i] = 2048;
    }
    send_pos_data.positions[0] = 1024;
    send_pos_data2.positions[0] = 1024;
    send_pos_data.positions[3] = 3072;
    send_pos_data2.positions[3] = 3072;
    send_pos_data.positions[6] = 1024;
    send_pos_data2.positions[6] = 1024;
    send_pos_data.positions[9] = 3072;
    send_pos_data2.positions[9] = 3072;

    auto start_time = node->now();
    
    auto timer = node->create_wall_timer(10ms, []() {
        // 空のタイマー
    });

    auto timer2 = node->create_wall_timer(10ms, [&serial, &node]() {
        std::vector<uint8_t> data_to_send;
    
        data_to_send.push_back(header[0]);
        data_to_send.push_back(header[1]);
        data_to_send.push_back(24); // データ長を指定
        
        // data_to_send.insert(data_to_send.end(), angle_data.bytes, angle_data.bytes + sizeof(angle_data.bytes));
        if (current_joy_msg.buttons.size() > 3 &&
            (current_joy_msg.buttons[0] == 1 ||
             current_joy_msg.buttons[1] == 1 ||
             current_joy_msg.buttons[2] == 1 ||
             current_joy_msg.buttons[3] == 1))
        {
            data_to_send.insert(data_to_send.end(), send_pos_data2.bytes, send_pos_data2.bytes + sizeof(send_pos_data2.bytes));
        }
        else
        {
            data_to_send.insert(data_to_send.end(), send_pos_data.bytes, send_pos_data.bytes + sizeof(send_pos_data.bytes));
        }
        
        data_to_send.push_back(footer[0]);
        data_to_send.push_back(footer[1]);

        if (serial.send_binary_data(data_to_send))
        {
            for (int i = 0; i < data_to_send.size(); ++i)
            {
                std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(data_to_send[i]) << " ";
            }
            std::cout << std::dec << std::endl;
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to send data to serial port.");
        }
    });

    rclcpp::spin(node);
    
    std::string torque_off = "TorqueOff\n";
    serial.send_data(torque_off);
    serial.close_port();
    
    rclcpp::shutdown();
    return 0;
}