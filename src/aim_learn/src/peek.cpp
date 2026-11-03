#include <cmath>
#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "rm_aim_sim_bridge/msg/armor_corners_array.hpp"
#include "rm_aim_sim_bridge/msg/aim_command.hpp"

using rm_aim_sim_bridge::msg::ArmorCornersArray;
using rm_aim_sim_bridge::msg::AimCommand;

rclcpp::Publisher<AimCommand>::SharedPtr g_pub;   // 全局发布器

const double V = 23.0;      // 弹丸初速 m/s
const double G = 9.7833;    // 重力 m/s^2
const double L = 0.22;      // 枪口比云台中心前伸 0.22 m

void on_armors(const ArmorCornersArray::SharedPtr msg)
{
    if (msg->armors.empty())
    {
        printf("没看到装甲\n"); return; 
    }

    const auto & p = msg->armors[0].corners;   // 4 个角点
    double cx = 0, cy = 0, cz = 0;              //cx cy cz就是离云台的坐标
    cx = (p[0].x + p[1].x + p[2].x + p[3].x) / 4.0;
    cy = (p[0].y + p[1].y + p[2].y + p[3].y) / 4.0;
    cz = (p[0].z + p[1].z + p[2].z + p[3].z) / 4.0;

    double D  = std::hypot(cx, cy);   // 水平距离
    double dz = cz;                   // 高度差

    // 计算仰角
    double e = 0.0;                   // 先猜 0°
    for (int i = 0; i < 3; ++i)
    {
    double d = D  - L * std::cos(e);          // 枪口到目标的水平距离
    double h = dz - L * std::sin(e);          // 枪口到目标的高度差
    double disc = V*V*V*V - G * (G*d*d + 2.0*h*V*V);    // 判别式
    if (disc < 0.0) { printf("超出射程，打不到\n"); return; }   // 根号里是负的
    e = std::atan2(V*V - std::sqrt(disc), G*d);   // 解出仰角（低伸解）
    }

    printf("D=%.3f  dz=%.3f  yaw=%.2f°  pitch=%.2f°\n",
        D, dz,
        std::atan2(cy, cx) * 180.0 / M_PI,
        e * 180.0 / M_PI);

    AimCommand cmd;
    cmd.header     = msg->header;        // 复制观测的时间戳
    cmd.session_id = msg->session_id;    // 复制会话号
    cmd.yaw        = std::atan2(cy, cx);
    cmd.pitch      = e;
    cmd.enable     = true;               // 接管云台
    cmd.fire       = true;              // 但先不开火
    g_pub->publish(cmd);

}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("peek");
    g_pub = node->create_publisher<AimCommand>("/aim_sim/command", 10);
    auto sub = node->create_subscription<ArmorCornersArray>("/aim_sim/armors", 10, on_armors);
    rclcpp::spin(node);

    sub.reset();
    node.reset();
    g_pub.reset();
    
    rclcpp::shutdown();
}