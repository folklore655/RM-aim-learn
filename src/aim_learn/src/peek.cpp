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

// ---------- 工具函数（必须定义在使用者之前） ----------

// 四角平均 = 板中心
static void plate_center(const rm_aim_sim_bridge::msg::ArmorCorners & a,
                         double & cx, double & cy, double & cz)
{
    cx = cy = cz = 0.0;
    for (int i = 0; i < 4; ++i) {
        cx += a.corners[i].x * 0.25;
        cy += a.corners[i].y * 0.25;
        cz += a.corners[i].z * 0.25;
    }
}

// 板面法向（单位向量）
static void plate_normal(const rm_aim_sim_bridge::msg::ArmorCorners & a,
                         double & nx, double & ny, double & nz)
{
    const auto & p = a.corners;
    double ax = p[1].x - p[0].x, ay = p[1].y - p[0].y, az = p[1].z - p[0].z;  // 上边
    double bx = p[3].x - p[0].x, by = p[3].y - p[0].y, bz = p[3].z - p[0].z;  // 左边
    nx = ay * bz - az * by;
    ny = az * bx - ax * bz;
    nz = ax * by - ay * bx;
    double len = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (len > 1e-9) { nx /= len; ny /= len; nz /= len; }
}

// 发"停控停火"指令
static void publish_stop(const ArmorCornersArray::SharedPtr & msg)
{
    AimCommand cmd;
    cmd.header     = msg->header;
    cmd.session_id = msg->session_id;
    cmd.yaw        = 0.0;
    cmd.pitch      = 0.0;
    cmd.enable     = false;
    cmd.fire       = false;
    g_pub->publish(cmd);
}

// ---------- 回调 ----------

void on_armors(const ArmorCornersArray::SharedPtr msg)
{
    if (msg->armors.empty()) {
        publish_stop(msg);          // 没目标也要报告：停控停火
        return;
    }

    // ---- 选板：正对 + 最近 ----
    int    best      = -1;
    double best_dist = 1e9;
    double best_face = 0.0;

    for (size_t i = 0; i < msg->armors.size(); ++i) {
        double cx, cy, cz;
        plate_center(msg->armors[i], cx, cy, cz);
        double dist = std::sqrt(cx*cx + cy*cy + cz*cz);
        if (dist < 1e-6) continue;

        double nx, ny, nz;
        plate_normal(msg->armors[i], nx, ny, nz);

        double ux = cx / dist, uy = cy / dist, uz = cz / dist;
        double face = std::fabs(nx*ux + ny*uy + nz*uz);   // 1=正对 0=侧对

        if (face > 0.6 && dist < best_dist) {
            best = i; best_dist = dist; best_face = face;
        }
    }

    if (best < 0) {                 // 全侧着，这帧不打
        publish_stop(msg);
        return;
    }

    // ---- 用选中那块板算角 ----
    const auto & p = msg->armors[best].corners;
    double cx = 0, cy = 0, cz = 0;
    for (int i = 0; i < 4; ++i) {
        cx += p[i].x * 0.25;
        cy += p[i].y * 0.25;
        cz += p[i].z * 0.25;
    }

    double D  = std::hypot(cx, cy);
    double dz = cz;

    double e = 0.0;
    for (int i = 0; i < 3; ++i) {
        double d = D  - L * std::cos(e);
        double h = dz - L * std::sin(e);
        double disc = V*V*V*V - G * (G*d*d + 2.0*h*V*V);
        if (disc < 0.0) { printf("超出射程，打不到\n"); return; }
        e = std::atan2(V*V - std::sqrt(disc), G*d);
    }

    printf("选中第 %d 块 face=%.2f dist=%.2f | D=%.3f dz=%.3f yaw=%.2f° pitch=%.2f°\n",
           best, best_face, best_dist,
           D, dz,
           std::atan2(cy, cx) * 180.0 / M_PI,
           e * 180.0 / M_PI);

    AimCommand cmd;
    cmd.header     = msg->header;
    cmd.session_id = msg->session_id;
    cmd.yaw        = std::atan2(cy, cx);
    cmd.pitch      = e;
    cmd.enable     = true;
    cmd.fire       = true;          // 注意：这里是 true（开火）
    g_pub->publish(cmd);
}

// ---------- main ----------

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