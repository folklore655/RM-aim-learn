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

// ---------- 目标速度估计（跨帧保留，所以放全局） ----------
// 用上一帧的中心和这一帧的中心做差分，得到速度；再做低通平滑压掉抖动。
static bool   g_has_prev = false;                       // 有没有上一帧
static double g_px = 0, g_py = 0, g_pz = 0, g_pt = 0;   // 上一帧中心(x,y,z) + 时间
static double g_vx = 0, g_vy = 0;                       // 平滑后的速度 m/s
static double g_vpeak = 0;                              // 见过的最大速度（判断目标是否在动/换向）

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

// 目标消失/这一帧没有可打的板：速度估计作废，避免下一帧拿旧速度瞎预测
static void reset_velocity()
{
    g_has_prev = false;
    g_vx = 0.0;
    g_vy = 0.0;
    g_vpeak = 0.0;
    g_vy = 0.0;
}

// ---------- 回调 ----------

void on_armors(const ArmorCornersArray::SharedPtr msg)
{
    double tnow = rclcpp::Time(msg->header.stamp).seconds();
    if (msg->armors.empty()) {
        // 没有观测：规范要求这一帧必须报告"停控停火"。
        // 但如果只是短暂丢失，速度先留着（下次恢复还能接着用）；
        // 超过 0.25 s 都看不到，才彻底作废。
        publish_stop(msg);
        if (!g_has_prev || tnow - g_pt > 0.25) reset_velocity();
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

    // ---- 确定这一帧要瞄的目标点 ----
    // 情况一：有"够正对"的板 → 用它，并且更新速度参考点
    // 情况二：这一帧一块可打的板都没有（仿真在斜视时只发布侧板）→ 不要停火！
    //         用上一次正面观测的位置 + 速度外推，继续瞄。
    //         之前这里写的是"停火 + 清速度"，结果每次丢失都要重新学速度，
    //         中间那 1~2 发就白白打空了。
    double cx = 0, cy = 0, cz = 0;
    bool moved = false;
    if (best >= 0) {
        plate_center(msg->armors[best], cx, cy, cz);
        moved = !g_has_prev ||
                std::fabs(cx - g_px) > 1e-9 || std::fabs(cy - g_py) > 1e-9;
    } else if (g_has_prev && tnow - g_pt < 0.25) {
        double dt2 = tnow - g_pt;              // 距离上次正面观测过了多久
        cx = g_px + g_vx * dt2;                // 匀速外推（平移靶匀速，够用）
        cy = g_py + g_vy * dt2;
        cz = g_pz;                             // 高度不变
    } else {
        reset_velocity();
        publish_stop(msg);
        return;
    }

    // ---- 速度估计：与"位置真正变化过的那一帧"做差分 ----
    // 注意：桥接会以比仿真状态更新更快的频率重复发布同一帧（位置完全相同、
    // 时间戳却在新涨），这些重复帧的 dy = 0，如果直接拿去差分会把速度算成 0。
    // 所以这里先判断"位置有没有变"，变了才更新速度和参考点。
    if (moved) {
        if (g_has_prev) {
            double dt = tnow - g_pt;
            if (dt > 1e-4 && dt < 0.2) {                 // 间隔不合理就跳过
                // 四角没有噪声，直接差分就是准确速度，不再做低通——低通会引入滞后
                g_vx = (cx - g_px) / dt;
                g_vy = (cy - g_py) / dt;
                double sp_now = std::hypot(g_vx, g_vy);
                if (sp_now > g_vpeak) g_vpeak = sp_now;
                g_vy = (cy - g_py) / dt;
            }
        }
        g_px = cx; g_py = cy; g_pz = cz; g_pt = tnow; g_has_prev = true;
    }

    // ---- 预测：瞄准"子弹到达那一刻"目标所在的位置 ----
    double D_now = std::hypot(cx, cy);   // 当前水平距离
    double tf    = D_now / V;            // 粗算飞行时间（够用）
    double lead  = 0.060 + tf;           // 观测→开火的总延迟 + 飞行时间

    double ax = cx + g_vx * lead;        // 预测点
    double ay = cy + g_vy * lead;

    double D  = std::hypot(ax, ay);      // 后面的弹道解算全用预测点
    double dz = cz;                      // 沿水平方向平移，高度不变

    // ---- 解仰角（弹道 + 迭代 3 次） ----
    double e = 0.0;
    for (int i = 0; i < 3; ++i) {
        double d = D  - L * std::cos(e);
        double h = dz - L * std::sin(e);
        double disc = V*V*V*V - G * (G*d*d + 2.0*h*V*V);
        if (disc < 0.0) { printf("超出射程，打不到\n"); return; }
        e = std::atan2(V*V - std::sqrt(disc), G*d);
    }

    printf("选中第 %d 块 face=%.2f dist=%.2f v=(%+.2f,%+.2f) | D=%.3f dz=%.3f yaw=%.2f° pitch=%.2f°\n",
           best, best_face, best >= 0 ? best_dist : 0.0, g_vx, g_vy,
           D, dz,
           std::atan2(ay, ax) * 180.0 / M_PI,
           e * 180.0 / M_PI);

    // ---- 发指令 ----
    AimCommand cmd;
    cmd.header     = msg->header;
    cmd.session_id = msg->session_id;
    cmd.yaw        = std::atan2(ay, ax);   // 用预测点算 yaw
    cmd.pitch      = e;
    cmd.enable     = true;

    // 目标在动（曾经见过速度）但现在速度明显变小 => 正处在换向减速段，预测必然偏，先别开枪
    bool near_turn = (g_vpeak > 0.5) && (std::hypot(g_vx, g_vy) < 0.9 * g_vpeak);
    cmd.fire       = !near_turn;           // 开火（换向期暂停）
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
