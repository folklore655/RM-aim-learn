// 自瞄节点：订阅 /aim_sim/armors，发布 /aim_sim/command。
// ROS 侧 pitch 抬头为正；指令的 header/session_id 必须复制观测。
// 一次回调六步：反推车心 → 更新速度/角速度 → 预测位姿 → 选板 → 解仰角 → 发指令。

#include <cmath>
#include <cstdio>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rm_aim_sim_bridge/msg/armor_corners_array.hpp"
#include "rm_aim_sim_bridge/msg/aim_command.hpp"

using rm_aim_sim_bridge::msg::ArmorCornersArray;
using rm_aim_sim_bridge::msg::AimCommand;

rclcpp::Publisher<AimCommand>::SharedPtr g_pub;

const double V        = 23.0;          // 弹速 m/s
const double G        = 9.7833;        // 重力 m/s^2
const double L        = 0.22;          // 枪口前伸量
const double R_PLATE  = 0.2480696;     // 车心 → 板中心（步兵 3 预设）
const double TAU      = 6.283185307179586;

// ---------- 跨帧状态 ----------
static bool   g_has_prev = false;
static double g_Cx = 0, g_Cy = 0;      // 上一帧车心
static double g_pt_pos = 0;            // 上次位置变化时间
static double g_vx = 0, g_vy = 0;      // 平移速度
static double g_vpeak = 0;             // 最大平移速度，判换向减速
static double g_phi = 0;               // 上帧朝外方位角
static double g_pt_phi = 0;            // 上次朝向变化时间
static bool   g_has_prev_phi = false;
static double g_omega = 0;             // 自转角速度

// 折到 (-pi, pi]
static double wrap_pi(double a)
{
    while (a >  M_PI) a -= TAU;
    while (a < -M_PI) a += TAU;
    return a;
}

// ---------- 工具函数 ----------

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

// 上边 × 左边归一化，方向由下面 voting 定
static void plate_normal(const rm_aim_sim_bridge::msg::ArmorCorners & a,
                         double & nx, double & ny, double & nz)
{
    const auto & p = a.corners;
    double ax = p[1].x - p[0].x, ay = p[1].y - p[0].y, az = p[1].z - p[0].z;
    double bx = p[3].x - p[0].x, by = p[3].y - p[0].y, bz = p[3].z - p[0].z;
    nx = ay * bz - az * by;
    ny = az * bx - ax * bz;
    nz = ax * by - ay * bx;
    double len = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (len > 1e-9) { nx /= len; ny /= len; nz /= len; }
}

// 停控停火，header 复制观测
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

static void reset_state()
{
    g_has_prev = false;
    g_has_prev_phi = false;
    g_vx = 0.0; g_vy = 0.0; g_vpeak = 0.0;
    g_omega = 0.0;
}

// ---------- 回调 ----------

void on_armors(const ArmorCornersArray::SharedPtr msg)
{
    double tnow = rclcpp::Time(msg->header.stamp).seconds();

    if (msg->armors.empty()) {
        publish_stop(msg);
        if (!g_has_prev || tnow - g_pt_pos > 0.25) reset_state();
        return;
    }

    // 1. 车心 C = p − r·n。n 块板 2^n 种符号组合，取各板反推最一致的一组；单板靠上帧消歧。
    const size_t n = msg->armors.size();
    std::vector<double> px(n), py(n), pz(n), phi(n);
    for (size_t i = 0; i < n; ++i) {
        double nx, ny, nz;
        plate_center(msg->armors[i], px[i], py[i], pz[i]);
        plate_normal(msg->armors[i], nx, ny, nz);
        phi[i] = std::atan2(ny, nx);
    }

    double Cx = 0.0, Cy = 0.0;
    int    bestMask = 0;
    {
        const int combos = 1 << static_cast<int>(n);
        double bestCost = 1e18;
        for (int mask = 0; mask < combos; ++mask) {
            double mx = 0.0, my = 0.0;
            for (size_t i = 0; i < n; ++i) {
                double s = ((mask >> i) & 1) ? -1.0 : 1.0;
                mx += px[i] - R_PLATE * s * std::cos(phi[i]);
                my += py[i] - R_PLATE * s * std::sin(phi[i]);
            }
            mx /= static_cast<double>(n);
            my /= static_cast<double>(n);

            double spread = 0.0;
            double maxFace = -2.0;
            for (size_t i = 0; i < n; ++i) {
                double s = ((mask >> i) & 1) ? -1.0 : 1.0;
                double ux = s * std::cos(phi[i]);
                double uy = s * std::sin(phi[i]);
                spread += std::hypot(px[i] - R_PLATE * ux - mx,
                                     py[i] - R_PLATE * uy - my);
                double dl = std::hypot(px[i], py[i]);
                if (dl > 1e-6) {
                    double f = -(ux * px[i] + uy * py[i]) / dl;
                    if (f > maxFace) maxFace = f;
                }
            }

            double cost = spread * static_cast<double>(n) * 8.0
                        + (g_has_prev ? std::hypot(mx - g_Cx, my - g_Cy) : 0.0)
                        + 0.20 * (1.0 - maxFace);
            if (cost < bestCost) {
                bestCost = cost;
                Cx = mx; Cy = my; bestMask = mask;
            }
        }
    }

    std::vector<double> rphi(n);
    for (size_t i = 0; i < n; ++i) {
        double s = ((bestMask >> i) & 1) ? -1.0 : 1.0;
        rphi[i] = std::atan2(s * std::sin(phi[i]), s * std::cos(phi[i]));
    }

    // 2. 重复帧：位置没变不算速度，朝向没变不算转速（桥接会用新时间戳重发同一帧）。
    const bool pos_moved = !g_has_prev ||
                           std::fabs(Cx - g_Cx) > 1e-6 ||
                           std::fabs(Cy - g_Cy) > 1e-6;
    const bool phi_moved = !g_has_prev_phi ||
                           std::fabs(wrap_pi(rphi[0] - g_phi)) > 1e-4;

    // 相邻帧的板可能不是同一块，用 k·90° 找最接近的相位差
    double dphi = 0.0, dmin = 1e9;
    if (g_has_prev_phi) {
        for (size_t i = 0; i < n; ++i) {
            for (int k = 0; k < 4; ++k) {
                double d = wrap_pi(rphi[i] - (g_phi + k * M_PI * 0.5));
                if (std::fabs(d) < dmin) { dmin = std::fabs(d); dphi = d; }
            }
        }
    }

    if (pos_moved) {
        if (g_has_prev) {
            double dt = tnow - g_pt_pos;
            if (dt > 1e-4 && dt < 0.2) {
                double vx = (Cx - g_Cx) / dt;
                double vy = (Cy - g_Cy) / dt;
                if (std::hypot(vx, vy) < 8.0) {   // 剔除跳变帧
                    g_vx = vx; g_vy = vy;
                    double sp = std::hypot(g_vx, g_vy);
                    if (sp > g_vpeak) g_vpeak = sp;
                }
            }
        }
        g_Cx = Cx; g_Cy = Cy; g_pt_pos = tnow; g_has_prev = true;
    }

    if (phi_moved) {
        if (g_has_prev_phi) {
            double dt = tnow - g_pt_phi;
            if (dt > 1e-4 && dt < 0.1) {
                double w = dphi / dt;
                if (std::fabs(w) < 25.0) g_omega = 0.8 * g_omega + 0.2 * w;
            }
        }
        g_phi = rphi[0]; g_pt_phi = tnow; g_has_prev_phi = true;
    }

    // 3. 预测 lead 后位置：绕车心精确旋转（线性外推丢 R(1−cosΔθ)，69° 时 0.158 m）。
    //    lead = 0.015 指令生效延迟 + 飞行时间；不补射击周期，子弹用出膛瞬间的指令。
    double D_now = std::hypot(Cx, Cy);
    double tf    = D_now / V;
    double lead  = 0.015 + tf;
    double dth   = g_omega * lead;
    double cs = std::cos(dth), sn = std::sin(dth);

    double best_face = -2.0, second_face = -2.0;
    int    best_idx = -1;
    double aim_x = 0, aim_y = 0, aim_z = 0;

    for (size_t i = 0; i < n; ++i) {
        double rx = px[i] - Cx, ry = py[i] - Cy;
        double ax = Cx + rx * cs - ry * sn + g_vx * lead;
        double ay = Cy + rx * sn + ry * cs + g_vy * lead;
        double az = pz[i];

        double np = rphi[i] + dth;
        double vx = -ax, vy = -ay;
        double dl = std::sqrt(vx * vx + vy * vy);
        if (dl < 1e-6) continue;
        double face = (std::cos(np) * vx + std::sin(np) * vy) / dl;

        if (face > best_face) {
            second_face = best_face;
            best_face = face;
            best_idx  = static_cast<int>(i);
            aim_x = ax; aim_y = ay; aim_z = az;
        } else if (face > second_face) {
            second_face = face;
        }
    }

    if (best_idx < 0) {
        publish_stop(msg);
        return;
    }

    // 4. 弹道低伸解，枪口偏移迭代 3 次
    double D  = std::hypot(aim_x, aim_y);
    double dz = aim_z;

    double e = 0.0;
    for (int i = 0; i < 3; ++i) {
        double d = D  - L * std::cos(e);
        double h = dz - L * std::sin(e);
        double disc = V*V*V*V - G * (G*d*d + 2.0*h*V*V);
        if (disc < 0.0) { printf("超出射程，打不到\n"); return; }
        e = std::atan2(V*V - std::sqrt(disc), G*d);
    }

    // 5. 开火门限：正对 >0.8、与次优差 >0.15、不在换向减速段
    bool facing_ok = (best_face > 0.80) && ((best_face - second_face) > 0.15);
    bool near_turn = (g_vpeak > 0.5) && (std::hypot(g_vx, g_vy) < 0.9 * g_vpeak);

    printf("face=%.2f f2=%.2f p%d w=%+.2f v=(%+.2f,%+.2f) C=(%.3f,%+.3f) | D=%.3f dz=%.3f yaw=%.2f° pitch=%.2f° fire=%d\n",
           best_face, second_face, best_idx, g_omega, g_vx, g_vy, Cx, Cy,
           D, dz,
           std::atan2(aim_y, aim_x) * 180.0 / M_PI,
           e * 180.0 / M_PI,
           (facing_ok && !near_turn) ? 1 : 0);

    // 6. 发指令
    AimCommand cmd;
    cmd.header     = msg->header;
    cmd.session_id = msg->session_id;
    cmd.yaw        = std::atan2(aim_y, aim_x);
    cmd.pitch      = e;
    cmd.enable     = true;
    cmd.fire       = facing_ok && !near_turn;
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
