#include <chrono>
#include <cstdlib>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

using namespace unitree::robot;
namespace fs = std::filesystem;

static std::ofstream odom_csv_file;
static std::mutex odom_csv_mutex;

static uint64_t CurrentTimestampMs()
{
    const auto now = std::chrono::system_clock::now();
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count());
}

static void WriteCsvHeader()
{
    odom_csv_file
        << "timestamp_ms,"
        << "position_x,position_y,position_z,"
        << "velocity_x,velocity_y,velocity_z,"
        << "quaternion_w,quaternion_x,quaternion_y,quaternion_z,"
        << "roll,pitch,yaw,"
        << "gyro_x,gyro_y,gyro_z,"
        << "acc_x,acc_y,acc_z,"
        << "yaw_speed,body_height\n";
}

static fs::path ExpandUserPath(const std::string &path)
{
    if (!path.empty() && path[0] == '~')
    {
        const char *home = std::getenv("HOME");
        if (home != nullptr)
        {
            if (path.size() == 1)
            {
                return fs::path(home);
            }
            if (path[1] == '/')
            {
                return fs::path(home) / path.substr(2);
            }
        }
    }

    return fs::path(path);
}

static void SportStateHandler(const void *message)
{
    const auto *state =
        static_cast<const unitree_go::msg::dds_::SportModeState_ *>(message);

    const auto &p = state->position();
    const auto &v = state->velocity();
    const auto &q = state->imu_state().quaternion();
    const auto &rpy = state->imu_state().rpy();
    const auto &gyro = state->imu_state().gyroscope();
    const auto &acc = state->imu_state().accelerometer();

    {
        std::lock_guard<std::mutex> lock(odom_csv_mutex);
        if (odom_csv_file.is_open())
        {
            odom_csv_file
                << CurrentTimestampMs() << ","
                << p[0] << "," << p[1] << "," << p[2] << ","
                << v[0] << "," << v[1] << "," << v[2] << ","
                << q[0] << "," << q[1] << "," << q[2] << "," << q[3] << ","
                << rpy[0] << "," << rpy[1] << "," << rpy[2] << ","
                << gyro[0] << "," << gyro[1] << "," << gyro[2] << ","
                << acc[0] << "," << acc[1] << "," << acc[2] << ","
                << state->yaw_speed() << "," << state->body_height()
                << std::endl;
        }
    }

    std::cout << "--------------- sportmodestate ---------------\n";

    std::cout << "position xyz: "
              << p[0] << ", "
              << p[1] << ", "
              << p[2] << "\n";

    std::cout << "velocity xyz: "
              << v[0] << ", "
              << v[1] << ", "
              << v[2] << "\n";

    std::cout << "imu quaternion wxyz: "
              << q[0] << ", "
              << q[1] << ", "
              << q[2] << ", "
              << q[3] << "\n";

    std::cout << "imu rpy: "
              << rpy[0] << ", "
              << rpy[1] << ", "
              << rpy[2] << "\n";

    std::cout << "gyro xyz: "
              << gyro[0] << ", "
              << gyro[1] << ", "
              << gyro[2] << "\n";

    std::cout << "acc xyz: "
              << acc[0] << ", "
              << acc[1] << ", "
              << acc[2] << "\n";

    std::cout << "yaw_speed: "
              << state->yaw_speed() << "\n";

    std::cout << "body_height: "
              << state->body_height() << "\n";

    std::cout << std::flush;
}

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <network_interface> [csv_file]\n";
        std::cerr << "Example: " << argv[0] << " enp7s0 go2_odom_data.csv\n";
        return 1;
    }

    const std::string iface = argv[1];
    const std::string csv_file_name = argc >= 3 ? argv[2] : "~/unitree_sdk2/example/go2/go2_odom_data.csv";
    const fs::path csv_file_path = fs::absolute(ExpandUserPath(csv_file_name));

    if (csv_file_path.has_parent_path())
    {
        fs::create_directories(csv_file_path.parent_path());
    }

    odom_csv_file.open(csv_file_path);
    if (!odom_csv_file.is_open())
    {
        std::cerr << "Failed to open CSV file: " << csv_file_path << "\n";
        return 1;
    }
    WriteCsvHeader();

    ChannelFactory::Instance()->Init(0, iface);

    ChannelSubscriber<unitree_go::msg::dds_::SportModeState_> subscriber(
        "rt/sportmodestate");

    subscriber.InitChannel(SportStateHandler);

    std::cout << "Listening to rt/sportmodestate on " << iface << "\n";
    std::cout << "Recording odom data to " << csv_file_path << "\n";
    std::cout << "Press Ctrl+C to exit.\n";

    while (true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}
