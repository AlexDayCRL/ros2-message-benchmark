#include <algorithm>
#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <cfloat>

#include <spdlog/spdlog.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace
{
    template <typename T>
    class Talker : public rclcpp::Node
    {
    public:
        Talker(T payload,
               float dataRateHz = 10,
               const std::string &nodeName = "talker",
               const std::string &topic = "topic")
            : Node(nodeName),
              payload_(payload)
        {
            pub_ = this->create_publisher<T>(topic, rclcpp::SensorDataQoS());

            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(static_cast<long int>(1 / dataRateHz * 1000)),
                std::bind(&Talker::timerCallback, this));
        }

        void timerCallback()
        {
            pub_->publish(payload_);
        }

    private:
        typename rclcpp::Publisher<T>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        T payload_;
        size_t seqNum_{0};
    };

    std::shared_ptr<Talker<sensor_msgs::msg::PointCloud2>> make_pcd2_talker(
        size_t pcdSize, float dataRateHz, std::string nodeName, std::string topic)
    {
        auto msg = sensor_msgs::msg::PointCloud2{};
        sensor_msgs::PointCloud2Modifier mod(msg);

        msg.height = 1;
        msg.width = pcdSize;
        mod.setPointCloud2FieldsByString(1, "xyz");
        mod.resize(msg.height * msg.width);

        // Now create iterators for fields
        sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
        {
            *iter_x = static_cast<float>(rand());
            *iter_y = static_cast<float>(rand());
            *iter_z = static_cast<float>(rand());
        }

        return std::make_shared<
            Talker<sensor_msgs::msg::PointCloud2>>(msg, dataRateHz, nodeName, topic);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    std::vector<std::shared_ptr<Talker<sensor_msgs::msg::PointCloud2>>> pcd2_talkers{};
    for (size_t i = 0; i < 3; i++) {
        spdlog::info("making pcd2 talker {}", i);
        auto talker = make_pcd2_talker(std::stoi(argv[argc - 1]), 10, "talker" + std::to_string(i), "~/pcd");
        pcd2_talkers.push_back(talker);
        executor.add_node(talker);
    }

    spdlog::info("starting to spin");
    executor.spin();
    rclcpp::shutdown();

    return 0;
}
