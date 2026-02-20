#include "mvp_localization_backend.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MvpLocalizationBackend>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}