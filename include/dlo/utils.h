/***********************************************************
 *                                                         *
 * Copyright (c)                                           *
 *                                                         *
 * The Verifiable & Control-Theoretic Robotics (VECTR) Lab *
 * University of California, Los Angeles                   *
 *                                                         *
 * Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez   *
 * Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu         *
 *                                                         *
 ***********************************************************/

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <Eigen/Geometry>


namespace dlo {

    template <typename T>
    struct identity { typedef T type; };

    template <typename T>
    void declare_param(rclcpp::Node* node, const std::string param_name, T& param, const typename identity<T>::type& default_value) {
        node->declare_parameter(param_name, default_value);
        node->get_parameter(param_name, param);
    }

    inline Eigen::Matrix4f poseMsgToEigen(const geometry_msgs::msg::Pose& pose) {
        Eigen::Translation3f translation(
            static_cast<float>(pose.position.x),
            static_cast<float>(pose.position.y),
            static_cast<float>(pose.position.z)
        );

        Eigen::Quaternionf rotation(
            static_cast<float>(pose.orientation.w),
            static_cast<float>(pose.orientation.x),
            static_cast<float>(pose.orientation.y),
            static_cast<float>(pose.orientation.z)
        );

        return (translation * rotation).matrix();
    }

}
