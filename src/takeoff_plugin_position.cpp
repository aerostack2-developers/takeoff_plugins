#include "takeoff_base.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace takeoff_plugin_position
{
    class Plugin : public takeoff_base::TakeOffBase
    {
    private:
        void ownInit(as2::Node *node_ptr)
        {
            twist_pub_ = node_ptr_->create_publisher<geometry_msgs::msg::TwistStamped>(
                node_ptr_->generate_global_name(as2_names::topics::motion_reference::twist), 
                as2_names::topics::motion_reference::qos);
        }

    public:
        rclcpp_action::GoalResponse onAccepted(const std::shared_ptr<const as2_msgs::action::TakeOff::Goal> goal) override
        {
            desired_speed_ = goal->takeoff_speed;
            desired_height_ = goal->takeoff_height;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse onCancel(const std::shared_ptr<GoalHandleTakeoff> goal_handle) override
        {
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        bool onExecute(const std::shared_ptr<GoalHandleTakeoff> goal_handle) override
        {
            rclcpp::Rate loop_rate(10);
            const auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<as2_msgs::action::TakeOff::Feedback>();
            auto result = std::make_shared<as2_msgs::action::TakeOff::Result>();

            // Check if goal is done
            while ((desired_height_ - actual_heigth_) > 0 + this->takeoff_height_threshold_)
            {
                if (goal_handle->is_canceling())
                {
                    result->takeoff_success = false;
                    goal_handle->canceled(result);
                    RCLCPP_INFO(node_ptr_->get_logger(), "Goal canceled");
                    // TODO: change this to hover

                    return false;
                }

                geometry_msgs::msg::TwistStamped msg;
                msg.header.stamp = node_ptr_->now();
                msg.header.frame_id = "enu";
                msg.twist.linear.z = desired_speed_;
                twist_pub_->publish(msg);

                feedback->actual_takeoff_height = actual_heigth_;
                feedback->actual_takeoff_speed = actual_z_speed_;
                goal_handle->publish_feedback(feedback);

                loop_rate.sleep();
            }

            result->takeoff_success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(node_ptr_->get_logger(), "Goal succeeded");
            // TODO: change this to hover

            geometry_msgs::msg::TwistStamped msg;
            msg.header.stamp = node_ptr_->now();
            msg.header.frame_id = "enu";
            msg.twist.linear.z = 0;
            twist_pub_->publish(msg);
            return true;
        }

    private:
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    }; // Plugin class
} // takeoff_plugin_position namespace

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(takeoff_plugin_position::Plugin, takeoff_base::TakeOffBase)