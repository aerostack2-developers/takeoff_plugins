#include "takeoff_base.hpp"
#include "as2_motion_command_handlers/speed_motion.hpp"

namespace takeoff_plugins
{
    class TakeOffSpeed : public takeoff_base::TakeOffBase
    {
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

            static as2::motionCommandsHandlers::SpeedMotion motion_handler(node_ptr_);

            // Check if goal is done
            while ((desired_height_ - actual_heigth_) > 0 + this->takeoff_height_threshold_)
            {
                if (goal_handle->is_canceling())
                {
                    result->takeoff_success = false;
                    goal_handle->canceled(result);
                    RCLCPP_INFO(node_ptr_->get_logger(), "Goal canceled");
                    // TODO: change this to hover
                    motion_handler.sendSpeedCommandWithYawSpeed(0.0, 0.0, 0.0, 0.0);
                    return false;
                }

                motion_handler.sendSpeedCommandWithYawSpeed(0.0, 0.0, desired_speed_, 0.0);

                feedback->actual_takeoff_height = actual_heigth_;
                feedback->actual_takeoff_speed = actual_z_speed_;
                goal_handle->publish_feedback(feedback);

                loop_rate.sleep();
            }

            result->takeoff_success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(node_ptr_->get_logger(), "Goal succeeded");
            // TODO: change this to hover
            motion_handler.sendSpeedCommandWithYawSpeed(0.0, 0.0, 0.0, 0.0);
            return true;
        }
    }; // TakeOffSpeed class
} // takeoff_plugins namespace

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(takeoff_plugins::TakeOffSpeed, takeoff_base::TakeOffBase)