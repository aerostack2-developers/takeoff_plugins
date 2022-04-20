#include "takeoff_base.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace takeoff_plugins
{
    class TakeOffSpeed : public takeoff_base::TakeOffBase
    {
    private:
        void ownInit(as2::Node *node_ptr)
        {
            // TODO: motion_ref speed
            traj_pub_ = node_ptr_->create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>(
                node_ptr_->generate_global_name(as2_names::topics::motion_reference::trajectory), as2_names::topics::motion_reference::qos);
        }

    public:
        rclcpp_action::GoalResponse onAccepted(const std::shared_ptr<const as2_msgs::action::TakeOff::Goal> goal) override
        {
            if (goal->takeoff_height < 0.0f)
            {
                RCLCPP_ERROR(node_ptr_->get_logger(), "TakeOffBehaviour: Invalid takeoff height");
                return rclcpp_action::GoalResponse::REJECT;
            }

            if (goal->takeoff_speed < 0.0f)
            {
                RCLCPP_ERROR(node_ptr_->get_logger(), "TakeOffBehaviour: Invalid takeoff speed");
                return rclcpp_action::GoalResponse::REJECT;
            }

            desired_speed_ = (goal->takeoff_speed != 0.0f) ? goal->takeoff_speed : DEFAULT_TAKEOFF_SPEED;
            desired_height_ = (goal->takeoff_height != 0.0f) ? goal->takeoff_height : DEFAULT_TAKEOFF_ALTITUDE;

            RCLCPP_INFO(node_ptr_->get_logger(), "TakeOffBehaviour: TakeOff with speed %f and height %f", desired_speed_, desired_height_);
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse onCancel(const std::shared_ptr<GoalHandleTakeoff> goal_handle) override
        {
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void onExecute(const std::shared_ptr<GoalHandleTakeoff> goal_handle) override
        {
            RCLCPP_INFO(node_ptr_->get_logger(), "Executing goal");

            rclcpp::Rate loop_rate(10);
            const auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<as2_msgs::action::TakeOff::Feedback>();
            auto result = std::make_shared<as2_msgs::action::TakeOff::Result>();

            rclcpp::Time start_time = node_ptr_->now();

            // Check if goal is done
            while ((desired_height_ - actual_heigth_) > 0 + TAKEOFF_HEIGHT_THRESHOLD)
            {
                if (goal_handle->is_canceling())
                {
                    result->takeoff_success = false;
                    goal_handle->canceled(result);
                    RCLCPP_INFO(node_ptr_->get_logger(), "Goal canceled");
                    // TODO: change this to hover

                    return;
                }

                rclcpp::Time t = node_ptr_->now();
                trajectory_msgs::msg::JointTrajectoryPoint msg;
                msg.time_from_start.sec = t.seconds() - start_time.seconds();
                msg.time_from_start.nanosec = t.nanoseconds() - start_time.nanoseconds();
                msg.positions = {0.0, 0.0, 0.0, 0.0};
                msg.velocities = {0.0, 0.0, 0.0, desired_speed_};
                msg.accelerations = {0.0, 0.0, 0.0, 0.0};
                traj_pub_->publish(msg);

                // RCLCPP_INFO(this->get_logger(), "Publish feedback");
                feedback->actual_takeoff_height = actual_heigth_;
                feedback->actual_takeoff_speed = actual_z_speed_;
                goal_handle->publish_feedback(feedback);

                loop_rate.sleep();
            }

            result->takeoff_success = true;

            goal_handle->succeed(result);
            RCLCPP_INFO(node_ptr_->get_logger(), "Goal succeeded");
            // TODO: change this to hover

            trajectory_msgs::msg::JointTrajectoryPoint msg;
            msg.positions = {0.0, 0.0, 0.0, 0.0};
            msg.velocities = {0.0, 0.0, 0.0, 0.0};
            msg.accelerations = {0.0, 0.0, 0.0, 0.0};
            traj_pub_->publish(msg);
        }

    private:
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr traj_pub_;
    };

}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(takeoff_plugins::TakeOffSpeed, takeoff_base::TakeOffBase)