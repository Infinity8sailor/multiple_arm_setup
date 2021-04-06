#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "multiple_arms_cpp_tutorial");
    ros::NodeHandle nh("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    double ros_rate;
    nh.param("ros_rate", ros_rate, 0.2); // 0.2 Hz = 5 seconds
    ros::Rate* loop_rate_ = new ros::Rate(ros_rate);

    // BEGIN_TUTORIAL
    //
    //
    // Define Planning Group name and end effectors
    std::string movegroup_name, ee_link_1, ee_link_2;

    nh.param<std::string>("move_group", movegroup_name, "dual_ur");
    nh.param<std::string>("ee_link1", ee_link_1, "ur5_1_gripper_finger1_finger_tip_link");
    nh.param<std::string>("ee_link2", ee_link_2, "ur5_2_gripper_finger1_finger_tip_link");

    // Initialising and defining the planning group for move_base
    moveit::planning_interface::MoveGroupInterface move_group(movegroup_name);
    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const robot_state::JointModelGroup *joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(movegroup_name);

    // Defining Multiple Target Poses for each Arm
    geometry_msgs::Pose target_pose_1, target_pose_2;
    target_pose_1.position.x = 0.5;
    target_pose_1.position.y = 0.0;
    target_pose_1.position.z = 0.8;
    target_pose_1.orientation.y = sqrt(2)/2;
    target_pose_1.orientation.w = sqrt(2)/2;

    target_pose_2.position.x = 0.5;
    target_pose_2.position.y = -0.7;
    target_pose_2.position.z = 1.2;
    target_pose_2.orientation.y = sqrt(2)/2;
    target_pose_2.orientation.w = sqrt(2)/2;


    //set Target poses or Pre-defined poses
    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(target_pose_1, ee_link_1);
    move_group.setPoseTarget(target_pose_2, ee_link_2);
    ROS_INFO_NAMED("multiple_arms_cpp_tutorial", "Setting the target position to x=%g, y=%g, z=%g",target_pose_1.position.x, target_pose_1.position.y, target_pose_1.position.z);


    // Initialising a new plan and planning
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // Executing the movement
    moveit::planning_interface::MoveItErrorCode motion_done = moveit_msgs::MoveItErrorCodes::FAILURE;

    if (success) {
        motion_done = move_group.execute(my_plan);
        }
    else {
        ROS_WARN("Something went wrong moving the robots to home position.");
    }
    
    // END_TUTORIAL
    ros::Duration(10.0).sleep(); // 2 seconds
    ros::shutdown();
    return 0;
}