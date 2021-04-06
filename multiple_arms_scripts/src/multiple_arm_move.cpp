#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

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
    std::string movegroup_name, ee_link_1, ee_link_2, ee_link_3;

    nh.param<std::string>("move_group", movegroup_name, "multiple_ur");
    nh.param<std::string>("ee_link1", ee_link_1, "ur5_1_gripper_finger1_finger_tip_link");
    nh.param<std::string>("ee_link2", ee_link_2, "ur5_2_gripper_finger1_finger_tip_link");
    nh.param<std::string>("ee_link3", ee_link_3, "ur5_3_gripper_finger1_finger_tip_link");

    // Initialising and defining the planning group for move_base
    moveit::planning_interface::MoveGroupInterface move_group(movegroup_name);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const robot_state::JointModelGroup *joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(movegroup_name);

    move_group.setStartStateToCurrentState();
    move_group.setNamedTarget("all_zero");
    // Initialising a new plan and planning
    bool success_0 = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // Executing the movement
    moveit::planning_interface::MoveItErrorCode motion_done = moveit_msgs::MoveItErrorCodes::FAILURE;

    if (success_0) {
        motion_done = move_group.execute(my_plan);
        }
    else {
        ROS_WARN("Something went wrong moving the robots to home position.");
    }
    
    ros::Duration(0.5).sleep(); // 2 seconds

    //Adding table constrain to planning Scene
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame();
    collision_object.id = "Table";
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.75;
    primitive.dimensions[1] = 1.4;
    primitive.dimensions[2] = 0.85;

    geometry_msgs::Pose table_pose;
    table_pose.position.x = 0.8;
    table_pose.position.y = 0;
    table_pose.position.z = 0.0;
    table_pose.orientation.y = 1.56;
    
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(table_pose);
    collision_object.operation = collision_object.ADD;
    
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    ROS_INFO_NAMED("multiple_arms_cpp_tutorial", "Added an object into the world");
    planning_scene_interface.addCollisionObjects(collision_objects);


    // Defining Multiple Target Poses for each Arm
    geometry_msgs::Pose target_pose_1, target_pose_2, target_pose_3;
    target_pose_1.position.x = 0.8;
    target_pose_1.position.y = 0.3;
    target_pose_1.position.z = 0.7;
    target_pose_1.orientation.y = 1;
    target_pose_1.orientation.w = 1;

    target_pose_2.position.x = 0.6;
    target_pose_2.position.y = -0.3;
    target_pose_2.position.z = 0.7;
    target_pose_2.orientation.y = 1;
    target_pose_2.orientation.w = 1;

    target_pose_3.position.x = 1.1;
    target_pose_3.position.y = -0.3;
    target_pose_3.position.z = 0.8;
    target_pose_3.orientation.y = 0;
    target_pose_3.orientation.w = 0;


    //set Target poses or Pre-defined poses
    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(target_pose_1, ee_link_1);
    move_group.setPoseTarget(target_pose_2, ee_link_2);
    move_group.setPoseTarget(target_pose_3, ee_link_3);

    ROS_INFO_NAMED("multiple_arms_cpp_tutorial", "Setting the target position to x=%g, y=%g, z=%g",target_pose_1.position.x, target_pose_1.position.y, target_pose_1.position.z);


    // Initialising a new plan and planning
    // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success_1 = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // Executing the movement
    motion_done = moveit_msgs::MoveItErrorCodes::FAILURE;

    if (success_1) {
        motion_done = move_group.execute(my_plan);
        }
    else {
        ROS_WARN("Something went wrong moving the robots to home position.");
    }
    
    ros::Duration(2.0).sleep(); // 2 seconds

    move_group.setStartStateToCurrentState();
    move_group.setNamedTarget("all_up");
    // Initialising a new plan and planning
    bool success_2 = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // Executing the movement
    motion_done = moveit_msgs::MoveItErrorCodes::FAILURE;

    if (success_2) {
        motion_done = move_group.execute(my_plan);
        }
    else {
        ROS_WARN("Something went wrong moving the robots to home position.");
    }
    
    ros::Duration(1.0).sleep(); // 2 seconds


    // END_TUTORIAL
    ros::shutdown();
    return 0;
}