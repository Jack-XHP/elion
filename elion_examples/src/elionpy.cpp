#include <fstream>
#include <string>

#include <ros/package.h>
#include <ros/ros.h>
#include <ros/publisher.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/MotionPlanResponse.h>

#include "elion_examples/util.h"
#include "elion_examples/json_util.h"
#include "elion_examples/rviz_util.h"
#include "elion_examples/elionpy.h"


const std::string BASE_CLASS = "planning_interface::PlannerManager";

class Plan{
    public:
        ros::NodeHandle node_handle;
        planning_interface::PlannerManagerPtr planner_instance;
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        ros::Publisher display_publisher;
        std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools;
        std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm;
        moveit::core::RobotModelPtr robot_model;

        const moveit::core::JointModelGroup* joint_model_group;

        Plan(const std::string robot_description, const std::string planning_group, const std::string fixed_frame, std::string planning_plugin_name){
            robot_model_loader::RobotModelLoaderPtr robot_model_loader(
                    new robot_model_loader::RobotModelLoader(robot_description));
            robot_model = robot_model_loader->getModel();
            moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
            joint_model_group = robot_state->getJointModelGroup(planning_group);

            elion::ClassLoaderSPtr planner_plugin_loader;

            elion::loadPlanningPlugin(planner_plugin_loader, planner_instance, robot_model, node_handle, BASE_CLASS,planning_plugin_name);
            // Use the default planning scene published by the move group node.

            psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(robot_description);
            bool has_planning_scene = psm->requestPlanningSceneState("/get_planning_scene");
            ROS_INFO_STREAM("Request planning scene " << (has_planning_scene ? "succeeded." : "failed."));
            psm->startSceneMonitor("/move_group/monitored_planning_scene");

            // Visualization
            // ^^^^^^^^^^^^^
            namespace rvt = rviz_visual_tools;
            visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>(fixed_frame, "/visualization_marker_array", psm);
            visual_tools->loadRobotStatePub("/display_robot_state");
            // visual_tools.loadTrajectoryPub();
            visual_tools->deleteAllMarkers();
            ros::Duration(1.0).sleep();

            display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
        };

        bool planning(elion_examples::elionpy::Request& req, elion_examples::elionpy::Response& res){
            moveit_msgs::MotionPlanRequest request = req.request;
            moveit_msgs::RobotState goal = req.goal;
            std::vector<moveit_msgs::CollisionObject> collisions = req.collisions;
            moveit_msgs::RobotState start = request.start_state;
            if (collisions.size() > 0){
                planning_scene_interface.addCollisionObjects(collisions);
            }
            psm->getPlanningScene()->printKnownObjects();
            auto CO = planning_scene_interface.getObjects();
            for (auto const& x : CO){
                ROS_INFO_STREAM("Visualized object" << x.first);
                elion::showCollsion(x.second, *visual_tools);
            }
            moveit_msgs::MotionPlanRequest req1;
            req1 = elion::createPTPProblem(start.joint_state.position, goal.joint_state.position, robot_model, joint_model_group, *visual_tools);
            req1.path_constraints = request.path_constraints;
            req1.allowed_planning_time = request.allowed_planning_time;       // 5.0 default planning time
            req1.planner_id = request.planner_id;  // RRTConnect as default planner

            bool success{ false };

            planning_interface::MotionPlanResponse res1;
            auto context1 = planner_instance->getPlanningContext(psm->getPlanningScene(), req1, res1.error_code_);
            if (context1 != nullptr){
                success = context1->solve(res1);
            }else{
                ROS_INFO_STREAM("Failed to create planning constext for the first problem.");
            }
            if (res1.trajectory_ && success){
                ROS_INFO_STREAM("Path found for position constraints of length: " << res1.trajectory_->getWayPointCount());
                elion::displaySolution(res1, joint_model_group, *visual_tools,
                                       (req1.path_constraints.orientation_constraints.size() > 0));
                moveit_msgs::DisplayTrajectory display_trajectory;
                moveit_msgs::MotionPlanResponse response;
                res1.getMessage(response);
                display_trajectory.trajectory_start = response.trajectory_start;
                display_trajectory.trajectory.push_back(response.trajectory);
                display_publisher.publish(display_trajectory);
                res.trajectory = response;
                return true;
            }else{
                return false;
            }
        };
};

int main(int argc, char** argv){
  ros::init(argc, argv, "elionpy");
  ros::AsyncSpinner spinner(0);
  spinner.start();
  Plan planner("robot_description", "panda_arm", "panda_link0", "elion/ElionPlanner");
  ros::ServiceServer service = planner.node_handle.advertiseService("constraint_planning", &Plan::planning, &planner);
  ros::waitForShutdown();
}