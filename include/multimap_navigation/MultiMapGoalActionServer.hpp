#ifndef MULTI_MAP_GOAL_SERVER_HPP
#define MULTI_MAP_GOAL_SERVER_HPP

#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h> 
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/LoadMap.h>
#include <multimap_navigation/SendGoalAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/tf.h>
#include <sqlite3.h>
#include <string>
#include <iostream>
#include <optional>

struct MapBounds {
    double min_x;
    double max_x;
    double min_y;
    double max_y;
};

class MultiMapGoalActionServer
{
public:
    /**
     * @brief Constructor for the MultiMapGoalActionServer class
     * @param name Name of the action server
     */
    explicit MultiMapGoalActionServer(const std::string &name);

    /**
     * @brief Destructor for the MultiMapGoalActionServer class
     */
    ~MultiMapGoalActionServer();

    /**
     * @brief Callback function when a new goal is received
     * @param goal Pointer to the received goal
     */
    void executeCB(const multimap_navigation::SendGoalGoalConstPtr &goal);



private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<multimap_navigation::SendGoalAction> action_server_;
    std::string action_name_;

    std::string current_map_;
    std::string wormhole_db_path_;
    std::string maps_path_;
    std::string change_map_service_;
    std::string clear_costmaps_service_;
    std::string amcl_update_service_;
    std::string initial_pose_topic_;
    std::string frame_id_;

    /**
     * @brief Check if a pose is within the bounds of the current map
     * @param pose Pose to check
     * @return True if the pose is within bounds, False otherwise
     */
    bool isPoseWithinMapBounds(const geometry_msgs::Pose& pose);

    /**
     * @brief Function to get the wormhole points between two maps
     * @param db_path Path to the SQLite database
     * @param current_map Name of the current map
     * @param target_map Name of the goal map
     * @return Optional containing the wormhole pose in current map if exists
     */
    std::optional<geometry_msgs::PoseStamped> getWormholePose(
        const std::string& db_path,
        const std::string& current_map,
        const std::string& target_map);

    /**
     * @brief Switch to a new map by calling the load map service
     * @param map_name Name of the map to switch to
     * @return True if switch was successful, False otherwise
     */
    bool switchToNewMap(const std::string &map_name);

    /**
     * @brief Publish an initial pose message to localize the robot in new map
     * @param wormhole_pos Position at which to localize the robot
     */
    void localizeRobot(const geometry_msgs::PoseStamped &wormhole_pos);

    /**
     * @brief Send a goal pose to move_base and wait for result
     * @param goal_pose Target goal pose
     * @return True if goal reached, False otherwise
     */
    bool sendGoalPose(const geometry_msgs::PoseStamped &goal_pose);
};

#endif // MULTI_MAP_GOAL_SERVER_HPP
