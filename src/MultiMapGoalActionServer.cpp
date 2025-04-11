#include "multimap_nav/MultiMapGoalActionServer.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "multi_map_goal_server");

    // Instantiate the action server with the topic name
    MultiMapGoalActionServer server("send_multi_map_nav_goal");

    // Keep spinning to handle callbacks
    ros::spin();
    return 0;
}

// Constructor
MultiMapGoalActionServer::MultiMapGoalActionServer(const std::string& name)
    : nh_(),
      action_server_(nh_, name, boost::bind(&MultiMapGoalActionServer::executeCB, this, _1), false),
      action_name_(name)
{
    ros::NodeHandle pnh("~");

    // Load parameters with fallback defaults
    pnh.param<std::string>("wormhole_db", wormhole_db_path_, ros::package::getPath("multimap_nav") + "/data/wormholes.db");
    pnh.param<std::string>("maps_path", maps_path_, ros::package::getPath("anscer_navigation") + "/maps");
    pnh.param<std::string>("change_map_service", change_map_service_, "change_map");
    pnh.param<std::string>("clear_costmaps_service", clear_costmaps_service_, "/move_base_node/clear_costmaps");
    pnh.param<std::string>("amcl_update_service", amcl_update_service_, "/request_nomotion_update");
    pnh.param<std::string>("initial_pose_topic", initial_pose_topic_, "/initialpose");
    pnh.param<std::string>("frame_id", frame_id_, "map");
    pnh.param<std::string>("current_map", current_map_, "room1");

    // Start the action server
    action_server_.start();
    ROS_INFO("Action server [%s] started.", action_name_.c_str());
}

MultiMapGoalActionServer::~MultiMapGoalActionServer() {}

// Called when an action goal is received
void MultiMapGoalActionServer::executeCB(const multimap_nav::SendGoalGoalConstPtr& goal)
{
    multimap_nav::SendGoalFeedback feedback;
    multimap_nav::SendGoalResult result;

    if (current_map_ == goal->map_name) {
        // Same map, directly go to the goal
        feedback.status_msg = "Robot is in the same map...";
        action_server_.publishFeedback(feedback);

        if (!isPoseWithinMapBounds(goal->goal_pose.pose)) {
            ROS_ERROR("Goal pose is outside the bounds of the specified map: %s", goal->map_name.c_str());
            result.success = false;
            action_server_.setAborted(result, "Goal pose is outside the target map boundaries.");
            return;
        }        

        feedback.status_msg = "Navigating to goal...";
        action_server_.publishFeedback(feedback);

        sendGoalPose(goal->goal_pose);

        result.success = true;
        action_server_.setSucceeded(result);
    } else {
        // Different map, need to use wormhole
        feedback.status_msg = "Received goal location in different map.";
        action_server_.publishFeedback(feedback);

        auto wormhole_pose_opt = getWormholePose(wormhole_db_path_, current_map_, goal->map_name);

        if (!wormhole_pose_opt.has_value()) {
            // Wormhole not found, abort
            feedback.status_msg = "No wormhole found to target map. Aborting navigation.";
            action_server_.publishFeedback(feedback);
            ROS_WARN("Aborting navigation: No wormhole path from '%s' to '%s'", current_map_.c_str(), goal->map_name.c_str());
            result.success = false;
            action_server_.setAborted(result, "No valid wormhole found.");
            return;
        }

        // Wormhole pose retrieved successfully
        geometry_msgs::PoseStamped wormhole_pose = wormhole_pose_opt.value();
        ROS_INFO("Navigating to wormhole at [%.2f, %.2f]", wormhole_pose.pose.position.x, wormhole_pose.pose.position.y);

        feedback.status_msg = "Navigating to wormhole...";
        action_server_.publishFeedback(feedback);
        sendGoalPose(wormhole_pose);

        feedback.status_msg = "Reached the wormhole point...";
        action_server_.publishFeedback(feedback);

        feedback.status_msg = "Switching to new map...";
        action_server_.publishFeedback(feedback);
        switchToNewMap(goal->map_name);

        feedback.status_msg = "Localizing the robot in new map...";
        action_server_.publishFeedback(feedback);
        localizeRobot(wormhole_pose);

        if (!isPoseWithinMapBounds(goal->goal_pose.pose)) {
            ROS_ERROR("Goal pose is outside the bounds of the specified map: %s", goal->map_name.c_str());
            result.success = false;
            action_server_.setAborted(result, "Goal pose is outside the target map boundaries.");
            return;
        }        

        feedback.status_msg = "Navigating to the goal location...";
        action_server_.publishFeedback(feedback);
        bool success = sendGoalPose(goal->goal_pose);

        // Final result
        if (success) {
            result.success = true;
            action_server_.setSucceeded(result);
        } else {
            result.success = false;
            action_server_.setAborted(result);
        }
    }
}

bool MultiMapGoalActionServer::isPoseWithinMapBounds(const geometry_msgs::Pose& pose) {

    nav_msgs::MapMetaData meta_data;

    // Wait for one message from /map_metadata
    auto msg = ros::topic::waitForMessage<nav_msgs::MapMetaData>("/map_metadata", ros::Duration(3.0));
    if (msg == nullptr) {
        ROS_ERROR("Failed to receive map metadata from /map_metadata");
    }

    meta_data = *msg;

    double origin_x = meta_data.origin.position.x;
    double origin_y = meta_data.origin.position.y;
    double resolution = meta_data.resolution;
    int width = meta_data.width;
    int height = meta_data.height;

    MapBounds bounds;
    bounds.min_x = origin_x;
    bounds.min_y = origin_y;
    bounds.max_x = origin_x + width * resolution;
    bounds.max_y = origin_y + height * resolution;

    return (pose.position.x >= bounds.min_x && pose.position.x <= bounds.max_x &&
            pose.position.y >= bounds.min_y && pose.position.y <= bounds.max_y);
}


// Retrieve the wormhole pose from SQLite DB
std::optional<geometry_msgs::PoseStamped> MultiMapGoalActionServer::getWormholePose(
    const std::string& db_path,
    const std::string& current_map,
    const std::string& target_map)
{
    sqlite3* db;
    if (sqlite3_open(db_path.c_str(), &db) != SQLITE_OK) {
        ROS_ERROR("Failed to open wormhole database at: %s", db_path.c_str());
        return std::nullopt;
    }

    // Query to get shared wormhole pose from current map
    std::string query = R"SQL(
        SELECT rel.pos_x, rel.pos_y, rel.pos_z, rel.ori_x, rel.ori_y, rel.ori_z, rel.ori_w
        FROM map_wormhole_relations rel
        JOIN wormholes w ON rel.wormhole_id = w.id
        WHERE rel.map_id = (SELECT id FROM maps WHERE name = ?)
          AND rel.wormhole_id IN (
              SELECT wh1.wormhole_id
              FROM map_wormhole_relations wh1
              JOIN map_wormhole_relations wh2 ON wh1.wormhole_id = wh2.wormhole_id
              WHERE wh1.map_id = (SELECT id FROM maps WHERE name = ?)
                AND wh2.map_id = (SELECT id FROM maps WHERE name = ?)
          )
        LIMIT 1;
    )SQL";

    sqlite3_stmt* stmt;
    if (sqlite3_prepare_v2(db, query.c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
        ROS_ERROR("Failed to prepare SQL statement: %s", sqlite3_errmsg(db));
        sqlite3_close(db);
        return std::nullopt;
    }

    // Bind map names to query
    sqlite3_bind_text(stmt, 1, current_map.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 2, current_map.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 3, target_map.c_str(), -1, SQLITE_TRANSIENT);

    if (sqlite3_step(stmt) == SQLITE_ROW) {
        // Extract pose from DB row
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.frame_id = frame_id_;
        pose_msg.header.stamp = ros::Time::now();

        pose_msg.pose.position.x    = sqlite3_column_double(stmt, 0);
        pose_msg.pose.position.y    = sqlite3_column_double(stmt, 1);
        pose_msg.pose.position.z    = sqlite3_column_double(stmt, 2);
        pose_msg.pose.orientation.x = sqlite3_column_double(stmt, 3);
        pose_msg.pose.orientation.y = sqlite3_column_double(stmt, 4);
        pose_msg.pose.orientation.z = sqlite3_column_double(stmt, 5);
        pose_msg.pose.orientation.w = sqlite3_column_double(stmt, 6);

        sqlite3_finalize(stmt);
        sqlite3_close(db);
        return pose_msg;
    }

    // No valid wormhole found
    ROS_WARN("No wormhole found from '%s' to '%s'", current_map.c_str(), target_map.c_str());
    sqlite3_finalize(stmt);
    sqlite3_close(db);
    return std::nullopt;
}

// Trigger service to switch maps
bool MultiMapGoalActionServer::switchToNewMap(const std::string& map_name)
{
    ros::ServiceClient client = nh_.serviceClient<nav_msgs::LoadMap>(change_map_service_);
    nav_msgs::LoadMap srv;

    // Build map URL from path + map name
    srv.request.map_url = maps_path_ + "/" + map_name + ".yaml";

    if (client.call(srv)) {
        current_map_ = map_name;
        ROS_INFO("Switched to map: %s", map_name.c_str());
        return true;
    } else {
        ROS_ERROR("Failed to switch to map: %s", map_name.c_str());
        return false;
    }
}

// Publish initial pose after map switch to localize robot
void MultiMapGoalActionServer::localizeRobot(const geometry_msgs::PoseStamped& wormhole_pos)
{
    ros::Publisher pub = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(initial_pose_topic_, 1, true);

    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.header = wormhole_pos.header;
    msg.pose.pose.position = wormhole_pos.pose.position;
    msg.pose.pose.orientation = wormhole_pos.pose.orientation;

    // Some basic 6D covariance
    double covariance[36] = {
        0.25, 0, 0, 0, 0, 0,
        0, 0.25, 0, 0, 0, 0,
        0, 0, 0.25, 0, 0, 0,
        0, 0, 0, 0.25, 0, 0,
        0, 0, 0, 0, 0.25, 0,
        0, 0, 0, 0, 0, 0.0685389
    };
    for (int i = 0; i < 36; ++i)
        msg.pose.covariance[i] = covariance[i];

    std_srvs::Empty srv;
    ros::Time start = ros::Time::now();

    // Publish pose and request AMCL update for a few seconds
    while (ros::Time::now() - start < ros::Duration(4.0)) {
        ros::service::call(clear_costmaps_service_, srv);
        ros::service::call(amcl_update_service_, srv);
        pub.publish(msg);
        ros::Duration(0.5).sleep();
    }

    ROS_INFO("Published initial pose for localization");
}

// Send a nav goal to move_base
bool MultiMapGoalActionServer::sendGoalPose(const geometry_msgs::PoseStamped& goal_pose)
{
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client("move_base", true);

    // Wait for move_base to be ready
    if (!client.waitForServer(ros::Duration(5.0))) {
        ROS_ERROR("move_base action server not available");
        return false;
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = goal_pose;

    ROS_INFO("Sending goal to move_base...");
    client.sendGoal(goal);
    client.waitForResult();

    if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Goal reached!");
        return true;
    } else {
        ROS_ERROR("Failed to reach goal.");
        return false;
    }
}
