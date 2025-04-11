#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <multimap_nav/SendGoalAction.h>
#include <tf/tf.h>

// Typedef for cleaner code (SimpleActionClient to handle SendGoalAction)
typedef actionlib::SimpleActionClient<multimap_nav::SendGoalAction> Client;

// Struct to neatly pack goal input
struct GoalInput
{
    double x;
    double y;
    double yaw;
    std::string map_name;
};

// Parse CLI input and validate it
GoalInput getInput(int argc, char **argv)
{
    if (argc < 5)
    {
        // If not enough args, shout and exit
        ROS_ERROR("Usage: multi_map_goal_client <x> <y> <yaw> <map_name>");
        ros::shutdown();
        exit(1);
    }

    GoalInput input;
    input.x = std::atof(argv[1]);     // Parse x position
    input.y = std::atof(argv[2]);     // Parse y position
    input.yaw = std::atof(argv[3]);   // Parse yaw (rotation in radians)
    input.map_name = argv[4];         // Target map name

    return input;
}

// Called when goal finishes (either succeeded, aborted, etc.)
void doneCallback(const actionlib::SimpleClientGoalState &state,
                  const multimap_nav::SendGoalResultConstPtr &result)
{
    ROS_INFO("Goal finished with state: %s", state.toString().c_str());
    if (result->success)
    {
        ROS_INFO("Goal reached successfully!");
    }
    else
    {
        ROS_ERROR("Failed to reach the goal.");
    }
}

// Called once the server starts processing the goal
void activeCallback()
{
    ROS_INFO("Goal is now active.");
}

// Called whenever feedback is received from the server
void feedbackCallback(const multimap_nav::SendGoalFeedbackConstPtr &feedback)
{
    ROS_INFO("Feedback: %s", feedback->status_msg.c_str());
}

// Main function to build and send the goal
void sendGoal(double x_pos, double y_pos, double yaw, const std::string &map_name)
{
    // Create the client for the action server (static so it doesn't get destroyed)
    static Client client("send_multi_map_nav_goal", true);

    // Wait for the action server to spin up
    ROS_INFO("Waiting for action server to start...");
    client.waitForServer();
    ROS_INFO("Action server started.");

    // Build the goal message
    multimap_nav::SendGoalGoal goal;

    goal.goal_pose.header.frame_id = "map";                  // Set frame of reference
    goal.goal_pose.header.stamp = ros::Time::now();          // Stamp it with current time

    goal.goal_pose.pose.position.x = x_pos;
    goal.goal_pose.pose.position.y = y_pos;
    goal.goal_pose.pose.position.z = 0.0;                     // 2D map so z = 0
    goal.goal_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);  // Convert yaw to quaternion

    goal.map_name = map_name;                                // Set the target map name

    // Send the goal with all three callbacks
    client.sendGoal(goal, &doneCallback, &activeCallback, &feedbackCallback);

    // Wait for result or timeout in 60s
    bool finished_before_timeout = client.waitForResult(ros::Duration(60.0));
    if (!finished_before_timeout)
    {
        ROS_ERROR("Action did not finish before the timeout.");
    }
}

// Standard main to run the goal client
int main(int argc, char **argv)
{
    ros::init(argc, argv, "multi_map_goal_client");  // Initialize ROS node

    GoalInput input = getInput(argc, argv);          // Grab input from CLI
    sendGoal(input.x, input.y, input.yaw, input.map_name);  // Send the goal to the action server

    return 0;
}
