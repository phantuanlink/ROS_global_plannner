#include "rrt_planner/rrt_planner.h"

namespace rrt_planner
{

RRTPlanner::RRTPlanner(ros::NodeHandle * node)
: nh_(node),
  private_nh_("~"),
  map_received_(false),
  init_pose_received_(false),
  goal_received_(false)
{
  // Get map and path topics from parameter server
  std::string map_topic, path_topic;
  private_nh_.param<std::string>("map_topic", map_topic, "/map");
  private_nh_.param<std::string>("path_topic", path_topic, "/path");

  // Subscribe to map topic
  map_sub_ = nh_->subscribe<const nav_msgs::OccupancyGrid::Ptr &>(
    map_topic, 1, &RRTPlanner::mapCallback, this);

  // Subscribe to initial pose topic that is published by RViz
  init_pose_sub_ = nh_->subscribe<const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &>(
    "/initialpose", 1, &RRTPlanner::initPoseCallback, this);

  // Subscribe to goal topic that is published by RViz
  goal_sub_ = nh_->subscribe<const geometry_msgs::PoseStamped::ConstPtr &>(
    "/move_base_simple/goal", 1, &RRTPlanner::goalCallback, this);

  // Advertise topic where calculated path is going to be published
  path_pub_ = nh_->advertise<nav_msgs::Path>(path_topic, 1, true);

  // This loops until the node is running, will exit when the node is killed
  while (ros::ok()) {
    // if map, initial pose, and goal have been received
    // build the map image, draw initial pose and goal, and plan
    if (map_received_ && init_pose_received_ && goal_received_) {
      buildMapImage();
      drawGoalInitPose();
      plan();
    } else {
      if (map_received_) {
        displayMapImage();
      }
      ros::Duration(0.1).sleep();
      ros::spinOnce();
    }
  }
}

void RRTPlanner::mapCallback(const nav_msgs::OccupancyGrid::Ptr & msg)
{
  map_grid_ = msg;

  // Build and display the map image
  buildMapImage();
  displayMapImage();

  // Reset these values for a new planning iteration
  map_received_ = true;
  init_pose_received_ = false;
  goal_received_ = false;

  ROS_INFO("Map obtained successfully. Please provide initial pose and goal through RViz.");
}

void RRTPlanner::initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & msg)
{
  if (init_pose_received_) {
    buildMapImage();
  }

  // Convert mas to Point2D
  poseToPoint(init_pose_, msg->pose.pose);

  // Reject the initial pose if the given point is occupied in the map
  if (!isPointUnoccupied(init_pose_)) {
    init_pose_received_ = false;
    ROS_WARN(
      "The initial pose specified is on or too close to an obstacle please specify another point");
  } else {
    init_pose_received_ = true;
    drawGoalInitPose();
    ROS_INFO("Initial pose obtained successfully.");
  }

  displayMapImage();
}

void RRTPlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr & msg)
{
  if (goal_received_) {
    buildMapImage();
  }

  // Convert msg to Point2D
  poseToPoint(goal_, msg->pose);

  // Reject the goal pose if the given point is occupied in the map
  if (!isPointUnoccupied(goal_)) {
    goal_received_ = false;
    ROS_WARN("The goal specified is on or too close to an obstacle please specify another point");
  } else {
    goal_received_ = true;
    drawGoalInitPose();
    ROS_INFO("Goal obtained successfully.");
  }

  displayMapImage();
}

void RRTPlanner::drawGoalInitPose()
{
  if (goal_received_) {
    drawCircle(goal_, 3, cv::Scalar(12, 255, 43));
  }
  if (init_pose_received_) {
    drawCircle(init_pose_, 3, cv::Scalar(255, 200, 0));
  }
}

void RRTPlanner::plan()
{
  // Reset these values so planning only happens once for a
  // given pair of initial pose and goal points
  goal_received_ = false;
  init_pose_received_ = false;

  // TODO: Fill out this function with the RRT algorithm logic to plan a collision-free
  //       path through the map starting from the initial pose and ending at the goal pose
  int height_ = map_->rows;
  int width_ = map_->cols;
  bool path_found = false;
  Node start_node(init_pose_, -1); //Make the start node with ID = 0 and parent ID = -1, means no parents
  rrtree.push_back(start_node);

  int count_samples = 1; //Keep track the number of created samples

  Point2D rand_point;
  int nearest_node_idx;
  Point2D new_point;  

  //Now lets make the tree
  while (rrtree.size() < num_samples)
  {
    do
    {
      rand_point = randomPointGenerator(height_, width_);
      nearest_node_idx = findNearestNode(rand_point);
      new_point = findNew(rand_point, rrtree.at(nearest_node_idx).point_);
    } while (notCollision(rrtree.at(nearest_node_idx).point_, new_point));
    
    Node new_node(new_point, nearest_node_idx); //Make a new node with the point created and its parentID
    rrtree.push_back(new_node);

    //Now check whether it is reached by the goal
    if ((distance(new_point, goal_) <= k) && (notCollision(new_point, goal_)))
    {
      ROS_INFO("YAYYYYYY!! THE GOAL IS REACHED!!! NICE");
      ROS_INFO("SOLUTION IS FOUND AFTER %d ITERATIONS", rrtree.size());
      path_found = true;
      Node goal_node(goal_, rrtree.size()-1); //the goal node has the last node as its the parent node
      rrtree.push_back(goal_node);
      break;
    }
  }

  if (path_found)
  {
    std::vector<Point2D> path = backTracking();
    ROS_INFO("PUBLISHING THE PATH RIGHT NOW!! DO YOU SEE IT IN THE MAP");
    publishPath(path);
  } else {
    ROS_INFO("THERE IS NO SOLUTION THO!!! try with more samples and smaller step size please!!");
  }
}

Point2D RRTPlanner::randomPointGenerator(int height_, int width_)
{
  std::default_random_engine generator;
  std::uniform_int_distribution<int> dis_height(0, height_);
  std::uniform_int_distribution<int> dis_width(0, width_);

  Point2D rand_pt;

  do
  {
    int x = dis_width(generator);
    int y = dis_height(generator);
    rand_pt.x(x);
    rand_pt.y(y);
  } while (isPointUnoccupied(rand_pt));
  
  return rand_pt;
}

int RRTPlanner::findNearestNode(Point2D rand_point)
{
  int index = 0;

  double closet_dis = distance(rand_point, rrtree.front().point_);
  for (unsigned int i = 1; i < rrtree.size(); i++)
  {
    if (distance(rand_point, rrtree[i].point_) < closet_dis)
    {
      index = i; //get the closet node index
    }
  }
  
  return index;
}

Point2D RRTPlanner::findNew(Point2D rand_point, Point2D nearest_point)
{
  Point2D new_point;
  double length = k;
  double theta = atan((rand_point.y()- nearest_point.y())/(rand_point.x()- nearest_point.x()));
  if (distance(rand_point, nearest_point) < k)
  {
    length = k;
  }
  new_point.x(nearest_point.x() + length*cos(theta));
  new_point.y(nearest_point.y() + length*sin(theta));

  return new_point;
}


bool RRTPlanner::notCollision(Point2D a, Point2D b)
{
  Point2D mid;
  int del_x = b.x() - a.x();
  int del_y = b.y() - a.y();

  int i = 1;
  int num = 5; //the number of middle points
  while (i <= num)
  {
    mid.x(a.x() + del_x * i/(num+1));
    mid.y(a.y() + del_y * i/(num+1));
    if (!isPointUnoccupied(mid))
    {
      return false;
    }
  }
  return true;
}

double RRTPlanner::distance(Point2D a, Point2D b)
{
  //TO DO
  double del_x = (double)a.x() - (double)b.x();
  double del_y = (double)a.y() - (double)b.y();

  return sqrt(pow(del_x, 2) + pow(del_y, 2));
}

std::vector<Point2D> RRTPlanner::backTracking()
{
  std::vector<Point2D> path;
  Node* cur_node;
  
  cur_node = &(rrtree.back());

  do
  {
    path.push_back(cur_node->point_);

    // draw the line to visualize the global path
    drawLine(cur_node->point_, rrtree.at(cur_node->parentID_).point_, cv::Scalar(12, 255, 43), 3);
    
    cur_node = &(rrtree.at(cur_node->parentID_));
 
  } while (cur_node->parentID_ != -1);

  return path;
}


void RRTPlanner::publishPath(const std::vector<Point2D>& path)
{
  // Create new Path msg
  nav_msgs::Path path_msg;
  path_msg.header.frame_id = map_grid_->header.frame_id;
  path_msg.header.stamp = ros::Time::now();

  // TODO: Fill nav_msgs::Path msg with the path calculated by RRT

  path_msg.poses.resize(path.size());
  for (unsigned int i = 0; i < path.size(); i++)
  {
    path_msg.poses[i] = pointToPose(path[i]);
  }

  // Publish the calculated path
  path_pub_.publish(path_msg);

  displayMapImage();
}

bool RRTPlanner::isPointUnoccupied(const Point2D & p)
{
  int x = p.x();
  int y = p.y();

  // TODO: Fill out this function to check if a given point is occupied/free in the map
  if (map_->at<cv::Vec3b>(x, y) == cv::Vec3b(0, 0, 0))
  {
    return false;
  }
  else 
  {
    return true;
  }
}

void RRTPlanner::buildMapImage()
{
  // Create a new opencv matrix with the same height and width as the received map
  map_ = std::unique_ptr<cv::Mat>(new cv::Mat(map_grid_->info.height,
                                              map_grid_->info.width,
                                              CV_8UC3,
                                              cv::Scalar::all(255)));

  // Fill the opencv matrix pixels with the map points
  for (int i = 0; i < map_grid_->info.height; i++) {
    for (int j = 0; j < map_grid_->info.width; j++) {
      if (map_grid_->data[toIndex(i, j)]) {
        map_->at<cv::Vec3b>(map_grid_->info.height - i - 1, j) = cv::Vec3b(0, 0, 0);
      } else {
        map_->at<cv::Vec3b>(map_grid_->info.height - i - 1, j) = cv::Vec3b(255, 255, 255);
      }
    }
  }
}

void RRTPlanner::displayMapImage(int delay)
{
  cv::imshow("Output", *map_);
  cv::waitKey(delay);
}

void RRTPlanner::drawCircle(Point2D & p, int radius, const cv::Scalar & color)
{
  cv::circle(
    *map_,
    cv::Point(p.y(), map_grid_->info.height - p.x() - 1),
    radius,
    color,
    -1);
}

void RRTPlanner::drawLine(Point2D & p1, Point2D & p2, const cv::Scalar & color, int thickness)
{
  cv::line(
    *map_,
    cv::Point(p2.y(), map_grid_->info.height - p2.x() - 1),
    cv::Point(p1.y(), map_grid_->info.height - p1.x() - 1),
    color,
    thickness);
}

inline geometry_msgs::PoseStamped RRTPlanner::pointToPose(const Point2D & p)
{
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = p.y() * map_grid_->info.resolution;
  pose.pose.position.y = p.x() * map_grid_->info.resolution;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = map_grid_->header.frame_id;
  return pose;
}

inline void RRTPlanner::poseToPoint(Point2D & p, const geometry_msgs::Pose & pose)
{
  p.x(pose.position.y / map_grid_->info.resolution);
  p.y(pose.position.x / map_grid_->info.resolution);
}

inline int RRTPlanner::toIndex(int x, int y)
{
  return x * map_grid_->info.width + y;
}

}  // namespace rrt_planner
