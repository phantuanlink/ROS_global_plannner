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

  // Get Parameters HAHAHAH, I totally forgot about this
  private_nh_.param<int>("max_samples_number", num_samples, 500);
  private_nh_.param<double>("max_step", k, 25.0);
  private_nh_.param<int>("goal_bias", goal_bias, 0); // means no goal bias

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

  // Reject the initialy7llll pose if the given point is occupied in the map
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

  bool informed = true;
  if (goal_bias == 0)
  {
    informed = false;
  }
  
  int height_ = map_->rows;
  int width_ = map_->cols;

  std::cout << "The size of the map is" << std::endl;
  std::cout << height_ << "   "  << width_ << std::endl;

  bool path_found = false;
  Node start_node(init_pose_, -1); //Make the start node with ID = 0 and parent ID = -1, means no parents
  rrtree.push_back(start_node);

  int count_samples = 1; //Keep track the number of created samples

  Point2D rand_point;
  int nearest_node_idx;
  Point2D nearest_point;
  Point2D new_point; 
  int count_iterations = 0; 

  std::uniform_int_distribution<int> dis_height(0, height_);
  std::uniform_int_distribution<int> dis_width(0, width_);

  //Now lets make the tree
  while (count_iterations < num_samples)//num_samples
  {
    if (count_iterations%10 == 0)
    {
      ROS_INFO("Interation number %d", rrtree.size());
    }

    do
    {
      ROS_INFO("New iteration");
      ROS_INFO("STEP1: geneate random point");
      //rand_point = randomPointGenerator(dis_height, dis_width);
      int rand_point_x = dis_width(generator);
      int rand_point_y = dis_height(generator);
      rand_point.x(rand_point_x);
      rand_point.y(rand_point_y);
      // if (informed && ((rrtree.size()%goal_bias) == 0))
      // {
      //   rand_point.x(goal_.x());   // Take the goal as a random point to get to the goal faster
      //   rand_point.y(goal_.y());
      // } 
      std::cout << rand_point.x() << "  " << rand_point.y() << std::endl;

      ROS_INFO("STEP2: Find the nearest node index");
      nearest_node_idx = findNearestNode(rand_point);
      std::cout << "The nearest Node index is " << nearest_node_idx << std::endl;
      nearest_point = rrtree[nearest_node_idx].point_;
      std::cout << nearest_point.x() << "   " << nearest_point.y() << std::endl;

      ROS_INFO("STEP3: Find new point");
      new_point = findNew(rand_point, nearest_point);
      std::cout << "New Points "<< std::endl;
      std::cout << new_point.x() << "   " << new_point.y()  << std::endl;
      std::cout << distance(new_point, nearest_point)  << std::endl;

      ROS_INFO("STEP4: Check the edge");

    } while (!isCollisionFreeLine(nearest_point, new_point));
    
    
    Node new_node(new_point, nearest_node_idx); //Make a new node with the point created and its parentID
    ROS_INFO("STEP5: Create the node and visualization");

    drawCircle(new_point, 1, cv::Scalar(12, 255, 43));
    drawLine(new_point, nearest_point,cv::Scalar(0, 0, 0));
    rrtree.push_back(new_node);

    //Now check whether it is reached by the goal
    if ((distance(new_point, goal_) <= k) && (isCollisionFreeLine(new_point, goal_)))
    {
      ROS_INFO("YAYYYYYY!! THE GOAL IS REACHED!!! NICE");
      ROS_INFO("SOLUTION IS FOUND AFTER %d ITERATIONS", rrtree.size());
      path_found = true;
      Node goal_node(goal_, rrtree.size()-1); //the goal node has the last node as its the parent node
      rrtree.push_back(goal_node);
      break;
    }

    count_iterations++;
  }

  if (path_found)
  {
    std::vector<Point2D> path = backTracking();
    ROS_INFO("PUBLISHING THE PATH RIGHT NOW!! DO YOU SEE IT IN THE MAP");
    publishPath(path);
  } else {
    ROS_INFO("THERE IS NO SOLUTION THO!!! try with more samples and smaller step size please!!");
    std::cout << rrtree.size() << std::endl;
  }
}

// Point2D RRTPlanner::randomPointGenerator(const std::uniform_int_distribution<int>& dis_height_, const std::uniform_int_distribution<int>& dis_width)
// {
//   Point2D rand_pt;

//   do
//   {
//     int x = dis_width(generator);
//     int y = dis_height(generator);
//     // int x = rand()%width_;
//     // int y = rand()%height_;
//     rand_pt.x(x);
//     rand_pt.y(y);
//     //std::cout << "point is free or not " << isFree << std::endl;
//   } while (!isPointUnoccupied(rand_pt));
  
//   return rand_pt;
// }

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
  std::cout << "Max_length " << k << std::endl;
  double length = k;
  double theta = atan2((double)(rand_point.y()- nearest_point.y()),(double)(rand_point.x()- nearest_point.x()));
  std::cout << theta << std::endl;
  if (distance(rand_point, nearest_point) < k)
  {
    length = k;
  }
  new_point.x(nearest_point.x() + length*cos(theta));
  new_point.y(nearest_point.y() + length*sin(theta));

  // std::cout << "New Points "<< std::endl;
  // std::cout << new_point.x() << "   " << new_point.y()  << std::endl;
  //drawCircle(new_point, 2,cv::Scalar(12, 255, 43));

  return new_point;
}


bool RRTPlanner::isCollisionFreeLine(Point2D a, Point2D b)
{
  Point2D mid;
  int del_x = b.x() - a.x();
  int del_y = b.y() - a.y();

  int i = 1;
  int num = (int)k/2; //the number of middle points (including the new point itself)
  while (i <= num)
  {
    mid.x(a.x() + del_x * i/(num));
    mid.y(a.y() + del_y * i/(num));
    if (!isPointUnoccupied(mid))
    {
      return false;
    }
    i++;
  }
  std::cout << "Oh it is not obstructed, yay " << std::endl;
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
  //transform index of points into map_grid_
  
  // TODO: Fill out this function to check if a given point is occupied/free in the map

  if (map_grid_->data[toIndex(p.x(), p.y())])
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
  //cv::Point(p.y(), map_grid_->info.height - p.x() - 1)
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
