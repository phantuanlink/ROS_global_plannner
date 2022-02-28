#ifndef RRT_PLANNER_INCLUDE_RRT_PLANNER_RRT_PLANNER_H_
#define RRT_PLANNER_INCLUDE_RRT_PLANNER_RRT_PLANNER_H_

#include <random>
#include <iostream>
#include <vector>
#include <cmath>
#include <stdlib.h>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <opencv2/opencv.hpp>

namespace rrt_planner
{

/**
 * A utility class to represent a 2D point
 */
class Point2D
{
public:
  Point2D(): x_(0), y_(0) {}
  Point2D(int x, int y): x_(x), y_(y) {}

  int x() const
  {
    return x_;
  }

  int y() const
  {
    return y_;
  }

  void x(int x)
  {
    x_ = x;
  }

  void y(int y)
  {
    y_ = y;
  }

private:
  int x_;
  int y_;
};

/**
 * Node class for the RRT tree, A node has info about the point, its ID and its parent node ID
 */
class Node
{
public:
  Node() {};
  
  Point2D point_;
  int parentID_;

  Node(Point2D point, int parentID)
  {
    point_ = point;
    parentID_ = parentID;
  }

  Node(Point2D point)
  {
    point_ = point;
    parentID_ = -1;
  }
  
};

/**
 * Main class which implements the RRT algorithm
 */
class RRTPlanner
{
public:
  explicit RRTPlanner(ros::NodeHandle *);

  ~RRTPlanner() = default;

  /**
   * Given a map, the initial pose, and the goal, this function will plan
   * a collision-free path through the map from the initial pose to the goal
   * using the RRT algorithm
   *
   * THE CANDIDATE IS REQUIRED TO IMPLEMENT THE LOGIC IN THIS FUNCTION
   */
  void plan();

  /**
   * Callback for map subscriber
   */
  void mapCallback(const nav_msgs::OccupancyGrid::Ptr &);

  /**
   * Callback for initial pose subscriber
   */
  void initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &);

  /**
   * Callback for goal subscriber
   */
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &);

private:

  /**
   * Publishes the path calculated by RRT as a nav_msgs::Path msg
   *
   */
  void publishPath(const std::vector<Point2D>& path);

  /**
   * Utility function to check if a given point is free/occupied in the map
   * @param p: point in the map
   * @return boolean true if point is unoccupied, false if occupied
   *
   * THE CANDIDATE IS REQUIRED TO IMPLEMENT THE LOGIC IN THIS FUNCTION
   */
  bool isPointUnoccupied(const Point2D & p);

  /**
   * Utility function to build a CV::Mat from a nav_msgs::OccupancyGrid for display
   */
  void buildMapImage();

  /**
   * Utility function to display the CV::Mat map image
   * @param delay
   */
  void displayMapImage(int delay = 1);

  /**
   * Utility function to draw initial pose and goal pose on the map image
   */
  void drawGoalInitPose();

  /**
   * Utility function to draw a circle on the map
   * @param p: center point of the circle
   * @param radius: radius of the circle
   * @param color: color of the circle
   */
  void drawCircle(Point2D & p, int radius, const cv::Scalar & color);

  /**
   * Utility function to draw a line on the map
   * @param p1: starting point of the line
   * @param p2: end point of the line
   * @param color: color of the line
   * @param thickness: thickness of the line
   */
  void drawLine(Point2D & p1, Point2D & p2, const cv::Scalar & color, int thickness = 1);

  /**
   * Utility function to convert a Point2D object to a geometry_msgs::PoseStamped object
   * @return corresponding geometry_msgs::PoseStamped object
   */
  inline geometry_msgs::PoseStamped pointToPose(const Point2D &);

  /**
   * Utility function to convert a geometry_msgs::PoseStamped object to a Point2D object
   */
  inline void poseToPoint(Point2D &, const geometry_msgs::Pose &);

  /**
   * Utility function to convert (x, y) matrix coordinate to corresponding vector coordinate
   */
  inline int toIndex(int, int);

  ////////////////////// ADDED FUNCTIONS HERE ///////////////////
  
  /**
   * Calculate distance between two points
   */
  double distance(Point2D a, Point2D b);

  /**
   * Generate points in C_free
   */
  Point2D randomPointGenerator(int height_, int width_);

  /**
   * Find the index of the nearest Node to the randon point
   */
  int findNearestNode(Point2D rand_point);

  /**
   * Return the coordinates of the points based on rand_poind and closet point
   */
  Point2D findNew(Point2D rand_point, Point2D nearest_point);

  /**
   * Return whether the line between two points is obstructed by obstacles
   * Checking around 4 points in the middle
   */
  bool isCollisionFreeLine(Point2D a, Point2D b);

  /**
   * Return the vector of all the nodes in the rrtree leading to the goal
   */
  std::vector<Point2D> backTracking();

  

  ros::NodeHandle * nh_;
  ros::NodeHandle private_nh_;

  bool map_received_;
  std::unique_ptr<cv::Mat> map_;
  nav_msgs::OccupancyGrid::Ptr map_grid_;

  bool init_pose_received_;
  Point2D init_pose_;

  bool goal_received_;
  Point2D goal_;

  ros::Subscriber map_sub_;
  ros::Subscriber init_pose_sub_;
  ros::Subscriber goal_sub_;
  ros::Publisher path_pub_;

  //Some Parameters that user define, gonna use dynamic reconfiguration to change paratmeters on the spot
  int num_samples; // total number of samples
  double k; // the max length of an edge
  int goal_bias; // the frequency considering goal point as one of the random points

  std::vector<Node> rrtree; 

  std::default_random_engine generator; 

};

}

#endif  // RRT_PLANNER_INCLUDE_RRT_PLANNER_RRT_PLANNER_H_
