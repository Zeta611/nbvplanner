#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <nbvplanner/nbvp.hpp>
#include <fstream>
#include <iostream>
#include <ctime>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nbvPlanner");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  nbvInspection::nbvPlanner<Eigen::Matrix<double, 4, 1> > planner(nh, nh_private);
  time_t start_time;
  struct tm * tmp;
  start_time = time(NULL);
  std::ofstream outFile("Time.txt");
  outFile << start_time << std::endl;
  outFile.close();
  ros::spin();
  return 0;
};