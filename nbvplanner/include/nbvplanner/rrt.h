/*
 * Copyright 2015 Andreas Bircher, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef RRTTREE_H_
#define RRTTREE_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <sstream>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <kdtree/kdtree.h>
#include <nbvplanner/tree.h>
#include <nbvplanner/mesh_structure.h>

#define SQ(x) ((x)*(x))
#define SQRT2 0.70711

namespace nbvInspection {

class RrtTree : public TreeBase<Eigen::Vector4d>
{
 public:
  typedef Eigen::Vector4d StateVec;

  RrtTree();
  RrtTree(mesh::StlMesh * mesh, volumetric_mapping::OctomapManager * manager);
  ~RrtTree();
  virtual void setStateFromPoseMsg(const geometry_msgs::PoseWithCovarianceStamped& pose);
  virtual void setStateFromOdometryMsg(const nav_msgs::Odometry& pose);
  virtual void setPeerStateFromPoseMsg(const geometry_msgs::PoseWithCovarianceStamped& pose, int n_peer);
  virtual void iterate(std::vector<Eigen::Vector4d> peer_target);
  virtual void initialize(std::vector<Eigen::Vector4d> peer_target);
  virtual void changeBestNode(std::vector<Node<Eigen::Vector4d> *> candidates, std::vector<Eigen::Vector4d> peer_target); //coordination mode only

  virtual void VRRT_iterate(std::vector<Eigen::Vector4d> peer_target);
  virtual void VRRT_initialize();
  virtual std::vector<geometry_msgs::Pose> VRRT_getBestEdge(std::string targetFrame);
  virtual void VRRT_SmoothPath(std::vector<nbvInspection::Node<StateVec>*>& states);
  virtual std::vector<geometry_msgs::Pose> getBestEdge(std::string targetFrame);

  virtual void clear();
  virtual std::vector<geometry_msgs::Pose> getPathBackToPrevious(std::string targetFrame);
  virtual void memorizeBestBranch();
  void publishNode(Node<StateVec> * node);
  double gain(StateVec state);
  std::vector<geometry_msgs::Pose> samplePath(StateVec start, StateVec end, std::string targetFrame,
                                              std::vector<geometry_msgs::Pose> ret);

  virtual std::vector<tf::Vector3> getPeerPose(int num);
  void setPeerPoseInTree(const geometry_msgs::Pose& pose, int n_peer);
  bool biased_coin(double probability);
  virtual struct kdtree* get_kdtree();

  static bool cmp(Node<StateVec> * a, Node<StateVec> * b);
  void getLeafNode(int dummy);
  virtual std::vector<Node<Eigen::Vector4d> *> getCandidates();
  virtual Eigen::Vector4d getRoot();
  virtual Eigen::Vector4d getBest();

protected:
  kdtree * kdTree_;
  std::stack<StateVec> history_;
  std::vector<StateVec> bestBranchMemory_;
  int g_ID_;
  int iterationCount_;
  std::fstream fileTree_;
  std::fstream filePath_;
  std::fstream fileResponse_;
  std::string logFilePath_;
  std::vector<double> inspectionThrottleTime_;

private:
  static std::vector<tf::Vector3> peer_vehicles_;
};
}

#endif
