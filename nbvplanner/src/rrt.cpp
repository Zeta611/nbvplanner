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

#ifndef RRTTREE_HPP_
#define RRTTREE_HPP_

#include <cstdlib>
#include <multiagent_collision_check/multiagent_collision_checker.h>
#include <nbvplanner/rrt.h>
#include <nbvplanner/tree.hpp>
#include <nbvplanner/tree.h>
#include <kdtree/kdtree.h>
#include <algorithm>

typedef Eigen::Vector4d StateVec;

nbvInspection::RrtTree::RrtTree()
    : nbvInspection::TreeBase<StateVec>::TreeBase()
{
  kdTree_ = kd_create(4);
  iterationCount_ = 0;
  for (int i = 0; i < 4; i++) {
    inspectionThrottleTime_.push_back(ros::Time::now().toSec());
  }

  // If logging is required, set up files here
  bool ifLog = false;
  std::string ns = ros::this_node::getName();
  ros::param::get(ns + "/nbvp/log/on", ifLog);
  if (ifLog) {
    time_t rawtime;
    struct tm * ptm;
    time(&rawtime);
    ptm = gmtime(&rawtime);
    logFilePath_ = ros::package::getPath("nbvplanner") + "/data/"
        + std::to_string(ptm->tm_year + 1900) + "_" + std::to_string(ptm->tm_mon + 1) + "_"
        + std::to_string(ptm->tm_mday) + "_" + std::to_string(ptm->tm_hour) + "_"
        + std::to_string(ptm->tm_min) + "_" + std::to_string(ptm->tm_sec);
    system(("mkdir -p " + logFilePath_).c_str());
    logFilePath_ += "/";
    fileResponse_.open((logFilePath_ + "response.txt").c_str(), std::ios::out);
    filePath_.open((logFilePath_ + "path.txt").c_str(), std::ios::out);
  }
}

nbvInspection::RrtTree::RrtTree(mesh::StlMesh * mesh, volumetric_mapping::OctomapManager * manager)
{
  mesh_ = mesh;
  manager_ = manager;
  kdTree_ = kd_create(4);
  iterationCount_ = 0;
  for (int i = 0; i < 4; i++) {
    inspectionThrottleTime_.push_back(ros::Time::now().toSec());
  }

  // If logging is required, set up files here
  bool ifLog = false;
  std::string ns = ros::this_node::getName();
  ros::param::get(ns + "/nbvp/log/on", ifLog);
  if (ifLog) {
    time_t rawtime;
    struct tm * ptm;
    time(&rawtime);
    ptm = gmtime(&rawtime);
    logFilePath_ = ros::package::getPath("nbvplanner") + "/data/"
        + std::to_string(ptm->tm_year + 1900) + "_" + std::to_string(ptm->tm_mon + 1) + "_"
        + std::to_string(ptm->tm_mday) + "_" + std::to_string(ptm->tm_hour) + "_"
        + std::to_string(ptm->tm_min) + "_" + std::to_string(ptm->tm_sec);
    system(("mkdir -p " + logFilePath_).c_str());
    logFilePath_ += "/";
    fileResponse_.open((logFilePath_ + "response.txt").c_str(), std::ios::out);
    filePath_.open((logFilePath_ + "path.txt").c_str(), std::ios::out);
  }
}

nbvInspection::RrtTree::~RrtTree()
{
  delete rootNode_;
  kd_free(kdTree_);
  if (fileResponse_.is_open()) {
    fileResponse_.close();
  }
  if (fileTree_.is_open()) {
    fileTree_.close();
  }
  if (filePath_.is_open()) {
    filePath_.close();
  }
}

void nbvInspection::RrtTree::setStateFromPoseMsg(
    const geometry_msgs::PoseWithCovarianceStamped& pose)
{
  // Get latest transform to the planning frame and transform the pose
  static tf::TransformListener listener;
  tf::StampedTransform transform;
  try {
    listener.lookupTransform(params_.navigationFrame_, pose.header.frame_id, pose.header.stamp,
                             transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }
  tf::Pose poseTF;
  tf::poseMsgToTF(pose.pose.pose, poseTF);
  tf::Vector3 position = poseTF.getOrigin();
  position = transform * position;
  tf::Quaternion quat = poseTF.getRotation();
  quat = transform * quat;
  root_[0] = position.x();
  root_[1] = position.y();
  root_[2] = position.z();
  root_[3] = tf::getYaw(quat);

  // Log the vehicle response in the planning frame
  static double logThrottleTime = ros::Time::now().toSec();
  if (ros::Time::now().toSec() - logThrottleTime > params_.log_throttle_) {
    logThrottleTime += params_.log_throttle_;
    if (params_.log_) {
      for (int i = 0; i < root_.size() - 1; i++) {
        fileResponse_ << root_[i] << ",";
      }
      fileResponse_ << root_[root_.size() - 1] << "\n";
    }
  }
  // Update the inspected parts of the mesh using the current position
  if (ros::Time::now().toSec() - inspectionThrottleTime_[0] > params_.inspection_throttle_) {
    inspectionThrottleTime_[0] += params_.inspection_throttle_;
    geometry_msgs::Pose poseTransformed;
    tf::poseTFToMsg(transform * poseTF, poseTransformed);
    setPeerPoseInTree(poseTransformed, 0);
    if (mesh_) {
      geometry_msgs::Pose poseTransformed;
      tf::poseTFToMsg(transform * poseTF, poseTransformed);
      mesh_->setPeerPose(poseTransformed, 0);
      mesh_->incorporateViewFromPoseMsg(poseTransformed, 0);
      // Publish the mesh marker for visualization in rviz
      visualization_msgs::Marker inspected;
      inspected.ns = "meshInspected";
      inspected.id = 0;
      inspected.header.seq = inspected.id;
      inspected.header.stamp = pose.header.stamp;
      inspected.header.frame_id = params_.navigationFrame_;
      inspected.type = visualization_msgs::Marker::TRIANGLE_LIST;
      inspected.lifetime = ros::Duration(10);
      inspected.action = visualization_msgs::Marker::ADD;
      inspected.pose.position.x = 0.0;
      inspected.pose.position.y = 0.0;
      inspected.pose.position.z = 0.0;
      inspected.pose.orientation.x = 0.0;
      inspected.pose.orientation.y = 0.0;
      inspected.pose.orientation.z = 0.0;
      inspected.pose.orientation.w = 1.0;
      inspected.scale.x = 1.0;
      inspected.scale.y = 1.0;
      inspected.scale.z = 1.0;
      visualization_msgs::Marker uninspected = inspected;
      uninspected.header.seq++;
      uninspected.id++;
      uninspected.ns = "meshUninspected";
      mesh_->assembleMarkerArray(inspected, uninspected);
      if (inspected.points.size() > 0) {
        params_.inspectionPath_.publish(inspected);
      }
      if (uninspected.points.size() > 0) {
        params_.inspectionPath_.publish(uninspected);
      }
    }
  }
}

void nbvInspection::RrtTree::setStateFromOdometryMsg(
    const nav_msgs::Odometry& pose)
{
  // Get latest transform to the planning frame and transform the pose
  static tf::TransformListener listener;
  tf::StampedTransform transform;
  try {
    listener.lookupTransform(params_.navigationFrame_, pose.header.frame_id, pose.header.stamp,
                             transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }
  tf::Pose poseTF;
  tf::poseMsgToTF(pose.pose.pose, poseTF);
  tf::Vector3 position = poseTF.getOrigin();
  position = transform * position;
  tf::Quaternion quat = poseTF.getRotation();
  quat = transform * quat;
  root_[0] = position.x();
  root_[1] = position.y();
  root_[2] = position.z();
  root_[3] = tf::getYaw(quat);

  // Log the vehicle response in the planning frame
  static double logThrottleTime = ros::Time::now().toSec();
  if (ros::Time::now().toSec() - logThrottleTime > params_.log_throttle_) {
    logThrottleTime += params_.log_throttle_;
    if (params_.log_) {
      for (int i = 0; i < root_.size() - 1; i++) {
        fileResponse_ << root_[i] << ",";
      }
      fileResponse_ << root_[root_.size() - 1] << "\n";
    }
  }
  // Update the inspected parts of the mesh using the current position
  if (ros::Time::now().toSec() - inspectionThrottleTime_[0] > params_.inspection_throttle_) {
    inspectionThrottleTime_[0] += params_.inspection_throttle_;
    geometry_msgs::Pose poseTransformed;
    tf::poseTFToMsg(transform * poseTF, poseTransformed);
    setPeerPoseInTree(poseTransformed, 0);
    if (mesh_) {
      geometry_msgs::Pose poseTransformed;
      tf::poseTFToMsg(transform * poseTF, poseTransformed);
      mesh_->setPeerPose(poseTransformed, 0);
      mesh_->incorporateViewFromPoseMsg(poseTransformed, 0);
      // Publish the mesh marker for visualization in rviz
      visualization_msgs::Marker inspected;
      inspected.ns = "meshInspected";
      inspected.id = 0;
      inspected.header.seq = inspected.id;
      inspected.header.stamp = pose.header.stamp;
      inspected.header.frame_id = params_.navigationFrame_;
      inspected.type = visualization_msgs::Marker::TRIANGLE_LIST;
      inspected.lifetime = ros::Duration(10);
      inspected.action = visualization_msgs::Marker::ADD;
      inspected.pose.position.x = 0.0;
      inspected.pose.position.y = 0.0;
      inspected.pose.position.z = 0.0;
      inspected.pose.orientation.x = 0.0;
      inspected.pose.orientation.y = 0.0;
      inspected.pose.orientation.z = 0.0;
      inspected.pose.orientation.w = 1.0;
      inspected.scale.x = 1.0;
      inspected.scale.y = 1.0;
      inspected.scale.z = 1.0;
      visualization_msgs::Marker uninspected = inspected;
      uninspected.header.seq++;
      uninspected.id++;
      uninspected.ns = "meshUninspected";
      mesh_->assembleMarkerArray(inspected, uninspected);
      if (inspected.points.size() > 0) {
        params_.inspectionPath_.publish(inspected);
      }
      if (uninspected.points.size() > 0) {
        params_.inspectionPath_.publish(uninspected);
      }
    }
  }
}

void nbvInspection::RrtTree::setPeerStateFromPoseMsg(
    const geometry_msgs::PoseWithCovarianceStamped& pose, int n_peer)
{
  // Get latest transform to the planning frame and transform the pose
  static tf::TransformListener listener;
  tf::StampedTransform transform;
  try {
    listener.lookupTransform(params_.navigationFrame_, pose.header.frame_id, pose.header.stamp,
                             transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }
  tf::Pose poseTF;
  tf::poseMsgToTF(pose.pose.pose, poseTF);
  geometry_msgs::Pose poseTransformed;
  tf::poseTFToMsg(transform * poseTF, poseTransformed);
  // Update the inspected parts of the mesh using the current position
  if (ros::Time::now().toSec() - inspectionThrottleTime_[n_peer] > params_.inspection_throttle_) {
    inspectionThrottleTime_[n_peer] += params_.inspection_throttle_;
    setPeerPoseInTree(poseTransformed, n_peer);
    if (mesh_) {
      mesh_->setPeerPose(poseTransformed, n_peer);
      mesh_->incorporateViewFromPoseMsg(poseTransformed, n_peer);
    }
  }
}

std::vector<tf::Vector3> nbvInspection::RrtTree::getPeerPose(int num)
{
  return peer_vehicles_;
}

void nbvInspection::RrtTree::setPeerPoseInTree(const geometry_msgs::Pose& pose, int n_peer)
{
  if (peer_vehicles_.size() > n_peer) {
    peer_vehicles_[n_peer] = tf::Vector3(pose.position.x, pose.position.y, pose.position.z);
    return;
  }
  while (peer_vehicles_.size() <= n_peer) {
    peer_vehicles_.push_back(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
  }
}

bool nbvInspection::RrtTree::biased_coin(double probability)
{
  if (((double) rand()) / ((double) RAND_MAX) < probability)
    return true;
  return false;
}

void nbvInspection::RrtTree::iterate(std::vector<Eigen::Vector4d> peer_target)
{
// In this function a new configuration is sampled and added to the tree.
  StateVec newState;
// Sample over a sphere with the radius of the maximum diagonal of the exploration
// space. Throw away samples outside the sampling region it exiting is not allowed
// by the corresponding parameter. This method is to not bias the tree towards the
// center of the exploration space.
  double radius = sqrt(
      SQ(params_.minX_ - params_.maxX_) + SQ(params_.minY_ - params_.maxY_)
      + SQ(params_.minZ_ - params_.maxZ_));
  bool solutionFound = false;
  while (!solutionFound) {
    for (int i = 0; i < 3; i++) {
      newState[i] = 2.0 * radius * (((double) rand()) / ((double) RAND_MAX) - 0.5);
    }
    if (SQ(newState[0]) + SQ(newState[1]) + SQ(newState[2]) > pow(radius, 2.0))
      continue;
    // Offset new state by root
    newState += rootNode_->state_;
    if (!params_.softBounds_) {
        if (newState.x() < params_.minX_ + 0.5 * params_.boundingBox_.x()) {
            continue;
        } else if (newState.y() < params_.minY_ + 0.5 * params_.boundingBox_.y()) {
            continue;
        } else if (newState.z() < params_.minZ_ + 0.5 * params_.boundingBox_.z()) {
            continue;
        } else if (newState.x() > params_.maxX_ - 0.5 * params_.boundingBox_.x()) {
            continue;
        } else if (newState.y() > params_.maxY_ - 0.5 * params_.boundingBox_.y()) {
            continue;
        } else if (newState.z() > params_.maxZ_ - 0.5 * params_.boundingBox_.z()) {
            continue;
        }
    }

    if (params_.explr_mode_==1) { // use of VBF
      bool outOfSelfVoronoi = false;
      double my_dist_sq = SQ(peer_vehicles_[0].x() - newState[0])
                          + SQ(peer_vehicles_[0].y() - newState[1]) + SQ(peer_vehicles_[0].z() - newState[2]);
      for (int i = 1; i < peer_vehicles_.size(); i++) {
        if (peer_vehicles_[i] == tf::Vector3(4, 4, 0.13))
          continue;
        double peer_dist_sq = SQ(peer_vehicles_[i].x() - newState[0]) + SQ(peer_vehicles_[i].y() - newState[1])
                              + SQ(peer_vehicles_[i].z() - newState[2]);
        if (peer_dist_sq < my_dist_sq)
          outOfSelfVoronoi = true;
      }
      if (outOfSelfVoronoi) {
        if (biased_coin(params_.voronoiBias_))
          continue;
      } else {
        if (biased_coin(1 - params_.voronoiBias_))
          continue;
      }
    }
    solutionFound = true;
  }

// Find nearest neighbour
  kdres * nearest = kd_nearest3(kdTree_, newState.x(), newState.y(), newState.z());
  if (kd_res_size(nearest) <= 0) {
    kd_res_free(nearest);
    return;
  }
  nbvInspection::Node<StateVec> * newParent = (nbvInspection::Node<StateVec> *) kd_res_item_data(nearest);
  kd_res_free(nearest);

// Check for collision of new connection plus some overshoot distance.
  Eigen::Vector3d origin(newParent->state_[0], newParent->state_[1], newParent->state_[2]);
  Eigen::Vector3d direction(newState[0] - origin[0], newState[1] - origin[1], newState[2] - origin[2]);
  if (direction.norm() > params_.extensionRange_) {
    direction = params_.extensionRange_ * direction.normalized();
  }
  newState[0] = origin[0] + direction[0];
  newState[1] = origin[1] + direction[1];
  newState[2] = origin[2] + direction[2];
  if (volumetric_mapping::OctomapManager::CellStatus::kFree
      == manager_->getLineStatusBoundingBox(
          origin, direction + origin + direction.normalized() * params_.dOvershoot_,
          params_.boundingBox_)
      && !multiagent::isInCollision(newParent->state_, newState, params_.boundingBox_, segments_)) {

    // Sample the new orientation
    newState[3] = 0;
      for (int i = 1; i < 20; i++){
          double temp = 2.0 * M_PI * i / 20;
          StateVec temp_state;
          temp_state[0] = newState[0];
          temp_state[1] = newState[1];
          temp_state[2] = newState[2];
          temp_state[3] = temp;
          if (gain(newState) < gain(temp_state)){
              newState[3] = temp;
          }
      }

    // Create new node and insert into tree
    nbvInspection::Node<StateVec> * newNode = new nbvInspection::Node<StateVec>;
    newNode->state_ = newState;
    newNode->parent_ = newParent;
    newNode->distance_ = newParent->distance_ + direction.norm();
    newParent->children_.push_back(newNode);

    if (params_.explr_mode_==2) { // cost-utility
      double others_f = 0;
      for (int i = 0; i < 2; i++) {
        if (peer_target[i][3]) {
          double dist_to_peer_sq = 0;
          for (int j = 0; j < 3; j++)
            dist_to_peer_sq += SQ(peer_target[i][j]-newState[j]);
          double dist_to_peer = sqrt(dist_to_peer_sq);
          if (dist_to_peer < params_.radiusInfluence_)
            others_f += sqrt(dist_to_peer_sq) / params_.radiusInfluence_;
        }
      }
      newNode->gain_ = newParent->gain_
                       + gain(newNode->state_) * exp(-params_.degressiveCoeff_ * newNode->distance_)
                       * exp(-params_.cuCoeff_ * others_f);
    } else { // use of VBF / no coordination
      newNode->gain_ = newParent->gain_
                       + gain(newNode->state_) * exp(-params_.degressiveCoeff_ * newNode->distance_);
    }

    double buf[4];
      for (int i = 0; i<4; i++){
          buf[i] = newState[i];
      }
    kd_insert(kdTree_, buf , newNode);
    
      // kd_insert4(kdTree_, newState[0], newState[1], newState[2], newState[3], newNode);

      // Display new node
    publishNode(newNode);
    rootNode_->allNode.push_back(newNode);

    if (newNode->parent_->parent_ == NULL) { //new direction
        newNode->dirNum_ = newParent->children_.size()-1;
    } else {
        newNode->dirNum_ = newParent->dirNum_;
    }

    newNode->isLeaf = true;
    if (newParent->isLeaf){
        newParent->isLeaf = false;
    }

    // Update best IG and node if applicable
    if (newNode->gain_ > bestGain_) {
      bestGain_ = newNode->gain_;
      bestNode_ = newNode;
    }
    counter_++;
  }
}

void nbvInspection::RrtTree::initialize(std::vector<Eigen::Vector4d> peer_target)
{
// This function is to initialize the tree, including insertion of remainder of previous best branch.
  g_ID_ = 0;
// Remove last segment from segment list (multi agent only)
  int i;
  for (i = 0; i < agentNames_.size(); i++) {
    if (agentNames_[i].compare(params_.navigationFrame_) == 0) {
      break;
    }
  }
  if (i < agentNames_.size()) {
    segments_[i]->clear();
  }
// Initialize kd-tree with root node and prepare log file
  kdTree_ = kd_create(4);

  if (params_.log_) {
    if (fileTree_.is_open()) {
      fileTree_.close();
    }
    fileTree_.open((logFilePath_ + "tree" + std::to_string(iterationCount_) + ".txt").c_str(),
                   std::ios::out);
  }

  rootNode_ = new Node<StateVec>;
  rootNode_->distance_ = 0.0;
  rootNode_->gain_ = params_.zero_gain_;
  rootNode_->parent_ = NULL;
  rootNode_->isLeaf = false;
  rootNode_->dirNum_ = 0;

  std::vector<Node<StateVec> *> v;
  rootNode_->allNode = v;
  rootNode_->leafNode = v;
  rootNode_->allNode.push_back(rootNode_);

  if (params_.exact_root_) {
    if (iterationCount_ <= 1) {
      exact_root_ = root_;
    }
    rootNode_->state_ = exact_root_;
  } else {
    rootNode_->state_ = root_;
  }

  double buf[4];
    for (int i =0; i<4; i++) {
      buf[i]=rootNode_->state_[i];
    }
  kd_insert(kdTree_, buf, rootNode_);

    iterationCount_++;

// Insert all nodes of the remainder of the previous best branch, checking for collisions and
// recomputing the gain.
  for (typename std::vector<StateVec>::reverse_iterator iter = bestBranchMemory_.rbegin();
      iter != bestBranchMemory_.rend(); ++iter) {
    StateVec newState = *iter;
    kdres * nearest = kd_nearest3(kdTree_, newState.x(), newState.y(), newState.z());
    if (kd_res_size(nearest) <= 0) {
      kd_res_free(nearest);
      continue;
    }
    nbvInspection::Node<StateVec> * newParent = (nbvInspection::Node<StateVec> *) kd_res_item_data(
        nearest);
    kd_res_free(nearest);

    // Check for collision
    Eigen::Vector3d origin(newParent->state_[0], newParent->state_[1], newParent->state_[2]);
    Eigen::Vector3d direction(newState[0] - origin[0], newState[1] - origin[1],
                              newState[2] - origin[2]);
    if (direction.norm() > params_.extensionRange_) {
      direction = params_.extensionRange_ * direction.normalized();
    }
    newState[0] = origin[0] + direction[0];
    newState[1] = origin[1] + direction[1];
    newState[2] = origin[2] + direction[2];
    if (volumetric_mapping::OctomapManager::CellStatus::kFree
        == manager_->getLineStatusBoundingBox(
            origin, direction + origin + direction.normalized() * params_.dOvershoot_,
            params_.boundingBox_)
        && !multiagent::isInCollision(newParent->state_, newState, params_.boundingBox_, segments_)) {
      // Create new node and insert into tree
      nbvInspection::Node<StateVec> * newNode = new nbvInspection::Node<StateVec>;
      newNode->state_ = newState;
      newNode->parent_ = newParent;
      newNode->distance_ = newParent->distance_ + direction.norm();
      newParent->children_.push_back(newNode);
      double dist_cost;
      if (newNode->distance_ >= params_.v_max_ * fabs(newParent->state_[3] - newState[3]) / params_.dyaw_max_)
          dist_cost = newNode->distance_;
      else
          dist_cost = params_.v_max_ * fabs(newParent->state_[3] - newState[3]) / params_.dyaw_max_;

      if (params_.explr_mode_==2) { // cost-utility
        double others_f = 0;
        for (int i = 0; i < 2; i++) {
          if (peer_target[i][3]) {
            double dist_to_peer_sq = 0;
            for (int j = 0; j < 3; j++)
              dist_to_peer_sq += SQ(peer_target[i][j]-newState[j]);
            double dist_to_peer = sqrt(dist_to_peer_sq);
            if (dist_to_peer < params_.radiusInfluence_)
              others_f += sqrt(dist_to_peer_sq) / params_.radiusInfluence_;
          }
        }
        newNode->gain_ = newParent->gain_
                         + gain(newNode->state_) * exp(-params_.degressiveCoeff_ * dist_cost)
                           * exp(-params_.cuCoeff_ * others_f);
      } else { // use of VBF / no coordination
        newNode->gain_ = newParent->gain_
                         + gain(newNode->state_) * exp(-params_.degressiveCoeff_ * dist_cost);
      }

      for (int i = 0; i<4; i++){buf[i] = newState[i];}
      kd_insert(kdTree_, buf, newNode);
        // kd_insert4(kdTree_, newState[0], newState[1], newState[2], newState[3], newNode);

      newNode->dirNum_ = 0;
      // Display new node
      publishNode(newNode);
      rootNode_->allNode.push_back(newNode);

      newNode->isLeaf = true;
      if (newParent->isLeaf) {
          newParent->isLeaf = false;
      }

      // Update best IG and node if applicable
      if (newNode->gain_ > bestGain_) {
        bestGain_ = newNode->gain_;
        bestNode_ = newNode;
      }
      counter_++;
    }
  }

// Publish visualization of total exploration area
  visualization_msgs::Marker p;
  p.header.stamp = ros::Time::now();
  p.header.seq = 0;
  p.header.frame_id = params_.navigationFrame_;
  p.id = 0;
  p.ns = "workspace";
  p.type = visualization_msgs::Marker::CUBE;
  p.action = visualization_msgs::Marker::ADD;
  p.pose.position.x = 0.5 * (params_.minX_ + params_.maxX_);
  p.pose.position.y = 0.5 * (params_.minY_ + params_.maxY_);
  p.pose.position.z = 0.5 * (params_.minZ_ + params_.maxZ_);
  tf::Quaternion quat;
  quat.setEuler(0.0, 0.0, 0.0);
  p.pose.orientation.x = quat.x();
  p.pose.orientation.y = quat.y();
  p.pose.orientation.z = quat.z();
  p.pose.orientation.w = quat.w();
  p.scale.x = params_.maxX_ - params_.minX_;
  p.scale.y = params_.maxY_ - params_.minY_;
  p.scale.z = params_.maxZ_ - params_.minZ_;
  p.color.r = 200.0 / 255.0;
  p.color.g = 100.0 / 255.0;
  p.color.b = 0.0;
  p.color.a = 0.1;
  p.lifetime = ros::Duration(0.0);
  p.frame_locked = false;
  params_.inspectionPath_.publish(p);
}

void nbvInspection::RrtTree::getLeafNode(int dummy)
{
    int n = rootNode_->allNode.size();
    for (int i=0; i<n; i++){
        if (rootNode_->allNode[i]->isLeaf) {
            rootNode_->leafNode.push_back(rootNode_->allNode[i]);
        }
    }
}

bool nbvInspection::RrtTree::cmp(Node<StateVec> * a, Node<StateVec> * b)
{
    return a->gain_ > b->gain_;
}

std::vector<nbvInspection::Node<StateVec> *> nbvInspection::RrtTree::getCandidates()
{
    int maxNum = rootNode_->children_.size();
    std::vector<std::vector<Node<StateVec> *>> classified(maxNum);

    int n = rootNode_->leafNode.size();
    int dir = 0;

    for (int i; i<n; i++){
        Node<StateVec> * currentNode = rootNode_->leafNode[i];
        dir = currentNode->dirNum_;
        classified[dir].push_back(currentNode);
    }

    int max = 0;
    for (int i=0; i<maxNum; i++){
        if (max < classified[i].size()) {
            max = classified[i].size();
        }
    }

    Node<StateVec> * dummyNode = new Node<StateVec>;
    dummyNode->gain_ = params_.zero_gain_;

    for (int i=0; i<maxNum; i++) {
        int t = classified[i].size();
        for (int j = 0; j < max - t; j++) {
            classified[i].push_back(dummyNode);
        }
    }

    for (int i=0; i<maxNum; i++){
        std::sort(classified[i].begin(), classified[i].end(), cmp);
    }

    std::vector<Node<StateVec> *> candidates;
    std::vector<Node<StateVec> *> competitive;

    for (int i=0; i<max; i++){
        competitive.clear();
        for (int j=0; j<maxNum; j++){
            competitive.push_back(classified[j][i]);
        }

        std::sort(competitive.begin(), competitive.end(), cmp);
        candidates.push_back(competitive[0]);
    }

    return candidates;
}

void nbvInspection::RrtTree::changeBestNode(std::vector<Node<Eigen::Vector4d> *> candidates,
                                            std::vector<Eigen::Vector4d> peer_target)
{
    int n = candidates.size();
    params_->
}

std::vector<geometry_msgs::Pose> nbvInspection::RrtTree::getBestEdge(std::string targetFrame)
{
// This function returns the first edge of the best branch
  std::vector<geometry_msgs::Pose> ret;
  nbvInspection::Node<StateVec> * current = bestNode_;
  if (current->parent_ != NULL) {
    while (current->parent_ != rootNode_ && current->parent_ != NULL) {
      current = current->parent_;
    }
    ret = samplePath(current->parent_->state_, current->state_, targetFrame, ret);
    history_.push(current->parent_->state_);
    exact_root_ = current->state_;
  }
  return ret;
}

double nbvInspection::RrtTree::gain(StateVec state)
{
// This function computes the gain
  double gain = 0.0;
  const double disc = manager_->getResolution();
  Eigen::Vector3d origin(state[0], state[1], state[2]);
  Eigen::Vector3d vec;
  double rangeSq = pow(params_.gainRange_, 2.0);
// Iterate over all nodes within the allowed distance
  for (vec[0] = std::max(state[0] - params_.gainRange_, params_.minX_);
      vec[0] < std::min(state[0] + params_.gainRange_, params_.maxX_); vec[0] += disc) {
    for (vec[1] = std::max(state[1] - params_.gainRange_, params_.minY_);
        vec[1] < std::min(state[1] + params_.gainRange_, params_.maxY_); vec[1] += disc) {
      for (vec[2] = std::max(state[2] - params_.gainRange_, params_.minZ_);
          vec[2] < std::min(state[2] + params_.gainRange_, params_.maxZ_); vec[2] += disc) {
        Eigen::Vector3d dir = vec - origin;
        // Skip if distance is too large
        if (dir.norm() > rangeSq) {
          continue;
        }
        bool bbreak = false;
        // Check that voxel center is inside one of the fields of view.
        for (typename std::vector<std::vector<Eigen::Vector3d>>::iterator itCBN = params_
            .camBoundNormals_.begin(); itCBN != params_.camBoundNormals_.end(); itCBN++) {
          bool inFoV = true;
          for (typename std::vector<Eigen::Vector3d>::iterator itSingleCBN = itCBN->begin();
              itSingleCBN != itCBN->end(); itSingleCBN++) {
            Eigen::Vector3d normal = Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitZ())
                * (*itSingleCBN);
            double val = dir.dot(normal.normalized());
            if (val < SQRT2 * disc) {
              inFoV = false;
              break;
            }
          }
          if (!inFoV) {
            bbreak = true;
            break;
          }
        }
        if (bbreak) {
          continue;
        }
        // Check cell status and add to the gain considering the corresponding factor.
        double probability;
        volumetric_mapping::OctomapManager::CellStatus node = manager_->getCellProbabilityPoint(
            vec, &probability);
        if (node == volumetric_mapping::OctomapManager::CellStatus::kUnknown) {
          // Rayshooting to evaluate inspectability of cell
          if (volumetric_mapping::OctomapManager::CellStatus::kOccupied
              != this->manager_->getVisibility(origin, vec, false)) {
            gain += params_.igUnmapped_;
            // TODO: Add probabilistic gain
            // gain += params_.igProbabilistic_ * PROBABILISTIC_MODEL(probability);
          }
        } else if (node == volumetric_mapping::OctomapManager::CellStatus::kOccupied) {
          // Rayshooting to evaluate inspectability of cell
          if (volumetric_mapping::OctomapManager::CellStatus::kOccupied
              != this->manager_->getVisibility(origin, vec, false)) {
            gain += params_.igOccupied_;
            // TODO: Add probabilistic gain
            // gain += params_.igProbabilistic_ * PROBABILISTIC_MODEL(probability);
          }
        } else {
          // Rayshooting to evaluate inspectability of cell
          if (volumetric_mapping::OctomapManager::CellStatus::kOccupied
              != this->manager_->getVisibility(origin, vec, false)) {
            gain += params_.igFree_;
            // TODO: Add probabilistic gain
            // gain += params_.igProbabilistic_ * PROBABILISTIC_MODEL(probability);
          }
        }
      }
    }
  }
// Scale with volume
  gain *= pow(disc, 3.0);
// Check the gain added by inspectable surface
  if (mesh_) {
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(state.x(), state.y(), state.z()));
    tf::Quaternion quaternion;
    quaternion.setEuler(0.0, 0.0, state[3]);
    transform.setRotation(quaternion);
    gain += params_.igArea_ * mesh_->computeInspectableArea(transform);
  }
  return gain;
}

std::vector<geometry_msgs::Pose> nbvInspection::RrtTree::getPathBackToPrevious(
    std::string targetFrame)
{

  std::cout << "-----------getPathBackToPrevious------------" << std::endl;

  std::vector<geometry_msgs::Pose> ret;
  if (history_.empty()) {
    return ret;
  }
  ret = samplePath(root_, history_.top(), targetFrame, ret);
  exact_root_ = history_.top();
  history_.pop();
  return ret;
}

void nbvInspection::RrtTree::memorizeBestBranch()
{
  bestBranchMemory_.clear();
  Node<StateVec> * current = bestNode_;
  while (current->parent_ && current->parent_->parent_) {
    for (int i = 1; i < 20; i++){
      double temp = 2.0 * M_PI * i / 20;
      StateVec temp_state;
      temp_state[0] = current->state_[0];
      temp_state[1] = current->state_[1];
      temp_state[2] = current->state_[2];
      temp_state[3] = temp;
      if (gain(current->state_) < gain(temp_state)){
          current->state_[3] = temp;
      }
    }
    bestBranchMemory_.push_back(current->state_);
    current = current->parent_;
  }
}

void nbvInspection::RrtTree::clear()
{
  delete rootNode_;
  rootNode_ = NULL;

  counter_ = 0;
  bestGain_ = params_.zero_gain_;
  bestNode_ = NULL;

  kd_free(kdTree_);
}

void nbvInspection::RrtTree::publishNode(Node<StateVec> * node)
{
  visualization_msgs::Marker p;
  p.header.stamp = ros::Time::now();
  p.header.seq = g_ID_;
  p.header.frame_id = params_.navigationFrame_;
  p.id = g_ID_;
  g_ID_++;
  p.ns = "vp_tree";
  p.type = visualization_msgs::Marker::ARROW;
  p.action = visualization_msgs::Marker::ADD;
  p.pose.position.x = node->state_[0];
  p.pose.position.y = node->state_[1];
  p.pose.position.z = node->state_[2];
  tf::Quaternion quat;
  quat.setEuler(0.0, 0.0, node->state_[3]);
  p.pose.orientation.x = quat.x();
  p.pose.orientation.y = quat.y();
  p.pose.orientation.z = quat.z();
  p.pose.orientation.w = quat.w();
  p.scale.x = std::max(node->gain_ / 20.0, 0.05);
  p.scale.y = 0.1;
  p.scale.z = 0.1;
  p.color.r = 167.0 / 255.0;
  p.color.g = 167.0 / 255.0;
  p.color.b = 0.0;
  p.color.a = 1.0;
  p.lifetime = ros::Duration(10.0);
  p.frame_locked = false;
  params_.inspectionPath_.publish(p);

  if (!node->parent_)
    return;

  p.id = g_ID_;
  g_ID_++;
  p.ns = "vp_branches";
  p.type = visualization_msgs::Marker::ARROW;
  p.action = visualization_msgs::Marker::ADD;
  p.pose.position.x = node->parent_->state_[0];
  p.pose.position.y = node->parent_->state_[1];
  p.pose.position.z = node->parent_->state_[2];
  Eigen::Quaternion<float> q;
  Eigen::Vector3f init(1.0, 0.0, 0.0);
  Eigen::Vector3f dir(node->state_[0] - node->parent_->state_[0],
                      node->state_[1] - node->parent_->state_[1],
                      node->state_[2] - node->parent_->state_[2]);
  q.setFromTwoVectors(init, dir);
  q.normalize();
  p.pose.orientation.x = q.x();
  p.pose.orientation.y = q.y();
  p.pose.orientation.z = q.z();
  p.pose.orientation.w = q.w();
  p.scale.x = dir.norm();
  p.scale.y = 0.03;
  p.scale.z = 0.03;
  p.color.r = 100.0 / 255.0;
  p.color.g = 100.0 / 255.0;
  p.color.b = 0.7;
  p.color.a = 1.0;
  p.lifetime = ros::Duration(10.0);
  p.frame_locked = false;
  params_.inspectionPath_.publish(p);

  if (params_.log_) {
    for (int i = 0; i < node->state_.size(); i++) {
      fileTree_ << node->state_[i] << ",";
    }
    fileTree_ << node->gain_ << ",";
    for (int i = 0; i < node->parent_->state_.size(); i++) {
      fileTree_ << node->parent_->state_[i] << ",";
    }
    fileTree_ << node->parent_->gain_ << "\n";
  }
}

std::vector<geometry_msgs::Pose> nbvInspection::RrtTree::samplePath(StateVec start, StateVec end,
                                                                    std::string targetFrame,
                                                                    std::vector<geometry_msgs::Pose> ret)
{
  // std::vector<geometry_msgs::Pose> ret;
  static tf::TransformListener listener;
  tf::StampedTransform transform;
  try {
    listener.lookupTransform(targetFrame, params_.navigationFrame_, ros::Time(0), transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return ret;
  }
  Eigen::Vector3d distance(end[0] - start[0], end[1] - start[1], end[2] - start[2]);
  double yaw_direction = end[3] - start[3];
  if (yaw_direction > M_PI) {
    yaw_direction -= 2.0 * M_PI;
  }
  if (yaw_direction < -M_PI) {
    yaw_direction += 2.0 * M_PI;
  }
  double disc = std::min(params_.dt_ * params_.v_max_ / distance.norm(),
                         params_.dt_ * params_.dyaw_max_ / abs(yaw_direction));
  assert(disc > 0.0);
  for (double it = 0.0; it <= 1.0; it += disc) {
    tf::Vector3 origin((1.0 - it) * start[0] + it * end[0], (1.0 - it) * start[1] + it * end[1],
                       (1.0 - it) * start[2] + it * end[2]);
    double yaw = start[3] + yaw_direction * it;
    if (yaw > M_PI)
      yaw -= 2.0 * M_PI;
    if (yaw < -M_PI)
      yaw += 2.0 * M_PI;
    tf::Quaternion quat;
    quat.setEuler(0.0, 0.0, yaw);
    origin = transform * origin;
    quat = transform * quat;
    tf::Pose poseTF(quat, origin);
    geometry_msgs::Pose pose;
    tf::poseTFToMsg(poseTF, pose);
    ret.push_back(pose);
    if (params_.log_) {
      filePath_ << poseTF.getOrigin().x() << ",";
      filePath_ << poseTF.getOrigin().y() << ",";
      filePath_ << poseTF.getOrigin().z() << ",";
      filePath_ << tf::getYaw(poseTF.getRotation()) << "\n";
    }
  }
  return ret;
}

struct kdtree* nbvInspection::RrtTree::get_kdtree()
{
  return kdTree_;
}

/*---------------------Volumetric RRT Method (Start) ----------------------------*/
void nbvInspection::RrtTree::VRRT_iterate(std::vector<Eigen::Vector4d> peer_target)
{
// In this function a new configuration is sampled and added to the tree.
    StateVec newState;
// Sample over a sphere with the radius of the maximum diagonal of the exploration
// space. Throw away samples outside the sampling region it exiting is not allowed
// by the corresponding parameter. This method is to not bias the tree towards the
// center of the exploration space.
  double radius = sqrt(
          SQ(params_.minX_ - params_.maxX_) + SQ(params_.minY_ - params_.maxY_)
          + SQ(params_.minZ_ - params_.maxZ_));
  bool solutionFound = false;
  while (!solutionFound) {
    for (int i = 0; i < 3; i++) {
      newState[i] = 2.0 * radius * (((double) rand()) / ((double) RAND_MAX) - 0.5);
    }
    if (SQ(newState[0]) + SQ(newState[1]) + SQ(newState[2]) > pow(radius, 2.0))
      continue;
    // Offset new state by root
    newState += rootNode_->state_;
    if (!params_.softBounds_) {
      if (newState.x() < params_.minX_ + 0.5 * params_.boundingBox_.x()) {
        continue;
      } else if (newState.y() < params_.minY_ + 0.5 * params_.boundingBox_.y()) {
        continue;
      } else if (newState.z() < params_.minZ_ + 0.5 * params_.boundingBox_.z()) {
        continue;
      } else if (newState.x() > params_.maxX_ - 0.5 * params_.boundingBox_.x()) {
        continue;
      } else if (newState.y() > params_.maxY_ - 0.5 * params_.boundingBox_.y()) {
        continue;
      } else if (newState.z() > params_.maxZ_ - 0.5 * params_.boundingBox_.z()) {
        continue;
      }
    }

    if (params_.explr_mode_==1) { // use of VBF
      bool outOfSelfVoronoi = false;
      double my_dist_sq = SQ(peer_vehicles_[0].x() - newState[0])
                          + SQ(peer_vehicles_[0].y() - newState[1]) + SQ(peer_vehicles_[0].z() - newState[2]);
      for (int i = 1; i < peer_vehicles_.size(); i++) {
        if (peer_vehicles_[i] == tf::Vector3(4, 4, 0.13))
          continue;
        double peer_dist_sq = SQ(peer_vehicles_[i].x() - newState[0]) + SQ(peer_vehicles_[i].y() - newState[1])
                              + SQ(peer_vehicles_[i].z() - newState[2]);
        if (peer_dist_sq < my_dist_sq)
          outOfSelfVoronoi = true;
      }
      if (outOfSelfVoronoi) {
        if (biased_coin(params_.voronoiBias_))
          continue;
      } else {
        if (biased_coin(1 - params_.voronoiBias_))
          continue;
      }
    }
    solutionFound=true;
  }

// Find nearest neighbour
  kdres * nearest = kd_nearest3(kdTree_, newState.x(), newState.y(), newState.z());
  if (kd_res_size(nearest) <= 0) {
    kd_res_free(nearest);
    return;
  }
  nbvInspection::Node<StateVec> * newParent = (nbvInspection::Node<StateVec> *) kd_res_item_data(nearest);
  kd_res_free(nearest);

// Check for collision of new connection plus some overshoot distance.
  Eigen::Vector3d origin(newParent->state_[0], newParent->state_[1], newParent->state_[2]);
  Eigen::Vector3d direction(newState[0] - origin[0], newState[1] - origin[1], newState[2] - origin[2]);
  if (direction.norm() > params_.extensionRange_) {
    direction = params_.extensionRange_ * direction.normalized();
  }
  newState[0] = origin[0] + direction[0];
  newState[1] = origin[1] + direction[1];
  newState[2] = origin[2] + direction[2];
  if (volumetric_mapping::OctomapManager::CellStatus::kFree
      == manager_->getLineStatusBoundingBox(
          origin, direction + origin + direction.normalized() * params_.dOvershoot_,
          params_.boundingBox_)
      && !multiagent::isInCollision(newParent->state_, newState, params_.boundingBox_, segments_)) {

    // Sample the new orientation
    newState[3] = 0;
    for (int i = 1; i < 20; i++){
      double temp = 2.0 * M_PI * i / 20;
      StateVec temp_state;
      temp_state[0] = newState[0];
      temp_state[1] = newState[1];
      temp_state[2] = newState[2];
      temp_state[3] = temp;
      if (gain(newState) < gain(temp_state)){
        newState[3] = temp;
      }
    }
    // Create new node and insert into tree
    nbvInspection::Node<StateVec> * newNode = new nbvInspection::Node<StateVec>;
    newNode->state_ = newState;
    newNode->parent_ = newParent;
    newNode->distance_ = newParent->distance_ + direction.norm();
    newParent->children_.push_back(newNode);
    double dist_cost;
    if (newNode->distance_ >= params_.v_max_ * fabs(newParent->state_[3] - newState[3]) / params_.dyaw_max_)
      dist_cost = newNode->distance_;
    else
      dist_cost = params_.v_max_ * fabs(newParent->state_[3] - newState[3]) / params_.dyaw_max_;
    if (params_.explr_mode_==2) { // cost-utility
      double others_f = 0;
      for (int i = 0; i < 2; i++) {
        if (peer_target[i][3]) {
          double dist_to_peer_sq = 0;
          for (int j = 0; j < 3; j++)
            dist_to_peer_sq += SQ(peer_target[i][j]-newState[j]);
          double dist_to_peer = sqrt(dist_to_peer_sq);
          if (dist_to_peer < params_.radiusInfluence_)
            others_f += sqrt(dist_to_peer_sq) / params_.radiusInfluence_;
        }
      }
      newNode->gain_ = gain(newNode->state_) * exp(-params_.degressiveCoeff_ * dist_cost)
                         * exp(-params_.cuCoeff_ * others_f);
    } else { // use of VBF / no coordination
      newNode->gain_ = gain(newNode->state_) * exp(-params_.degressiveCoeff_ * dist_cost);
    }

    double buf[4];
    for (int i = 0; i<4; i++){
      buf[i] = newState[i];
    }
    kd_insert(kdTree_, buf , newNode);

    // Display new node
    publishNode(newNode);
    rootNode_->allNode.push_back(newNode);

    if (newNode->parent_->parent_ == NULL) { //new direction
      newNode->dirNum_ = newParent->children_.size()-1;
    } else {
      newNode->dirNum_ = newParent->dirNum_;
    }

    newNode->isLeaf = true;
    if (newParent->isLeaf){
      newParent->isLeaf = false;
    }

    // Update best IG and node if applicable
    if (newNode->gain_ > bestGain_) {
      bestGain_ = newNode->gain_;
      bestNode_ = newNode;
    }
    counter_++;
  }
}

std::vector<geometry_msgs::Pose> nbvInspection::RrtTree::VRRT_getBestEdge(std::string targetFrame) {
// This function returns the first edge of the best branch
  std::vector<geometry_msgs::Pose> ret;
  nbvInspection::Node<StateVec> * current = bestNode_;

  std::vector<nbvInspection::Node<StateVec>*> states;
  std::vector<nbvInspection::Node<StateVec>*> states_reverse;
  if (current->parent_ != NULL) {
    while (current->parent_ != rootNode_ && current->parent_ != NULL) {
      states_reverse.push_back(current);
      current = current->parent_;
    }
    states_reverse.push_back(current);
    states_reverse.push_back(current->parent_);
    exact_root_ = bestNode_->state_;
  }
  for(std::vector<nbvInspection::Node<StateVec>*>::reverse_iterator iter = states_reverse.rbegin(); iter != states_reverse.rend(); iter++){
    states.push_back(*iter);
  }

  VRRT_SmoothPath(states);

  for(std::vector<nbvInspection::Node<StateVec>*>::iterator iter = states.begin(); iter != states.end()-1; iter++){
    ret = samplePath((*iter)->state_, (*(iter+1))->state_, targetFrame, ret);
    history_.push((*iter)->state_);
  }
  return ret;
}

void nbvInspection::RrtTree::VRRT_SmoothPath(std::vector<nbvInspection::Node<StateVec>*>& states) {

  int span = 2;
  while (span < states.size()) {
    bool changed = false;
    for (int i = 0; i + span < states.size(); i++) {
      Eigen::Vector3d start(states[i]->state_[0], states[i]->state_[1], states[i]->state_[2]);
      Eigen::Vector3d end(states[i + span]->state_[0], states[i + span]->state_[1], states[i + span]->state_[2]);
      if (volumetric_mapping::OctomapManager::CellStatus::kFree == manager_->getLineStatusBoundingBox(start, end, params_.boundingBox_))
      {
        for (int x = 1; x < span; x++) {
          states.erase(states.begin() + i + 1);
        }
        changed = true;
      }

    }
    if (!changed) span++;
  }
}

void nbvInspection::RrtTree::VRRT_initialize()
{
// This function is to initialize the tree, including insertion of remainder of previous best branch.
  g_ID_ = 0;
// Remove last segment from segment list (multi agent only)
  int i;
  for (i = 0; i < agentNames_.size(); i++) {
    if (agentNames_[i].compare(params_.navigationFrame_) == 0) {
      break;
    }
  }
  if (i < agentNames_.size()) {
    segments_[i]->clear();
  }
// Initialize kd-tree with root node and prepare log file
  kdTree_ = kd_create(4);

  if (params_.log_) {
    if (fileTree_.is_open()) {
      fileTree_.close();
    }
    fileTree_.open((logFilePath_ + "tree" + std::to_string(iterationCount_) + ".txt").c_str(),
                   std::ios::out);
  }

  rootNode_ = new Node<StateVec>;
  rootNode_->distance_ = 0.0;
  rootNode_->gain_ = params_.zero_gain_;
  rootNode_->parent_ = NULL;
  rootNode_->isLeaf = false;
  rootNode_->dirNum_ = 0;

  std::vector<Node<StateVec> *> v;
  rootNode_->allNode = v;
  rootNode_->leafNode = v;
  rootNode_->allNode.push_back(rootNode_);

  if (params_.exact_root_) {
    if (iterationCount_ <= 1) {
      exact_root_ = root_;
    }
    rootNode_->state_ = exact_root_;
  } else {
    rootNode_->state_ = root_;
  }

  double buf[4];
  for (int i=0; i<4; i++) {
    buf[i]=rootNode_->state_[i];
  }
  kd_insert(kdTree_, buf, rootNode_);

  iterationCount_++;

// Publish visualization of total exploration area
  visualization_msgs::Marker p;
  p.header.stamp = ros::Time::now();
  p.header.seq = 0;
  p.header.frame_id = params_.navigationFrame_;
  p.id = 0;
  p.ns = "workspace";
  p.type = visualization_msgs::Marker::CUBE;
  p.action = visualization_msgs::Marker::ADD;
  p.pose.position.x = 0.5 * (params_.minX_ + params_.maxX_);
  p.pose.position.y = 0.5 * (params_.minY_ + params_.maxY_);
  p.pose.position.z = 0.5 * (params_.minZ_ + params_.maxZ_);
  tf::Quaternion quat;
  quat.setEuler(0.0, 0.0, 0.0);
  p.pose.orientation.x = quat.x();
  p.pose.orientation.y = quat.y();
  p.pose.orientation.z = quat.z();
  p.pose.orientation.w = quat.w();
  p.scale.x = params_.maxX_ - params_.minX_;
  p.scale.y = params_.maxY_ - params_.minY_;
  p.scale.z = params_.maxZ_ - params_.minZ_;
  p.color.r = 200.0 / 255.0;
  p.color.g = 100.0 / 255.0;
  p.color.b = 0.0;
  p.color.a = 0.1;
  p.lifetime = ros::Duration(0.0);
  p.frame_locked = false;
  params_.inspectionPath_.publish(p);
}

Eigen::Vector4d nbvInspection::RrtTree::getRoot(){
  Eigen::Vector4d ret;

  if(rootNode_!=NULL) {
    ret[0] = rootNode_->state_[0];
    ret[1] = rootNode_->state_[1];
    ret[2] = rootNode_->state_[2];
    ret[3] = rootNode_->state_[3];
  } else {
    ret[0] = 0;
    ret[1] = 0;
    ret[2] = 0;
    ret[3] = 0;
  }
  return ret;
}

Eigen::Vector4d nbvInspection::RrtTree::getBest() {
  Eigen::Vector4d ret;

  if (bestNode_!=NULL) {
    ret[0] = bestNode_->state_[0];
    ret[1] = bestNode_->state_[1];
    ret[2] = bestNode_->state_[2];
    ret[3] = bestNode_->state_[3];
  } else {
    ret[0] = 0;
    ret[1] = 0;
    ret[2] = 0;
    ret[3] = 0;
  }
  return ret;
}

std::vector<tf::Vector3> nbvInspection::RrtTree::peer_vehicles_ = { };
#endif
