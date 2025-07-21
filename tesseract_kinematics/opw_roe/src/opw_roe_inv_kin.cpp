/**
 * @file opw_inv_kin.cpp
 * @brief Tesseract OPW Inverse kinematics implementation.
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <stdexcept>
#include <console_bridge/console.h>
#include <opw_kinematics/opw_kinematics.h>
#include <opw_kinematics/opw_utilities.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/opw_roe/opw_roe_inv_kin.h>
#include <tesseract_kinematics/core/utils.h>

namespace tesseract_kinematics
{
OPWRoeInvKin::OPWRoeInvKin(opw_kinematics::Parameters<double> params,
                           std::string base_link_name,
                           std::string tip_link_name,
                           std::vector<std::string> joint_names,
                           Eigen::Isometry3d& base_link_transform,
                           double extender_min,
                           double extender_max,
                           double extender_sampling_min_deviation,
                           double extender_sampling_max_deviation,
                           double extender_step,
                           std::string solver_name)
  : params_(params)
  , base_link_name_(std::move(base_link_name))
  , tip_link_name_(std::move(tip_link_name))
  , joint_names_(std::move(joint_names))
  , base_link_transform_(std::move(base_link_transform))
  , extender_min_(extender_min)
  , extender_max_(extender_max)
  , extender_sampling_min_deviation_(extender_sampling_min_deviation)
  , extender_sampling_max_deviation_(extender_sampling_max_deviation)
  , extender_step_(extender_step)
  , solver_name_(std::move(solver_name))
{
  if (joint_names_.size() != 7)
    throw std::runtime_error("OPWRoeInvKin, only supports seven joints!");
}

InverseKinematics::UPtr OPWRoeInvKin::clone() const { return std::make_unique<OPWRoeInvKin>(*this); }

OPWRoeInvKin::OPWRoeInvKin(const OPWRoeInvKin& other) { *this = other; }

OPWRoeInvKin& OPWRoeInvKin::operator=(const OPWRoeInvKin& other)
{
  base_link_name_ = other.base_link_name_;
  tip_link_name_ = other.tip_link_name_;
  joint_names_ = other.joint_names_;
  params_ = other.params_;
  solver_name_ = other.solver_name_;
  return *this;
}

IKSolutions OPWRoeInvKin::calcInvKin(const tesseract_common::TransformMap& tip_link_poses,
                                     const Eigen::Ref<const Eigen::VectorXd>& /*seed*/) const
{
  assert(tip_link_poses.size() == 1);                                                       // NOLINT
  assert(tip_link_poses.find(tip_link_name_) != tip_link_poses.end());                      // NOLINT
  assert(std::abs(1.0 - tip_link_poses.at(tip_link_name_).matrix().determinant()) < 1e-6);  // NOLINT

  // Fill up the extender samples
  std::vector<double> extender_samples;
  extender_samples.reserve(
      static_cast<std::size_t>((extender_sampling_max_deviation_ - extender_sampling_min_deviation_) / extender_step_));
  double value = tip_link_poses.at(tip_link_name_).translation().y() + extender_sampling_min_deviation_;
  while (value < tip_link_poses.at(tip_link_name_).translation().y() + extender_sampling_max_deviation_)
  {
    if (value >= extender_min_ && value <= extender_max_)
    {
      extender_samples.push_back(value);
    }

    value += extender_step_;
  }

  // Initialize the solution set
  IKSolutions solution_set;
  solution_set.reserve(8 * extender_samples.size());  // 8 is the maximum number of solutions for OPW

  // Default base link transform
  Eigen::Isometry3d base_link_transform = base_link_transform_;

  // Declare the target pose
  Eigen::Isometry3d target_pose;

  for (const auto& extender_value : extender_samples)
  {
    base_link_transform.translation().y() = extender_value;
    target_pose = base_link_transform.inverse() *
                  tip_link_poses.at(tip_link_name_);  // Get the target pose relative to the base link, which gets moved
                                                      // according to the extender samples

    // NOLINTNEXTLINE
    opw_kinematics::Solutions<double> sols = opw_kinematics::inverse(params_, target_pose);

    // Check the output
    for (auto& sol : sols)
    {
      if (opw_kinematics::isValid<double>(sol))
      {
        // Add the extender value in front of the output to get the full joint solution
        Eigen::VectorXd solution(7);
        solution << extender_value, Eigen::Map<Eigen::VectorXd>(sol.data(), static_cast<Eigen::Index>(sol.size()));
        solution_set.emplace_back(solution);
      }
    }
  }

  return solution_set;
}

Eigen::Index OPWRoeInvKin::numJoints() const { return 7; }

std::vector<std::string> OPWRoeInvKin::getJointNames() const { return joint_names_; }
std::string OPWRoeInvKin::getBaseLinkName() const { return base_link_name_; }
std::string OPWRoeInvKin::getWorkingFrame() const { return base_link_name_; }
std::vector<std::string> OPWRoeInvKin::getTipLinkNames() const { return { tip_link_name_ }; }
std::string OPWRoeInvKin::getSolverName() const { return solver_name_; }

}  // namespace tesseract_kinematics
