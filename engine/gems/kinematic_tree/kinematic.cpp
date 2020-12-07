/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "kinematic.hpp"

#include <string>
#include <utility>
#include <vector>

#include "engine/core/assert.hpp"
#include "engine/core/math/dual_quaternion.hpp"
#include "engine/core/math/pose3.hpp"
#include "engine/core/math/types.hpp"
#include "engine/gems/kinematic_tree/kinematic_tree.hpp"
#include "engine/gems/optimization/gradient_descent.hpp"

namespace isaac {
namespace kinematic_tree {

namespace {
// Implementation of the ForwardKinematicJacobian unctions with the kinematic chain already
// extracted.
Matrix8Xd ForwardKinematicJacobianImpl(
    const KinematicTree& tree, const VectorXd& state,
    const std::vector<const KinematicTree::Link*>& kinematic_chain) {
  Matrix8Xd jac = Matrix8Xd::Zero(8, state.size());
  // Compute the comulative left product of all the transformation (it will be used to speed
  // up computation of q_1 * q_2 * .. * q'_k * ... * q_n)
  std::vector<DualQuaternionD> left_compositions = {DualQuaternionD::Identity()};
  for (size_t id = 0; id < kinematic_chain.size(); id++) {
    left_compositions.push_back(
        left_compositions.back() *
        kinematic_chain[id]->motor->transformation(
            tree.getMotorStateView(state, kinematic_chain[id]->id)));
  }
  // Populate each column of the jocabian one by one.
  for (size_t id = 0; id < kinematic_chain.size(); id++) {
    // Skip motor without argument as they do not contribute to the jacobian directly.
    if (kinematic_chain[id]->motor->getNumberOfArguments() == 0) continue;
    const auto jacobian = kinematic_chain[id]->motor->jacobian(
        tree.getMotorStateView(state, kinematic_chain[id]->id));
    const int dimension = tree.getLinkOffset(kinematic_chain[id]->id);
    // Jacobian for a given parameter alpha_k is:
    //  q_1 * q_2 * ... q_(k-1) * q'_k * q_(k+1) * ... * q_n
    // The left composition up to q_(k-1) is already computed and stored in left_compositions
    const DualQuaternionD& left_composition = left_compositions[id];
    // The right composition (q_(k+1) to the end) can be computed using the precomputed left
    // composition by simply removing the left part before k+1 using the inverse.
    const DualQuaternionD right_composition =
        left_compositions[id+1].inverse() * left_compositions.back();
    jac.block(0, dimension, 8, kinematic_chain[id]->motor->getNumberOfArguments()) =
        left_composition.productMatrixLeft() * right_composition.productMatrixRight() * jacobian;
  }
  return jac;
}
}  // namespace

DualQuaternionD ForwardKinematic(const KinematicTree& tree, const VectorXd& state,
                                 const std::string& link_name) {
  auto* link = tree.tryGetLink(link_name);
  ASSERT(link != nullptr, "`%s` is not a valid link name", link_name.c_str());
  return ForwardKinematic(tree, state, link->id);
}

DualQuaternionD ForwardKinematic(const KinematicTree& tree, const VectorXd& state,
                                 KinematicTree::LinkId link_id) {
  const int num_dimensions = tree.getNumberOfLinks();
  ASSERT(link_id < num_dimensions, "Invalid link id: %d >= %d", link_id, num_dimensions);
  const int state_dimensions = tree.getMotorStateDimensions();
  ASSERT(state.size() == state_dimensions, "Invalid state dimension: %d != %d",
         state.size(), state_dimensions);
  auto* link = tree.tryGetLink(link_id);
  ASSERT(link != nullptr, "The link with id %zu does not exist", link_id);

  DualQuaternionD out = DualQuaternionD::Identity();
  while (link != nullptr) {
    if (link->motor) {
      out = link->motor->transformation(tree.getMotorStateView(state, link->id)) * out;
    }
    link = link->parent;
  }
  return out;
}

Matrix8Xd ForwardKinematicJacobian(const KinematicTree& tree, const VectorXd& state,
                                   const std::string& link_name) {
  auto* link = tree.tryGetLink(link_name);
  ASSERT(link != nullptr, "`%s` is not a valid link name", link_name.c_str());
  return ForwardKinematicJacobian(tree, state, link->id);
}

// Returns the Jacobian associated to the forward kinematic function above.
Matrix8Xd ForwardKinematicJacobian(const KinematicTree& tree, const VectorXd& state,
                                   KinematicTree::LinkId link_id) {
  auto* link = tree.tryGetLink(link_id);
  ASSERT(link != nullptr, "The link with id %zu does not exist", link_id);
  std::vector<const KinematicTree::Link*> kinematic_chain;
  while (link != nullptr) {
    if (link->motor) {
      kinematic_chain.push_back(link);
    }
    link = link->parent;
  }
  std::reverse(kinematic_chain.begin(), kinematic_chain.end());
  return ForwardKinematicJacobianImpl(tree, state, kinematic_chain);
}

bool IsValidState(const KinematicTree& tree, const VectorXd& state) {
  for (int idx = 0; idx < tree.getNumberOfLinks(); idx++) {
    auto* link = tree.tryGetLink(idx);
    if (link->motor) {
      if (!link->motor->isValidState(tree.getMotorStateView(state, link->id))) {
        return false;
      }
    }
  }
  return true;
}

std::optional<VectorXd> InverseKinematic(const KinematicTree& tree,
                                         const DualQuaternionD& root_T_link,
                                         const std::string& link_name,
                                         std::optional<VectorXd> starting_state) {
  auto* link = tree.tryGetLink(link_name);
  ASSERT(link != nullptr, "`%s` is not a valid link name", link_name.c_str());
  return InverseKinematic(tree, root_T_link, link->id, starting_state);
}

std::optional<VectorXd> InverseKinematic(const KinematicTree& tree,
                                         const DualQuaternionD& root_T_link,
                                         KinematicTree::LinkId link_id,
                                         std::optional<VectorXd> starting_state) {
  const int num_dimensions = tree.getNumberOfLinks();
  ASSERT(link_id < num_dimensions, "Invalid link id: %d >= %d", link_id, num_dimensions);
  auto* link = tree.tryGetLink(link_id);
  ASSERT(link != nullptr, "The link with id %zu does not exist", link_id);
  const int state_dimensions = tree.getMotorStateDimensions();
  // Generate a starting position
  VectorXd out;
  VectorXd lower_range(state_dimensions);
  VectorXd upper_range(state_dimensions);
  if (starting_state) {
    ASSERT(starting_state->size() == state_dimensions,
           "Starting state does not have the right dimensions: %d vs %d",
           starting_state->size(), state_dimensions);
    out = *starting_state;
  } else {
    out = VectorXd::Random(state_dimensions);
  }

  // Extracts all the link and order them from root to end effector
  std::vector<const KinematicTree::Link*> chain;
  while (link != nullptr) {
    if (link->motor) {
      chain.push_back(link);
      // Add some margin to make sure we don't exit the validity range.
      const int num_arguments = link->motor->getNumberOfArguments();
      const int offset = tree.getLinkOffset(link->id);
      const VectorXd kMargin = VectorXd::Constant(num_arguments, 0.01);
      lower_range.block(offset, 0, num_arguments, 1) = link->motor->getLowerRange() + kMargin;
      upper_range.block(offset, 0, num_arguments, 1) = link->motor->getUpperRange() - kMargin;
    }
    link = link->parent;
  }
  std::reverse(chain.begin(), chain.end());

  // Helper function that the difference between two quaterions and store it as a VectorXd
  auto difference = [&](const DualQuaternionD& dq1, const DualQuaternionD& dq2) {
    return (dq1 - dq2).coeffs();
  };

  // Weight of the error relative to the range limit.
  constexpr double kWeight = 10.0;
  // Function that computes the error of a given state for the gradient descent.
  auto value_f = [&](const VectorXd& state) {
    double error = difference(ForwardKinematic(tree, state, link_id), root_T_link).squaredNorm();
    for (int i = 0; i < state_dimensions; i++) {
      if (state[i] < lower_range[i]) {
        error += kWeight * std::pow(state[i] - lower_range[i], 2);
      }
      if (state[i] > upper_range[i]) {
        error += kWeight * std::pow(state[i] - upper_range[i], 2);
      }
    }
    return 0.5 * error;
  };
  // Function that computes the error and the gradient of a given state for the gradient descent.
  auto value_and_gradient_f = [&](const VectorXd& state) {
    const VectorXd diff = difference(ForwardKinematic(tree, state, link_id), root_T_link);
    VectorXd derr = ForwardKinematicJacobianImpl(tree, state, chain).transpose() * diff;
    double error = diff.squaredNorm();
    for (int i = 0; i < state_dimensions; i++) {
      if (state[i] < lower_range[i]) {
        error += kWeight * std::pow(state[i] - lower_range[i], 2);
        derr[i] += kWeight * (state[i] - lower_range[i]);
      }
      if (state[i] > upper_range[i]) {
        error += kWeight * std::pow(state[i] - upper_range[i], 2);
        derr[i] += kWeight * (state[i] - upper_range[i]);
      }
    }
    return std::pair<double, VectorXd> {0.5 * error, derr};
    };
  // Function that updates a state.
  auto update_f = [&](const VectorXd& state, const VectorXd& dstate) {
    return state + dstate;
  };

  constexpr unsigned kMaxIterations = 1000;
  constexpr double kGradientTolerance = 1e-6;
  constexpr double kLineSearchStepFactor = 0.5;
  constexpr double kLineSearchArmijoFactor = 0.5;
  constexpr unsigned kLineSearchMaxIteration = 30;
  constexpr bool kBreakOnFailedLineSearch = true;

  VectorXd state;
  optimization::GradientDescentInfo info;
  std::tie(state, info) = optimization::GradientDescent(
      out, value_f, value_and_gradient_f, update_f,
      {kMaxIterations, kGradientTolerance, kLineSearchStepFactor, kLineSearchArmijoFactor,
       kLineSearchMaxIteration, kBreakOnFailedLineSearch});
  if (!info.converged || !IsValidState(tree, state)) return std::nullopt;
  return state;
}

}  // namespace kinematic_tree
}  // namespace isaac
