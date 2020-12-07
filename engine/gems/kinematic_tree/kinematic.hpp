/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>

#include "engine/core/math/dual_quaternion.hpp"
#include "engine/core/math/pose3.hpp"
#include "engine/core/math/types.hpp"
#include "engine/core/optional.hpp"
#include "engine/gems/kinematic_tree/kinematic_tree.hpp"

namespace isaac {
namespace kinematic_tree {

// Returns the dual quaternion representing the transformation from the root to the output of the
// given link specified by name.
DualQuaternionD ForwardKinematic(const KinematicTree& tree, const VectorXd& state,
                                 const std::string& link_name);
// Returns the dual quaternion representing the transformation from the root to the output of the
// given link specified by id.
DualQuaternionD ForwardKinematic(const KinematicTree& tree, const VectorXd& state,
                                 KinematicTree::LinkId link_id);

// Returns the Jacobian associated to the forward kinematic function above.
Matrix8Xd ForwardKinematicJacobian(const KinematicTree& tree, const VectorXd& state,
                                   const std::string& link_name);

// Returns the Jacobian associated to the forward kinematic function above.
Matrix8Xd ForwardKinematicJacobian(const KinematicTree& tree, const VectorXd& state,
                                   KinematicTree::LinkId link_id);

// Returns whether or not the current state is valid (checking for joints limit)
// TODO(ben): Make it also check for self collision.
bool IsValidState(const KinematicTree& tree, const VectorXd& state);

// Returns a valid state that move the given link of a kinematic tree to a given Position.
// In case of failure it will returns nullopt.
std::optional<VectorXd> InverseKinematic(const KinematicTree& tree,
                                         const DualQuaternionD& root_T_joint,
                                         const std::string& joint_name,
                                         std::optional<VectorXd> starting_state = std::nullopt);
// Same as above but using a custom joint specified by id
std::optional<VectorXd> InverseKinematic(const KinematicTree& tree,
                                         const DualQuaternionD& root_T_joint,
                                         KinematicTree::LinkId joint_id,
                                         std::optional<VectorXd> starting_state = std::nullopt);

}  // namespace kinematic_tree
}  // namespace isaac
