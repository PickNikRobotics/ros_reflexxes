/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, PickNik Inc
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Inc nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Andy Zelenak
   Desc: Convenience functions for interacting with Reflexxes
*/

#pragma once

#include <float.h>
#include <iostream>
#include <libreflexxestype2/ReflexxesAPI.h>
#include <vector>

namespace reflexxes_wrapper
{

/** \brief Set the current state (position, velocity, and acceleration). */
inline bool setCurrentState(RMLInputParameters* reflexxes_params, const size_t num_dof, const std::vector<double>& current_positions, const std::vector<double>& current_velocities, const std::vector<double>& current_accelerations)
{
  if (current_positions.size() != current_velocities.size() || current_velocities.size() != current_accelerations.size() || current_velocities.size() != num_dof)
  {
    std::cout << "An input vector does not match the degrees of freedom." << std::endl;
    return false;
  }

  for (size_t joint_idx = 0; joint_idx < num_dof; ++joint_idx)
  {
    reflexxes_params->CurrentPositionVector->VecData[joint_idx] = current_positions.at(joint_idx);
    reflexxes_params->CurrentVelocityVector->VecData[joint_idx] = current_velocities.at(joint_idx);
    reflexxes_params->CurrentAccelerationVector->VecData[joint_idx] = current_accelerations.at(joint_idx);
  }

  return true;
}

/** \brief Reset the output struct. This might be useful if Reflexxes output is fed in as input for the next iteraion, as the Reflexxes examples do.  */
inline bool resetOutputStruct(RMLOutputParameters* reflexxes_params, const size_t num_dof, const std::vector<double>& current_positions, const std::vector<double>& current_velocities, const std::vector<double>& current_accelerations)
{
  if (current_positions.size() != num_dof)
  {
    std::cout << "An input vector does not match the degrees of freedom." << std::endl;
    return false;
  }

  for (size_t joint_idx = 0; joint_idx < num_dof; ++joint_idx)
  {
    reflexxes_params->NewPositionVector->VecData[joint_idx] = current_positions.at(joint_idx);
    reflexxes_params->NewVelocityVector->VecData[joint_idx] = current_velocities.at(joint_idx);
    reflexxes_params->NewAccelerationVector->VecData[joint_idx] = current_accelerations.at(joint_idx);
  }

  return true;
}

/** \brief Set the current position only, do not change current velocity or acceleration */
inline bool setCurrentPositions(RMLInputParameters* reflexxes_params, const size_t num_dof, const std::vector<double>& current_positions)
{
  if (current_positions.size() != num_dof)
  {
    std::cout << "An input vector does not match the degrees of freedom." << std::endl;
    return false;
  }

  for (size_t joint_idx = 0; joint_idx < num_dof; ++joint_idx)
  {
    reflexxes_params->CurrentPositionVector->VecData[joint_idx] = current_positions.at(joint_idx);
  }

  return true;
}

/** \brief Set the target state (position and velocity). */
inline bool setTargetState(RMLPositionInputParameters* reflexxes_params, const size_t num_dof, const std::vector<double>& target_positions, const std::vector<double>& target_velocities)
{
  if (target_positions.size() != target_velocities.size() || target_velocities.size() != num_dof)
  {
    std::cout << "An input vector does not match the degrees of freedom." << std::endl;
    return false;
  }

  for (size_t joint_idx = 0; joint_idx < num_dof; ++joint_idx)
  {
    reflexxes_params->TargetPositionVector->VecData[joint_idx] = target_positions.at(joint_idx);
    reflexxes_params->TargetVelocityVector->VecData[joint_idx] = target_velocities.at(joint_idx);
  }

  return true;
}

/** \brief Set the target state (position and velocity). */
inline bool setTargetState(RMLVelocityInputParameters* reflexxes_params, const size_t num_dof, const std::vector<double>& target_positions, const std::vector<double>& target_velocities)
{
  if (target_positions.size() != target_velocities.size() || target_velocities.size() != num_dof)
  {
    std::cout << "An input vector does not match the degrees of freedom." << std::endl;
    return false;
  }

  for (size_t joint_idx = 0; joint_idx < num_dof; ++joint_idx)
  {
    reflexxes_params->TargetVelocityVector->VecData[joint_idx] = target_velocities.at(joint_idx);
  }

  return true;
}

/** \brief Set velocity and acceleration limits. */
inline bool setLimits(RMLPositionInputParameters* reflexxes_params, const size_t num_dof, const std::vector<double>& max_velocities, const std::vector<double>& max_accelerations)
{
  if (max_velocities.size() != max_accelerations.size() || max_velocities.size() != num_dof)
  {
    std::cout << "An input vector does not match the degrees of freedom." << std::endl;
    return false;
  }

  for (size_t joint_idx = 0; joint_idx < num_dof; ++joint_idx)
  {
    reflexxes_params->MaxVelocityVector->VecData[joint_idx] = max_velocities.at(joint_idx);
    reflexxes_params->MaxAccelerationVector->VecData[joint_idx] = max_accelerations.at(joint_idx);
    reflexxes_params->MaxJerkVector->VecData[joint_idx] = DBL_MAX;
  }

  return true;
}

/** \brief Set velocity and acceleration limits. */
inline bool setLimits(RMLVelocityInputParameters* reflexxes_params, const size_t num_dof, const std::vector<double>& max_velocities, const std::vector<double>& max_accelerations)
{
  if (max_velocities.size() != max_accelerations.size() || max_velocities.size() != num_dof)
  {
    std::cout << "An input vector does not match the degrees of freedom." << std::endl;
    return false;
  }

  for (size_t joint_idx = 0; joint_idx < num_dof; ++joint_idx)
  {
    reflexxes_params->MaxAccelerationVector->VecData[joint_idx] = max_accelerations.at(joint_idx);
    reflexxes_params->MaxJerkVector->VecData[joint_idx] = DBL_MAX;
  }

  return true;
}

/** \brief Set velocity and acceleration limits. */
inline bool setScaledLimits(RMLPositionInputParameters* reflexxes_params,
                            const size_t num_dof,
                            double velocity_scale,
                            double acceleration_scale,
                            const std::vector<double>& max_velocities,
                            const std::vector<double>& max_accelerations)
{
  if (max_velocities.size() != max_accelerations.size() || max_velocities.size() != num_dof)
  {
    std::cout << "An input vector does not match the degrees of freedom." << std::endl;
    return false;
  }

  if (acceleration_scale < 0 || velocity_scale < 0)
  {
    std::cout << "Velocity and acceleration scales should be greater than zero" << std::endl;
    return false;
  }

  for (size_t joint_idx = 0; joint_idx < num_dof; ++joint_idx)
  {
    reflexxes_params->MaxVelocityVector->VecData[joint_idx] = velocity_scale * max_velocities.at(joint_idx);
    reflexxes_params->MaxAccelerationVector->VecData[joint_idx] = acceleration_scale * max_accelerations.at(joint_idx);
    reflexxes_params->MaxJerkVector->VecData[joint_idx] = DBL_MAX;
  }

  return true;
}

/** \brief Set all entries in the selection vector true, to enable all DOF. */
inline void setSelectionVectorAllTrue(RMLInputParameters* reflexxes_params, const size_t num_dof)
{
  for (size_t joint_idx = 0; joint_idx < num_dof; ++joint_idx)
  {
    reflexxes_params->SelectionVector->VecData[joint_idx] = true;
  }
}

/** \brief Set positions/velocities/accelerations to zero */
inline void initializePositionInputStateToZeros(RMLInputParameters* reflexxes_params, const size_t num_dof)
{
  for (size_t joint_idx = 0; joint_idx < num_dof; ++joint_idx)
  {
    reflexxes_params->CurrentPositionVector->VecData[joint_idx] = std::numeric_limits<double>::quiet_NaN();
    reflexxes_params->CurrentVelocityVector->VecData[joint_idx] = 0;
    reflexxes_params->CurrentAccelerationVector->VecData[joint_idx] = 0;
  }
}

}  // namespace reflexxes_wrapper
