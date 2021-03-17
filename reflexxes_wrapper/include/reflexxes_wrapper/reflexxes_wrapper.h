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

#include <iostream>
#include <libreflexxestype2/ReflexxesAPI.h>
#include <vector>

namespace reflexxes_wrapper
{
/** \brief Set velocity and acceleration limits. */
inline bool setLimits(std::unique_ptr<RMLPositionInputParameters>& reflexxes_params, const size_t num_dof, const std::vector<double>& max_velocities, const std::vector<double>& max_accelerations)
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
  }

  return true;
}

/** \brief Set all entries in the selection vector true, to enable all joints. */
inline void setSelectionVectorAllTrue(std::unique_ptr<RMLPositionInputParameters>& reflexxes_params, const size_t num_dof)
{
  for (size_t joint_idx = 0; joint_idx < num_dof; ++joint_idx)
  {
    reflexxes_params->SelectionVector->VecData[joint_idx] = true;
  }
}

}  // namespace reflexxes_wrapper
