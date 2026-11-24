/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) Gennaro Raiola
 */
#pragma once

#include <Eigen/Geometry>

namespace wolf_controller {

// Force is a free vector. The translation in a contact pose only applies to
// points, and must never be added to the force measurement.
inline Eigen::Vector3d contactForceInWorld(const Eigen::Affine3d& world_T_contact,
                                         const Eigen::Vector3d& local_force) {
  return world_T_contact.linear() * local_force;
}

}  // namespace wolf_controller
