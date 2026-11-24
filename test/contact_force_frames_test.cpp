#include <wolf_controller_core/contact_force_frames.h>

#include <cmath>
#include <iostream>
#include <stdexcept>

int main() {
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  pose.translation() << 42.0, -17.0, 0.5;
  const Eigen::Vector3d sensor_force(2.0, 3.0, 40.0);
  if (!wolf_controller::contactForceInWorld(pose, sensor_force).isApprox(sensor_force))
    throw std::runtime_error("Contact translation changed measured force");
  pose.linear() = Eigen::AngleAxisd(std::acos(-1.0) / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d expected(-3.0, 2.0, 40.0);
  for (int tick = 0; tick < 10; ++tick) {
    if (!wolf_controller::contactForceInWorld(pose, sensor_force).isApprox(expected))
      throw std::runtime_error("World force does not match the sensor frame rotation");
  }
  std::cout << "Contact force frame tests passed\n";
}
