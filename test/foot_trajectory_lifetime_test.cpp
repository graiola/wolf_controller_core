// SPDX-License-Identifier: MIT
#include <wolf_controller_core/wpg/foot_trajectory_ellipse.h>

#include <iostream>
#include <memory>
#include <stdexcept>

namespace {
class OwnedEllipse final : public wolf_controller::Ellipse {
 public:
  explicit OwnedEllipse(std::shared_ptr<int> resource) : resource_(std::move(resource)) {}

 private:
  std::shared_ptr<int> resource_;
};
}

int main() {
  auto resource = std::make_shared<int>(42);
  std::weak_ptr<int> lifetime = resource;
  // GaitGenerator's factory returns a base pointer before the shared_ptr takes
  // ownership. The deleter must destroy the complete concrete trajectory.
  wolf_controller::TrajectoryInterface* trajectory = new OwnedEllipse(std::move(resource));
  std::shared_ptr<wolf_controller::TrajectoryInterface> owner(trajectory);
  if (lifetime.expired()) throw std::runtime_error("Trajectory lost its owned resource");
  owner.reset();
  if (!lifetime.expired()) throw std::runtime_error("Deleting a base trajectory skipped derived cleanup");
  std::cout << "Polymorphic trajectory cleanup passed\n";
}
