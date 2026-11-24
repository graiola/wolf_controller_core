# wolf_controller_core

Core whole-body locomotion control algorithms used by WoLF.

For MPC observations, centroidal momentum uses world axes; contact forces are
rotated from their local sensor frame without adding the contact translation.
Raw external forces are retained separately to avoid rotating an old world
sample again between sensor updates. EXT contact diagnostics come from the
inverse-dynamics contact schedule rather than the WPG gait generator.

Current and requested control modes use atomic access. The ROS 2 MPC transport
also checks pending EXT requests through `getRequestedControlMode()`, so a
request waiting for all-foot contact can be canceled if its prepared references
expire before activation. Rebuild the controller and WBID together with this
core revision; the coordinated integration changes shared C++ interfaces.

The Eigen-only regression test `test/contact_force_frames_test.cpp` checks that
contact translation has no effect and that world rotation is applied correctly.

Swing trajectories are owned through `TrajectoryInterface` pointers. Its virtual
destructor ensures the concrete trajectory and its members are destroyed when
the gait generator releases or replaces them. The previous base deletion was
undefined behavior and was detected by AddressSanitizer during the real
controller migration test. `test/foot_trajectory_lifetime_test.cpp` checks derived
resource cleanup through the same base-pointer ownership path. It compiles the
actual trajectory sources in isolation; the full controller gate also exercises
the installed libraries. Rebuild `wpg` and all controller consumers together
after this virtual-interface change.

`setJointState` takes actuated-joint vectors only, after initialization. It
validates all four vector sizes before changing the internal tails and leaves
the six floating-base coordinates unchanged. The old loop counted generalized
coordinates and added the base offset a second time, writing beyond the state
vectors. The real controller regression uses the production indexed setters
for its live samples and separately checks this bulk API's dimensions and
state preservation.

On 2026-11-24 the native Humble SDK build and both core CTest regressions passed
(2/2, 0.01 seconds). This checks force-frame arithmetic and trajectory ownership,
not closed-loop robot stability or real-time deadlines.

## Setup

For installation and workspace setup, see [wolf-setup](https://github.com/graiola/wolf-setup).
