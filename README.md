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

## Setup

For installation and workspace setup, see [wolf-setup](https://github.com/graiola/wolf-setup).
