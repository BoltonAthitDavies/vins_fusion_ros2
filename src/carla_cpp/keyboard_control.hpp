/*******************************************************
 * carla_cpp: KeyboardControl
 *
 * Raw-mode (termios) terminal teleop, so it works headless / over SSH
 * unlike the pygame-based carla_manual_control. Because a terminal delivers
 * key *events* (not held state), throttle/steer ramp on repeat and decay on
 * absence to emulate "key held" — a minor feel difference from pygame.
 *
 *   W/S  throttle / brake     A/D  steer left / right
 *   Q    toggle reverse       Space toggle hand brake
 *   P    toggle autopilot     C    toggle right-loop coverage autopilot
 *   X / Esc  quit
 *
 * "Coverage" mirrors carla_manual_control's C mode: a Traffic-Manager route
 * (Right/Left/Left/Left) that drives a closed loop while ignoring lights/signs,
 * for repeatable SLAM loop-closure data collection.
 *
 * No CARLA / ROS / estimator headers.
 *******************************************************/
#pragma once

#include <termios.h>

#include "types.hpp"

namespace carla_cpp {

class KeyboardControl {
 public:
  KeyboardControl();
  ~KeyboardControl();

  void start();  // enter raw mode (no-op if stdin is not a tty)
  void stop();   // restore terminal

  // Drain pending keystrokes and update the command/state. Non-blocking.
  void poll();

  const ControlCmd &command() const { return cmd_; }
  bool autopilot() const { return autopilot_; }
  bool coverage() const { return coverage_; }
  bool trajectory() const { return trajectory_; }   // T mode (MPC /trajectory_cmd)
  // Each returns true exactly once after the corresponding key press, so the
  // caller flips the CARLA mode only on the transition. autopilot, coverage, and
  // trajectory are mutually exclusive (enabling one clears the others).
  bool consumeAutopilotToggle();
  bool consumeCoverageToggle();
  bool consumeTrajectoryToggle();
  bool quitRequested() const { return quit_; }
  bool interactive() const { return interactive_; }
  // True while no TM/MPC mode drives the car (manual control).
  bool manualDriving() const { return !autopilot_ && !coverage_ && !trajectory_; }

 private:
  bool interactive_ = false;
  bool raw_active_ = false;
  struct termios old_termios_;

  ControlCmd cmd_;
  float steer_cache_ = 0.0f;
  bool autopilot_ = false;
  bool coverage_ = false;
  bool trajectory_ = false;
  bool toggle_pending_ = false;
  bool coverage_toggle_pending_ = false;
  bool trajectory_toggle_pending_ = false;
  bool quit_ = false;
};

}  // namespace carla_cpp
