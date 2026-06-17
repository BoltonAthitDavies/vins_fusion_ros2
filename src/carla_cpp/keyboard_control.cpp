#include "keyboard_control.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <cstdio>

namespace carla_cpp {

namespace {
constexpr float kThrottleStep = 0.10f;
constexpr float kThrottleDecay = 0.20f;
constexpr float kBrakeOn = 0.6f;
constexpr float kSteerStep = 0.10f;
constexpr float kSteerDecay = 0.7f;
constexpr float kSteerMax = 0.7f;

float clampf(float v, float lo, float hi) { return std::max(lo, std::min(hi, v)); }
}  // namespace

KeyboardControl::KeyboardControl() { interactive_ = isatty(STDIN_FILENO) != 0; }

KeyboardControl::~KeyboardControl() { stop(); }

void KeyboardControl::start() {
  if (!interactive_) {
    printf(
        "[carla_cpp] stdin is not a TTY: keyboard disabled. Start with "
        "autopilot or run from an interactive terminal.\n");
    return;
  }
  if (tcgetattr(STDIN_FILENO, &old_termios_) != 0) {
    interactive_ = false;
    return;
  }
  struct termios raw = old_termios_;
  raw.c_lflag &= ~(ICANON | ECHO);  // non-canonical, no echo
  raw.c_cc[VMIN] = 0;
  raw.c_cc[VTIME] = 0;
  tcsetattr(STDIN_FILENO, TCSANOW, &raw);
  int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
  raw_active_ = true;
  printf(
      "[carla_cpp] keyboard: W/S throttle/brake  A/D steer  Q reverse  "
      "Space handbrake  P autopilot  C coverage(right-loop)  T trajectory(MPC)  X quit\n");
}

void KeyboardControl::stop() {
  if (raw_active_) {
    tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
    raw_active_ = false;
  }
}

bool KeyboardControl::consumeAutopilotToggle() {
  bool t = toggle_pending_;
  toggle_pending_ = false;
  return t;
}

bool KeyboardControl::consumeCoverageToggle() {
  bool t = coverage_toggle_pending_;
  coverage_toggle_pending_ = false;
  return t;
}

bool KeyboardControl::consumeTrajectoryToggle() {
  bool t = trajectory_toggle_pending_;
  trajectory_toggle_pending_ = false;
  return t;
}

void KeyboardControl::poll() {
  if (!interactive_) return;

  bool w = false, s = false, a = false, d = false;
  char c;
  while (read(STDIN_FILENO, &c, 1) == 1) {
    switch (c) {
      case 'w': case 'W': w = true; break;
      case 's': case 'S': s = true; break;
      case 'a': case 'A': a = true; break;
      case 'd': case 'D': d = true; break;
      case 'q': case 'Q': cmd_.reverse = !cmd_.reverse; break;
      case ' ': cmd_.hand_brake = !cmd_.hand_brake; break;
      case 'p': case 'P':
        autopilot_ = !autopilot_;
        toggle_pending_ = true;
        if (autopilot_) { coverage_ = false; trajectory_ = false; }  // mutually exclusive
        break;
      case 'c': case 'C':
        coverage_ = !coverage_;
        coverage_toggle_pending_ = true;
        if (coverage_) { autopilot_ = false; trajectory_ = false; }
        break;
      case 't': case 'T':
        trajectory_ = !trajectory_;
        trajectory_toggle_pending_ = true;
        if (trajectory_) { autopilot_ = false; coverage_ = false; }
        break;
      case 'x': case 'X': case 27: quit_ = true; break;
      default: break;
    }
  }

  // Throttle: ramp while held, decay otherwise.
  cmd_.throttle = w ? clampf(cmd_.throttle + kThrottleStep, 0.f, 1.f)
                    : clampf(cmd_.throttle - kThrottleDecay, 0.f, 1.f);
  // Brake: simple on/off (releases immediately).
  cmd_.brake = s ? kBrakeOn : 0.f;
  // Steering: accumulate then decay toward center when neither key held.
  if (a) steer_cache_ -= kSteerStep;
  if (d) steer_cache_ += kSteerStep;
  if (!a && !d) steer_cache_ *= kSteerDecay;
  steer_cache_ = clampf(steer_cache_, -kSteerMax, kSteerMax);
  cmd_.steer = steer_cache_;
}

}  // namespace carla_cpp
