#ifndef AIRCRAFTMOTION_H

#include <array>
#include <cmath>

// Calculate the aircraft's state vector
inline std::array<double, 12>
aircraft_state(double roll, double pitch, double yaw, double forward_vel,
               double lateral_vel, double downward_vel,
               std::array<double, 3> force_vector,
               std::array<double, 3> moment_vector,
               std::array<std::array<double, 3>, 3> inertia_tensor) {

  std::array<double, 12> state_vector{0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  // inertial x velocity
  state_vector[0] =
      forward_vel * cos(pitch) * cos(yaw) +
      lateral_vel * (sin(roll) * sin(pitch) * cos(yaw) - cos(roll) * sin(yaw)) +
      downward_vel * (cos(roll) * sin(pitch) * cos(yaw) - sin(roll) * sin(yaw));
  // inertial y velocity
  state_vector[1] =
      forward_vel * cos(pitch) * sin(yaw) +
      lateral_vel * (sin(roll) * sin(pitch) * sin(yaw) + cos(roll) * cos(yaw)) +
      downward_vel * (cos(roll) * sin(pitch) * sin(yaw) - sin(roll) * cos(yaw));
  // inertial z velocity
  state_vector[2] = -forward_vel * sin(pitch) +
                    lateral_vel * sin(roll) * cos(pitch) +
                    downward_vel * cos(roll) * cos(pitch);
  // roll angle angular velocity
  // pitch angle angular velocity
  // yaw angle angular velocity

  return state_vector;
}

#endif // !
