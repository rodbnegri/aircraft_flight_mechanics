/*
GNU General Public License with Academic Attribution
Copyright (C) 2024 Rodolfo Batista Negri

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

!!!!!!!!!!!!!!~~~ Additional Terms for Academic Use: ~~!!!!!!!!!!!!!!!!!!

If this software is used in academic papers or publications, the authors
are required to mention the original authorship in the text of the paper
or publication, followed by the repository's URL.

Example, suppose Software X was used for data analysis:
"The data analysis was performed using Software X, developed by
Dr. Rodolfo B. Negri~\footnote{[URL]}."
*/

#ifndef AIRCRAFTMOTION_HPP

// #include "framesnrotations.h"
#include <array>
#include <cmath>

// Calculate the aircraft's equations of motion
inline std::array<double, 12>
aircrafts_EOM(double earth_pos_x, double earth_pos_y, double earth_pos_z,
              double roll, double pitch, double yaw, double forward_vel,
              double lateral_vel, double downward_vel, double forward_ang_vel,
              double lateral_ang_vel, double downward_ang_vel, double mass,
              std::array<double, 3> force_vector,
              std::array<double, 3> moment_vector,
              std::array<std::array<double, 3>, 3> inertia_tensor) {

  std::array<double, 12> state_vector{0.0};

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
  state_vector[3] = forward_ang_vel + lateral_ang_vel * sin(roll) * tan(pitch) +
                    downward_ang_vel * cos(roll) * tan(pitch);
  // pitch angle angular velocity
  state_vector[4] = lateral_ang_vel * cos(pitch) - downward_ang_vel * sin(roll);
  // yaw angle angular velocity
  state_vector[5] =
      1.0 / cos(pitch) *
      (lateral_ang_vel * sin(pitch) + downward_ang_vel * cos(roll));
  // forward acceleration
  state_vector[6] = downward_ang_vel * lateral_vel -
                    lateral_ang_vel * downward_vel + force_vector[0] / mass;
  // lateral acceleration
  state_vector[7] = forward_ang_vel * downward_vel -
                    downward_ang_vel * forward_vel + force_vector[1] / mass;
  // downward acceleration
  state_vector[8] = lateral_ang_vel * forward_vel -
                    forward_ang_vel * lateral_vel + force_vector[2] / mass;
  //  elements of the inertia tensor
  double Ixz{-inertia_tensor[0][2]};
  // double Izx{-inertia_tensor[3][1]};
  double Ixx{inertia_tensor[0][0]};
  double Iyy{inertia_tensor[1][1]};
  double Izz{inertia_tensor[2][2]};
  // forward angular velocity
  state_vector[9] =
      (Ixz * moment_vector[2] + Izz * moment_vector[0] +
       Ixz * (Ixx - Iyy + Izz) * forward_ang_vel * lateral_ang_vel -
       (pow(Ixz, 2) - Iyy * Izz + pow(Izz, 2)) * lateral_ang_vel *
           downward_ang_vel) /
      (Ixx * Izz - pow(Ixz, 2));
  // lateral angular velocity
  state_vector[10] =
      (moment_vector[1] - (Ixx - Izz) * downward_ang_vel * forward_ang_vel -
       Ixz * (pow(forward_ang_vel, 2) - pow(downward_ang_vel, 2))) /
      Iyy;
  // downward angular velocity
  state_vector[11] =
      (Ixx * moment_vector[2] + Ixz * moment_vector[0] -
       Ixz * (Ixx - Iyy + Izz) * downward_ang_vel * lateral_ang_vel +
       (pow(Ixz, 2) - Iyy * Ixx + pow(Ixx, 2)) * lateral_ang_vel *
           forward_ang_vel) /
      (Ixx * Izz - pow(Ixz, 2));

  return state_vector;
}

#endif // !
