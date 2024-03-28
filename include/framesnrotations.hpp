/*
GNU General Public License with Academic Attribution

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
Rodolfo B. Negri~\footnote{[URL]}."
*/

#ifndef FRAMESNROTATIONS_HPP

#include <array>
#include <cmath>

// Function to perform rotation from the body-fixed frame (aircraft) to the
// Earth
inline std::array<double, 3>
body_to_earth(const std::array<double, 3> body_coords, double roll,
              double pitch, double yaw) {
  std::array<double, 3> earth_coords = {0.0};

  // Compute elements of the rotation matrix
  double c_pitch = cos(pitch);
  double s_pitch = sin(pitch);
  double c_roll = cos(roll);
  double s_roll = sin(roll);
  double c_yaw = cos(yaw);
  double s_yaw = sin(yaw);

  // Compute rotation matrix (earth to body)
  std::array<std::array<double, 3>, 3> rotation_matrix = {
      {{c_pitch * c_yaw, c_pitch * s_yaw, -s_pitch},
       {c_yaw * s_roll * s_pitch - s_yaw * c_roll,
        s_roll * s_pitch * s_yaw + c_roll * c_yaw, s_roll * c_pitch},
       {c_roll * s_pitch * c_yaw + s_roll * s_yaw,
        c_roll * s_pitch * s_yaw - s_roll * c_yaw, c_roll * c_pitch}}};

  std::array<std::array<double, 3>, 3> transpose{{{0.0}}};

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      transpose[i][j] = rotation_matrix[j][i];
    }
  }

  // Apply rotation matrix to convert Earth-fixed coordinates to body-fixed
  // coordinates
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      earth_coords[i] += transpose[i][j] * body_coords[j];
    }
  }

  return earth_coords;
}

// Function to perform rotation from the Earth frame to the body-fixed frame
// (aircraft)
// roll (bank angle)
// pitch
// yaw (heading angle)
inline std::array<double, 3>
earth_to_body(const std::array<double, 3> earth_coords, double roll,
              double pitch, double yaw) {
  std::array<double, 3> body_coords = {0.0};

  // Compute elements of the rotation matrix
  double c_pitch = cos(pitch);
  double s_pitch = sin(pitch);
  double c_roll = cos(roll);
  double s_roll = sin(roll);
  double c_yaw = cos(yaw);
  double s_yaw = sin(yaw);

  // phi    roll
  // psi    yaw
  // theta  pitch

  // Compute rotation matrix
  std::array<std::array<double, 3>, 3> rotation_matrix = {
      {{c_pitch * c_yaw, c_pitch * s_yaw, -s_pitch},

       {c_yaw * s_roll * s_pitch - s_yaw * c_roll,
        s_roll * s_pitch * s_yaw + c_roll * c_yaw, s_roll * c_pitch},

       {c_roll * s_pitch * c_yaw + s_roll * s_yaw,
        c_roll * s_pitch * s_yaw - s_roll * c_yaw, c_roll * c_pitch}}};

  // Apply rotation matrix to convert Earth-fixed coordinates to body-fixed
  // coordinates
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      body_coords[i] += rotation_matrix[i][j] * earth_coords[j];
    }
  }

  return body_coords;
}

// Function that returns the body angular velocities in the body-fixed frame
inline std::array<double, 3> body_angular_vel(double roll, double pitch,
                                              double yaw, double roll_vel,
                                              double pitch_vel,
                                              double yaw_vel) {
  // forward_angle <-> p
  // lateral_angle <-> q
  // downward_angle <-> r
  double forward_angle{0.0}, lateral_angle{0.0}, downward_angle{0.0};

  forward_angle = roll_vel - yaw_vel * sin(pitch);
  lateral_angle = pitch_vel * cos(roll) + yaw_vel * cos(pitch) * sin(roll);
  downward_angle = yaw_vel * cos(roll) * cos(pitch) - pitch_vel * sin(roll);

  std::array<double, 3> body_angular_vel_vector{
      {forward_angle, lateral_angle, downward_angle}};

  return body_angular_vel_vector;
}

#endif
