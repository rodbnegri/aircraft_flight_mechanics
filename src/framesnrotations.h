#ifndef FRAMESNROTATIONS_H

#include <array>
#include <cmath>

// Function to perform rotation from the body-fixed frame (aircraft) to the
// Earth
inline std::array<double, 3>
body_to_earth(const std::array<double, 3> &body_coords, double pitch,
              double bank_angle, double yaw) {
  std::array<double, 3> earth_coords = {0.0, 0.0, 0.0};

  // Compute elements of the rotation matrix
  double c_pitch = cos(pitch);
  double s_pitch = sin(pitch);
  double c_bank_angle = cos(bank_angle);
  double s_bank_angle = sin(bank_angle);
  double c_yaw = cos(yaw);
  double s_yaw = sin(yaw);

  // Compute rotation matrix (earth to body)
  std::array<std::array<double, 3>, 3> rotation_matrix = {
      {{c_pitch * c_bank_angle,
        c_yaw * c_bank_angle * s_pitch - s_yaw * s_bank_angle,
        s_yaw * c_bank_angle * s_pitch + c_yaw * s_bank_angle},
       {s_bank_angle, c_yaw * c_bank_angle, -s_yaw * c_bank_angle},
       {-s_pitch * c_bank_angle,
        c_yaw * s_bank_angle * s_pitch + s_yaw * c_bank_angle,
        s_yaw * s_bank_angle * s_pitch - c_yaw * c_bank_angle}}};

  std::array<std::array<double, 3>, 3> transpose;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      transpose[i][j] = rotation_matrix[j][i];
    }
  }

  // Apply rotation matrix to convert Earth-fixed coordinates to body-fixed
  // coordinates
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      earth_coords[i] += transpose[i][j] * earth_coords[j];
    }
  }

  return earth_coords;
}

// Function to perform rotation from the Earth frame to the body-fixed frame
// (aircraft)
inline std::array<double, 3>
earth_to_body(const std::array<double, 3> &earth_coords, double pitch,
              double bank_angle, double yaw) {
  std::array<double, 3> body_coords = {0.0, 0.0, 0.0};

  // Compute elements of the rotation matrix
  double c_pitch = cos(pitch);
  double s_pitch = sin(pitch);
  double c_bank_angle = cos(bank_angle);
  double s_bank_angle = sin(bank_angle);
  double c_yaw = cos(yaw);
  double s_yaw = sin(yaw);

  // Compute rotation matrix
  std::array<std::array<double, 3>, 3> rotation_matrix = {
      {{c_pitch * c_bank_angle,
        c_yaw * c_bank_angle * s_pitch - s_yaw * s_bank_angle,
        s_yaw * c_bank_angle * s_pitch + c_yaw * s_bank_angle},
       {s_bank_angle, c_yaw * c_bank_angle, -s_yaw * c_bank_angle},
       {-s_pitch * c_bank_angle,
        c_yaw * s_bank_angle * s_pitch + s_yaw * c_bank_angle,
        s_yaw * s_bank_angle * s_pitch - c_yaw * c_bank_angle}}};

  // Apply rotation matrix to convert Earth-fixed coordinates to body-fixed
  // coordinates
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      body_coords[i] += rotation_matrix[i][j] * earth_coords[j];
    }
  }

  return body_coords;
}

#endif
