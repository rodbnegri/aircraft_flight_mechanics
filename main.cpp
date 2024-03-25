#include "src/framesnrotations.h"
#include <iostream>

int main() {
  double bank_angle{0.0 * M_PI / 180.0}, pitch{90.0 * M_PI / 180.0},
      yaw{0.0 * M_PI / 180.0};
  std::array<double, 3> earth_coord{1.0, 0.0, 0.0}, body_coord{0.0, 0.0, 0.0};

  body_coord = earth_to_body(earth_coord, bank_angle, pitch, yaw);

  std::cout << "The body-fixed coords. are: ";
  for (const auto &coord : body_coord) {
    std::cout << coord << " ";
  }
  std::cout << "\n";

  std::array<double, 3> earth_coord_new =
      body_to_earth(body_coord, bank_angle, pitch, yaw);

  std::cout << "Returning to earth coords.: ";
  for (const auto &coord : earth_coord_new) {
    std::cout << coord << " ";
  }
  std::cout << "\n";

  return 0;
}
