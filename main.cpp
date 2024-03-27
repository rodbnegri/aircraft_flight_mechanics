#include "src/atmosphere.hpp"
// #include "src/framesnrotations.hpp"
#include <iostream>

int main() {

  double height_m;
  double TAS_mps;

  std::cout << "Enter height in meters: ";
  std::cin >> height_m;

  std::cout << "Enter TAS (True Airspeed) in meters per second: ";
  std::cin >> TAS_mps;

  double calibrated_airspeed_mps = calibrated_airspeed(TAS_mps, height_m);
  double temperature = ISA_temperature(height_m);
  double sound_speed = ISA_soundspeed(temperature);
  double pressure = ISA_airpressure(temperature, height_m);
  double density = ISA_density(temperature, pressure);

  std::cout << "Speed of sound [m/s]: " << sound_speed << "\n";
  std::cout << "Temperature [K]: " << temperature << "\n";
  std::cout << "Pressure [Pa]: " << pressure << "\n";
  std::cout << "Density [kg/m³]: " << density << "\n";
  std::cout << "Mach number : " << TAS_mps / sound_speed << "\n";
  std::cout << "Calibrated air speed [m/s]: " << calibrated_airspeed_mps
            << "\n";

  return 0;
}
