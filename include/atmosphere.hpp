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

#ifndef ATMOSPHERE_HPP

#include <cmath>
#include <iostream>

const double GRAVITY_SEALEVEL_mps2{9.80665};
const double UNIVERSAL_GAS_CONSTANT_JpKpkg{287}; // (dry air)
const double HEAT_CAPACITY_RATIO{1.4};
const double SOUND_SPEED_SEALEVEL_mps{340.262};
const double DENSITY_SEALEVEL_kgpm3{1.225};
const double PRESSURE_SEALEVEL_Pa{101325};
const double TEMPERATURE_SEALEVEL_K{288.15};
const int TROPOPAUSE_HEIGHT_m{11000};

// function to calculate the temperature following the ISA model
inline double ISA_temperature(double height_m) {
  double temperature_K{TEMPERATURE_SEALEVEL_K - 0.0065 * height_m};
  if (height_m > TROPOPAUSE_HEIGHT_m) {
    temperature_K = TEMPERATURE_SEALEVEL_K - 0.0065 * TROPOPAUSE_HEIGHT_m;
  };
  return temperature_K;
}

// function to calculate the sound speed following the ISA model
inline double ISA_soundspeed(double temperature_K) {

  double sound_speed_mps{std::sqrt(
      temperature_K * UNIVERSAL_GAS_CONSTANT_JpKpkg * HEAT_CAPACITY_RATIO)};

  return sound_speed_mps;
}

// function to calculate the pressure following the ISA model
inline double ISA_airpressure(double temperature_K, double height_m) {

  if (height_m <= TROPOPAUSE_HEIGHT_m && height_m >= 0)
  // aircraft is inside troposphere
  {
    double pressure_Pa{
        PRESSURE_SEALEVEL_Pa *
        pow(temperature_K / TEMPERATURE_SEALEVEL_K,
            GRAVITY_SEALEVEL_mps2 / UNIVERSAL_GAS_CONSTANT_JpKpkg / 0.0065)};
    return pressure_Pa;
  } else if (height_m > TROPOPAUSE_HEIGHT_m &&
             height_m < 20100) // aircrafit is inside stratosphere
  {
    // tropopause value
    double T11{ISA_temperature(TROPOPAUSE_HEIGHT_m)};
    double p11{
        PRESSURE_SEALEVEL_Pa *
        pow(temperature_K / TEMPERATURE_SEALEVEL_K,
            GRAVITY_SEALEVEL_mps2 / UNIVERSAL_GAS_CONSTANT_JpKpkg / 0.0065)};
    // calculating the pressure
    double pressure_Pa{p11 * std::exp(-GRAVITY_SEALEVEL_mps2 /
                                      UNIVERSAL_GAS_CONSTANT_JpKpkg / T11 *
                                      (height_m - TROPOPAUSE_HEIGHT_m))};
    return pressure_Pa;
  } else
  // error
  {
    std::cerr << "Error: Invalid height (not implemented).\n";
    return -1;
  };
}

// function to calculate the density in accordance with the ISA model
inline double ISA_density(double temperature_K, double pressure_Pa) {
  double density_kgpm3{pressure_Pa / UNIVERSAL_GAS_CONSTANT_JpKpkg /
                       temperature_K};
  return density_kgpm3;
}

// function to calculate the calibrated airspeed
inline double calibrated_airspeed(double true_air_speed_mps, double height_m) {
  double temperature_K = ISA_temperature(height_m);
  double sound_speed_mps = ISA_soundspeed(temperature_K);
  double Mach_number{true_air_speed_mps / sound_speed_mps};
  double pressure_Pa = ISA_airpressure(temperature_K, height_m);
  double dp{0};
  dp = pressure_Pa *
       (pow(0.5 * (HEAT_CAPACITY_RATIO - 1) * Mach_number * Mach_number + 1,
            HEAT_CAPACITY_RATIO / (HEAT_CAPACITY_RATIO - 1)) -
        1);
  double calibrated_airspeed_mps{0};
  calibrated_airspeed_mps =
      std::sqrt(2 * SOUND_SPEED_SEALEVEL_mps * SOUND_SPEED_SEALEVEL_mps /
                (HEAT_CAPACITY_RATIO - 1) *
                (pow(dp / PRESSURE_SEALEVEL_Pa + 1,
                     (HEAT_CAPACITY_RATIO - 1) / HEAT_CAPACITY_RATIO) -
                 1));

  return calibrated_airspeed_mps;
}

#endif // !ATMOSPHERE_HPP
