#include "crop_row_perception/attitude_gate.hpp"
#include <cstdlib>
#include <iostream>
#include <limits>
#define CHECK(x) do {if (!(x)) {std::cerr << "FAIL line " << __LINE__ << ": " #x "\n"; std::exit(1);}} while (false)
using namespace crop_row_perception;
int main() {
  AttitudeGate gate({0.01, -0.02, 0.035, 50});
  CHECK(!gate.at(1000000000).usable);
  gate.add({1000000000, 0.01, -0.02, true}, 1000000000);
  CHECK(gate.at(1050000000).usable);  // inclusive age boundary
  CHECK(!gate.at(1050000001).usable);
  CHECK(!gate.at(999999999).usable);  // future sample cannot be borrowed
  gate.add({1010000000, 0.01, -0.02, false}, 1010000000);
  CHECK(!gate.at(1010000000).usable);  // no fallback to older valid sample
  CHECK(gate.at(1005000000).usable);  // later invalid pose does not rewrite history
  gate.add({1020000000, 0.1, -0.02, true}, 1020000000);
  CHECK(!gate.at(1020000000).usable);
  gate.add({1030000000, 0.01, 0.1, true}, 1030000000);
  CHECK(!gate.at(1030000000).usable);
  gate.add({1040000000, std::numeric_limits<double>::quiet_NaN(), 0, true}, 1040000000);
  CHECK(!gate.at(1040000000).usable);
  gate.add({1050000000, 0.01, -0.02, true}, 1050000000);
  CHECK(gate.at(1050000000).usable);  // recovery on fresh valid pose
  gate.add({1040000000, 0.01, -0.02, true}, 1060000000);
  CHECK(!gate.at(1050000000).usable);  // backward clock latches
  gate.add({1070000000, 0.01, -0.02, true}, 1070000000);
  CHECK(!gate.at(1070000000).usable);
  AttitudeGate future({});
  future.add({200, 0, 0, true}, 100);
  CHECK(!future.at(200).usable);
  AttitudeGate bounded({});
  for (int64_t i = 1; i <= 130; ++i) {bounded.add({i, 0, 0, true}, i);}
  CHECK(!bounded.at(1).usable); CHECK(bounded.at(130).usable);
  for (const auto config : {AttitudeGateConfig{0, 0, 0, 50},
    AttitudeGateConfig{0, 0, 0.2, 50}, AttitudeGateConfig{0, 0, .035, -1},
    AttitudeGateConfig{0, 2, .035, 50}})
  {
    bool threw = false;
    try {AttitudeGate invalid(config);} catch (const std::invalid_argument &) {threw = true;}
    CHECK(threw);
  }
  std::cout << "attitude matching/age/validity/tilt/clock/bounds/config contracts passed\n";
}
