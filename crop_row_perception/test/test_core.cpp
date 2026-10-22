// Standalone contract tests: run with any C++17 compiler, without ROS/OpenCV/NPU.
#include "crop_row_perception/core.hpp"
#include <cstdlib>
#include <iostream>
#include <future>
#include <limits>
#include <memory>
#define CHECK(x) do {if (!(x)) {std::cerr << "FAIL line " << __LINE__ << ": " #x "\n"; std::exit(1);}} while (false)
using namespace crop_row_perception;
int main() {
  FrameClockMapper clock;
  CHECK(clock.map(990000000, 1000000000, 5000000000, 250).value() == 4990000000);
  CHECK(!clock.map(0, 1000000000, 5000000000, 250));
  CHECK(!clock.map(1000000001, 1000000000, 5000000000, 250));
  CHECK(!clock.map(700000000, 1000000000, 5000000000, 250));
  CHECK(clock.map(1250000000, 1250000000, 5250000000, 250).value() == 5250000000);
  CHECK(!clock.map(1400000000, 1500000000, 5300000000, 250));  // ROS clock jump
  CHECK(!clock.map(1400000000, 1400000000, 5400000000, 250));  // latched
  LatestFrameQueue<int> queue;
  CHECK(queue.put(1)); CHECK(queue.put(2)); CHECK(queue.dropped() == 1);
  CHECK(queue.take().value() == 2);
  auto waiter = std::async(std::launch::async, [&queue] {return queue.take();});
  queue.close(); CHECK(!waiter.get()); CHECK(!queue.put(3));
  LatestFrameQueue<std::shared_ptr<int>> owned;
  auto buffer = std::make_shared<int>(42);
  std::weak_ptr<int> lifetime = buffer;
  owned.put(buffer); buffer.reset(); CHECK(!lifetime.expired());
  auto in_flight = owned.take();
  owned.put(std::make_shared<int>(43)); owned.put(std::make_shared<int>(44));
  CHECK(**in_flight == 42); CHECK(!lifetime.expired());
  in_flight.reset(); CHECK(lifetime.expired()); owned.close();
  rk_row_nav_config_t config;
  rk_row_nav_config_init(&config); config.model_path = "fake";
  {
    RowNavSdk sdk(config);
    rk_row_nav_frame_t frame;
    rk_row_nav_frame_init(&frame);
    uint8_t pixel = 0; frame.data = &pixel;
    rk_row_nav_result_t result;
    CHECK(sdk.process(frame, result) == RK_ROW_NAV_OK);
    CHECK(usable(result)); CHECK(result.lateral_error_m > 0); CHECK(result.heading_error_rad < 0);
    pixel = 1; CHECK(sdk.process(frame, result) == RK_ROW_NAV_INFERENCE_ERROR);
    CHECK(!usable(result)); CHECK(std::isnan(result.lateral_error_m));
    pixel = 2; CHECK(sdk.process(frame, result) == 0); CHECK(!usable(result));
    pixel = 3; CHECK(sdk.process(frame, result) == 0); CHECK(!usable(result));
  }
  config.model_path = "!fail";
  bool failed = false;
  try {RowNavSdk sdk(config);} catch (const std::runtime_error &) {failed = true;}
  CHECK(failed);
  std::cout << "clock/queue/SDK lifecycle/error/NaN/sign contracts passed\n";
}
