#include "ros_context_work.hpp"
#include <cassert>
#include <stdexcept>
int main() {
  auto context=std::make_shared<rclcpp::Context>();
  context->init(0,nullptr);
  bool ran=false;
  assert(sim2real::WithRosContext(context,[&]{ran=true;}));assert(ran);
  bool propagated=false;
  try {sim2real::WithRosContext(context,[]{throw std::runtime_error("live graph failure");});}
  catch(const std::runtime_error&){propagated=true;}
  assert(propagated && context->is_valid());
  // Reproduce invalidation AFTER entry, before the graph query returns.
  assert(!sim2real::WithRosContext(context,[&]{context->shutdown("test");throw std::runtime_error("context invalid");}));
  ran=false;assert(!sim2real::WithRosContext(context,[&]{ran=true;}));assert(!ran);
}
