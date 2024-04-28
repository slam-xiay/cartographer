#ifndef CARTOGRAPHER_SLAM_H_
#define CARTOGRAPHER_SLAM_H_
// #include "cartographer/Command.h"
#include "cartographer/mapping/map_builder.h"
namespace cartographer {
namespace slam {

class Slam {
 public:
  explicit Slam();
  ~Slam();
  Slam(const Slam&) = delete;
  Slam& operator=(const Slam&) = delete;
  // bool SetSlamCallBack(Command::Request& req, Command::Response& res);

 private:
  std::unique_ptr<mapping::MapBuilder> map_builder_ptr_;
};
}  // namespace slam
}  // namespace cartographer
#endif