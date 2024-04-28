#include "slam.h"
namespace cartographer {
namespace slam {
Slam::Slam() { map_builder_ptr_ = std::make_unique<mapping::MapBuilder>(); };

Slam::~Slam(){};

}  // namespace slam
}  // namespace cartographer