#include "slam.h"
namespace cartographer {
namespace slam {
Slam::Slam() {
  map_builder_ptr_ = std::make_unique<mapping::MapBuilder>();
  node_handle_ptr_ = std::make_unique<ros::NodeHandle>();
  command_srv_ = node_handle_ptr_->advertiseService(
      "command", &Slam::SetCommandCallBack, this);
  submaps_publisher_ =
      node_handle_ptr_->advertise<nav_msgs::OccupancyGrid>("map", 100);
  submap_poses_publisher_ =
      node_handle_ptr_->advertise<geometry_msgs::PoseStamped>("submap_poses",
                                                              100);
};

Slam::~Slam(){};

bool Slam::SetCommandCallBack(Command::Request& req, Command::Response& res) {
  if (req.command == "load") {
    std::string filename = std::string(kMapsDirectory) +
                           std::to_string(req.filename) + ".pbstream";
    map_builder_ptr_->LoadStateFromFile(filename, false);
    PublishSubmaps();
  }
  return true;
};

void Slam::PublishNodes() {}

void Slam::PublishSubmaps() {
  auto to_geometry_pose = [&](const transform::Rigid3d& rigid3d) {
    geometry_msgs::Pose pose;
    pose.position.x = rigid3d.translation().x();
    pose.position.y = rigid3d.translation().y();
    pose.position.z = rigid3d.translation().z();
    pose.orientation.w = rigid3d.rotation().w();
    pose.orientation.x = rigid3d.rotation().x();
    pose.orientation.y = rigid3d.rotation().y();
    pose.orientation.z = rigid3d.rotation().z();
    return pose;
  };
  mapping::MapById<mapping::SubmapId, mapping::SubmapPose> all_submap_poses =
      map_builder_ptr_->pose_graph()->GetAllSubmapPoses();
  for (auto&& submap_id_pose : all_submap_poses) {
    geometry_msgs::Pose submap_pose =
        to_geometry_pose(submap_id_pose.data.pose);
    LOG(ERROR) << "Submap pose 1:(" << submap_id_pose.data.pose << ").";
    mapping::proto::SubmapQuery::Response proto;
    std::string error =
        map_builder_ptr_->SubmapToProto(submap_id_pose.id, &proto);
    if (proto.textures().size() > 1) {
      LOG(ERROR) << "Proto textures size is (" << proto.textures().size()
                 << "),continue.";
      continue;
    }
    if (proto.textures().empty()) {
      LOG(ERROR) << "Proto textures is empty,continue.";
      continue;
    }
    for (auto&& texture : proto.textures()) {
      nav_msgs::OccupancyGrid grid;
      grid.header.stamp = ros::Time::now();
      grid.header.frame_id = "map";
      grid.info.resolution = texture.resolution();
      grid.info.width = texture.width();
      grid.info.height = texture.height();
      // grid.info.origin = submap_pose;
      transform::Rigid3d slice_pose = transform::ToRigid3(texture.slice_pose());
      grid.info.origin =
          to_geometry_pose(submap_id_pose.data.pose * slice_pose);
      // grid.info.origin.position.x = 0.;
      // grid.info.origin.position.y = 0.;
      // grid.info.origin.position.y = 0.;
      // grid.info.origin.position.z = 0.;
      // grid.info.origin.orientation.w = 1.;
      // grid.info.origin.orientation.x = 0.;
      // grid.info.origin.orientation.y = 0.;
      // grid.info.origin.orientation.z = 0.;
      std::string cells;
      ::cartographer::common::FastGunzipString(texture.cells(), &cells);
      const int num_pixels = texture.width() * texture.height();
      CHECK_EQ(cells.size(), 2 * num_pixels);
      LOG(ERROR) << "id:" << submap_id_pose.id << ",Cells size:("
                 << cells.size() << "),num_pixels:(" << num_pixels << ").";
      for (int i = 0; i < texture.height(); ++i) {
        for (int j = 0; j < texture.width(); ++j) {
          int intensity = cells[(i * texture.width() + j) * 2];
          if (intensity > 0)
            grid.data.push_back(intensity);
          else
            grid.data.push_back(-1);
        }
      }
      submaps_publisher_.publish(grid);
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header = grid.header;
      pose_stamped.pose = submap_pose;
      submap_poses_publisher_.publish(pose_stamped);
      sleep(2);
    }
  }
};

}  // namespace slam
}  // namespace cartographer