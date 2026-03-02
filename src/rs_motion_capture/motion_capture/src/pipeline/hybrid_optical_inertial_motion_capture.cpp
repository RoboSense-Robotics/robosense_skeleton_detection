//
// Created by sti on 2025/6/6.
//

#include "motion_capture/pipeline/hybrid_optical_inertial_motion_capture.h"

namespace robosense {
namespace motion_capture {

void HybridOpticalInertialMotionCapture::init() {
  AINFO << name() << ": start init...";
  // init sensor config
  auto config_node = rally::ConfigureManager::getInstance().getCfgNode();
  SensorManager::getInstance().init(config_node["sensor"]);

  // get pipeline conf
  auto pipeline_config_node = config_node["pipeline"]["HybridOpticalInertialMotionCapture"];

  // optical cfg
  auto optical_cfg_node = pipeline_config_node["optical"];
  auto inertial_cfg_node = pipeline_config_node["inertial"];

  // init optical and inertial motion capture interface
  optical_motion_capture_interface_ptr_ = std::make_shared<MotionCaptureInterface>();
  optical_motion_capture_interface_ptr_->init(optical_cfg_node);
  inertial_motion_capture_interface_ptr_ = std::make_shared<MotionCaptureInterface>();
  inertial_motion_capture_interface_ptr_->init(inertial_cfg_node);

  // init lat recorder
  sync_lat_ptr_ = std::make_shared<TimeRecorder>("sensor sync latency");
  optical_lat_ptr_ = std::make_shared<TimeRecorder>("optical motion capture latency");

  // init worker thread
  optical_motion_capture_worker_ptr_ = std::make_shared<rally::ConsumerWorker<Msg::Ptr>>("optical_motion_capture");
  inertial_motion_capture_worker_ptr_ = std::make_shared<rally::ConsumerWorker<Msg::Ptr>>("inertial_motion_capture");

  // bind
  optical_motion_capture_worker_ptr_->bind([&](const Msg::Ptr &msg_ptr) {
    optical_motion_capture_interface_ptr_->process(msg_ptr);
    optical_lat_ptr_->lat(msg_ptr->timestamp);
    if (!msg_ptr->valid_flag) {
      return;
    }
    for (const auto& f : optical_cbs_) {
      f(msg_ptr);
    }
  });
  inertial_motion_capture_worker_ptr_->bind([&](const Msg::Ptr &msg_ptr) {
    inertial_motion_capture_interface_ptr_->process(msg_ptr);
    for (const auto& f : inertial_cbs_) {
      f(msg_ptr);
    }
  });

  rally::ApproximateSynOptions sync_options;
  sync_options.frame_id_vec = sync_frame_ids_;
  sync_options.inter_message_lower_bound = {13, 13, 25, 25, 15};
  sync_options.queue_size = {60, 60, 30, 30, 50};
  sync_ptr_.reset(new rally::ApproximateSynchronizer<AnyMsg>(sync_options));

  auto sync_cb_func = [&](const std::vector<AnyMsg::Ptr> &msg_vec) {
    Msg::Ptr msg_ptr = std::make_shared<Msg>();
    msg_ptr->use_glove = true;
    for (const auto& cur_msg : msg_vec) {
      switch (cur_msg->msg_type) {
        case MsgType::IMAGE: {
          msg_ptr->input_msg_ptr->image_map[rally::kCameraEnumName2TypeMap.at(cur_msg->frame_id)] =
              *cur_msg->any_ptr->AnyCast<Image::Ptr>();
          break;
        }
        case MsgType::POINTCLOUD: {
          msg_ptr->input_msg_ptr->lidar_map[rally::kLidarEnumName2TypeMap.at(cur_msg->frame_id)] =
              *cur_msg->any_ptr->AnyCast<PointCloud::Ptr>();
          if (cur_msg->frame_id == "left_ac_lidar") {
            msg_ptr->timestamp = cur_msg->timestamp;
          }
          break;
        }
        case MsgType::JOINT_POSE: {
          msg_ptr->input_msg_ptr->hand_joints_pose =
              *cur_msg->any_ptr->AnyCast<JointPose::Ptr>();
          break;
        }
        case MsgType::JOINT_POSE2: {
          msg_ptr->input_msg_ptr->hand_joints_pose2 =
                  *cur_msg->any_ptr->AnyCast<JointPose2::Ptr>();
          break;
        }
        default: {
          RTHROW("invalid sync msg type!");
        }
      }
    }

    msg_ptr->seq = this->seq_;
    this->seq_ = (this->seq_ == UINT32_MAX) ? 0 : (this->seq_ + 1);
    double lidar_ts = rally::toSeconds(msg_ptr->timestamp);
    AINFO << "------------------------------------ start perception "
            << std::fixed << std::setprecision(6) << lidar_ts
            << " [seq: " << msg_ptr->seq << "] "
            << " [delta: " << rally::getNowInSeconds() - lidar_ts << "] "
            << " ------------------------------------";
    AINFO << "===== [timestamp] and [frame_id] as follows ===== ";
    for (const auto& iter : msg_ptr->input_msg_ptr->image_map) {
      double ts = rally::toSeconds(iter.second->timestamp);
      AINFO << std::fixed << std::setprecision(6) << ts << " ["
              << rally::kCameraEnum2NameMap.at(iter.first) << "]";
    }
    for (const auto& iter : msg_ptr->input_msg_ptr->lidar_map) {
      double ts = rally::toSeconds(iter.second->timestamp);
      AINFO << std::fixed << std::setprecision(6) << ts << " ["
              << rally::kLidarEnum2NameMap.at(iter.first) << "]";
    }
    if (msg_ptr->input_msg_ptr->hand_joints_pose) {
      double ts = rally::toSeconds(msg_ptr->input_msg_ptr->hand_joints_pose->timestamp);
      AINFO << std::fixed << std::setprecision(6) << ts << " [hand_joints_pose]";
    }
    if (msg_ptr->input_msg_ptr->hand_joints_pose2) {
      double ts = rally::toSeconds(msg_ptr->input_msg_ptr->hand_joints_pose2->timestamp);
      AINFO << std::fixed << std::setprecision(6) << ts << " [hand_joints_pose2]";
    }
    AINFO << "================================================= ";
    sync_lat_ptr_->lat(msg_ptr->timestamp);
    optical_motion_capture_worker_ptr_->add(msg_ptr);
  };
  sync_ptr_->regSynCallback(sync_cb_func);
  AINFO << name() << ": finish init.";
}

void HybridOpticalInertialMotionCapture::addSensorData(const std::string &type, const Image::Ptr &data) {
  uint64_t timestamp = data->timestamp;
  double now = rally::getNowInSeconds();
  double delta = now - rally::toSeconds(timestamp);
  AINFO << std::fixed << std::setprecision(6) << name()
        << " receive " << type << " time: " << now
        << ", header time: " << rally::toSeconds(timestamp)
        << ", delta = " << delta << "s";
  AnyMsg::Ptr any_msg_ptr(new AnyMsg);
  any_msg_ptr->frame_id = type;
  any_msg_ptr->timestamp = timestamp;
  any_msg_ptr->msg_type = MsgType::IMAGE;
  any_msg_ptr->any_ptr.reset(new rally::Any(data));
  sync_ptr_->addData(any_msg_ptr);
}

void HybridOpticalInertialMotionCapture::addSensorData(const std::string &type, const PointCloud::Ptr &data) {
  uint64_t timestamp = data->timestamp;
  double now = rally::getNowInSeconds();
  double delta = now - rally::toSeconds(timestamp);
  AINFO << std::fixed << std::setprecision(6) << name()
        << " receive " << type << " time: " << now
        << ", header time: " << rally::toSeconds(timestamp)
        << ", delta = " << delta << "s";
  AnyMsg::Ptr any_msg_ptr(new AnyMsg);
  any_msg_ptr->frame_id = type;
  any_msg_ptr->timestamp = timestamp;
  any_msg_ptr->msg_type = MsgType::POINTCLOUD;
  any_msg_ptr->any_ptr.reset(new rally::Any(data));
  sync_ptr_->addData(any_msg_ptr);
}

void HybridOpticalInertialMotionCapture::addSensorData(const std::string &type, const JointPose::Ptr &data) {
  uint64_t timestamp = data->timestamp;
  double now = rally::getNowInSeconds();
  double delta = now - rally::toSeconds(timestamp);
  AINFO << std::fixed << std::setprecision(6) << name()
        << " receive " << type << " time: " << now
        << ", header time: " << rally::toSeconds(timestamp)
        << ", delta = " << delta << "s";
  AnyMsg::Ptr any_msg_ptr(new AnyMsg);
  any_msg_ptr->frame_id = type;
  any_msg_ptr->timestamp = timestamp;
  any_msg_ptr->msg_type = MsgType::JOINT_POSE;
  any_msg_ptr->any_ptr.reset(new rally::Any(data));
  sync_ptr_->addData(any_msg_ptr);

  Msg::Ptr msg_ptr = std::make_shared<Msg>();
  msg_ptr->use_glove = true;
  msg_ptr->input_msg_ptr->hand_joints_pose = data;
  inertial_motion_capture_worker_ptr_->add(msg_ptr);
}

void HybridOpticalInertialMotionCapture::addSensorData(const std::string &type, const JointPose2::Ptr &data) {
  uint64_t timestamp = data->timestamp;
  double now = rally::getNowInSeconds();
  double delta = now - rally::toSeconds(timestamp);
  AINFO << std::fixed << std::setprecision(6) << name()
         << " receive " << type << " time: " << now
         << ", header time: " << rally::toSeconds(timestamp)
         << ", delta = " << delta << "s";
  AnyMsg::Ptr any_msg_ptr(new AnyMsg);
  any_msg_ptr->frame_id = type;
  any_msg_ptr->timestamp = timestamp;
  any_msg_ptr->msg_type = MsgType::JOINT_POSE2;
  any_msg_ptr->any_ptr.reset(new rally::Any(data));
  sync_ptr_->addData(any_msg_ptr);

  Msg::Ptr msg_ptr = std::make_shared<Msg>();
  msg_ptr->use_glove = true;
  msg_ptr->input_msg_ptr->hand_joints_pose2 = data;
  inertial_motion_capture_worker_ptr_->add(msg_ptr);
}

void HybridOpticalInertialMotionCapture::start() {
  sync_ptr_->start();
  optical_motion_capture_worker_ptr_->start();
  inertial_motion_capture_worker_ptr_->start();
}

void HybridOpticalInertialMotionCapture::stop() {
  sync_ptr_->stop();
  optical_motion_capture_worker_ptr_->stop();
  inertial_motion_capture_worker_ptr_->stop();
}

void HybridOpticalInertialMotionCapture::regOpticalCallback(const std::function<void(const Msg::Ptr &)> &callback) {
  const std::unique_lock<std::mutex> lock{optical_cb_reg_mutex_};
  optical_cbs_.emplace_back(callback);
}

void HybridOpticalInertialMotionCapture::regInertialCallback(const std::function<void(const Msg::Ptr &)> &callback) {
  const std::unique_lock<std::mutex> lock{inertial_cb_reg_mutex_};
  inertial_cbs_.emplace_back(callback);
}

RS_REGISTER_PIPELINE(HybridOpticalInertialMotionCapture)

}
}
