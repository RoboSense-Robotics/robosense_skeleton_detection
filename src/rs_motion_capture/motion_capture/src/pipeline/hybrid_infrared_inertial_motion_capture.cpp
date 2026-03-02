#include "motion_capture/pipeline/hybrid_infrared_inertial_motion_capture.h"
#include "rally/utils/global/configure_manager.h"

namespace robosense {
namespace motion_capture {

void HybridInfraredInertialMotionCapture::init() {
  AINFO << name() << ": start init...";
  auto config_node = rally::ConfigureManager::getInstance().getCfgNode();
  auto pipeline_config_node =
      config_node["pipeline"]["HybridInfraredInertialMotionCapture"];

  auto infrared_cfg_node = pipeline_config_node["infrared"];
  auto inertial_cfg_node = pipeline_config_node["inertial"];

  infrared_motion_capture_interface_ptr_ =
      std::make_shared<MotionCaptureInterface>();
  infrared_motion_capture_interface_ptr_->init(infrared_cfg_node);
  inertial_motion_capture_interface_ptr_ =
      std::make_shared<MotionCaptureInterface>();
  inertial_motion_capture_interface_ptr_->init(inertial_cfg_node);

  sync_lat_ptr_ = std::make_shared<TimeRecorder>("sensor sync latency");
  infrared_lat_ptr_ =
      std::make_shared<TimeRecorder>("infrared motion capture latency");

  // init worker thread
  infrared_motion_capture_worker_ptr_ =
      std::make_shared<rally::ConsumerWorker<Msg::Ptr>>(
          "infrared_motion_capture");
  inertial_motion_capture_worker_ptr_ =
      std::make_shared<rally::ConsumerWorker<Msg::Ptr>>(
          "inertial_motion_capture");

  infrared_motion_capture_worker_ptr_->bind([&](const Msg::Ptr &msg_ptr) {
    infrared_motion_capture_interface_ptr_->process(msg_ptr);
    infrared_lat_ptr_->lat(msg_ptr->timestamp);
    if (!msg_ptr->valid_flag) {
      return;
    }
    for (const auto &f : infrared_cbs_) {
      f(msg_ptr);
    }
  });
  inertial_motion_capture_worker_ptr_->bind([&](const Msg::Ptr &msg_ptr) {
    inertial_motion_capture_interface_ptr_->process(msg_ptr);
    for (const auto &f : inertial_cbs_) {
      f(msg_ptr);
    }
  });

  rally::ApproximateSynOptions sync_options;
  sync_options.frame_id_vec = sync_frame_ids_;
  sync_options.inter_message_lower_bound = {5, 5, 15};
  sync_options.queue_size = {100, 100, 50};
  /*sync_options.inter_message_lower_bound = {5, 5};*/
  /*sync_options.queue_size = {100, 100};*/
  sync_ptr_.reset(new rally::ApproximateSynchronizer<AnyMsg>(sync_options));

  auto sync_cb_func = [&](const std::vector<AnyMsg::Ptr> &msg_vec) {
    Msg::Ptr msg_ptr = std::make_shared<Msg>();
    msg_ptr->use_glove = true;
    for (const auto &cur_msg : msg_vec) {
      switch (cur_msg->msg_type) {
      case MsgType::JOINT_POSE: {
        msg_ptr->input_msg_ptr->hand_joints_pose =
            *cur_msg->any_ptr->AnyCast<JointPose::Ptr>();
        break;
      }
      case MsgType::JOINT_POSE2: {
        RINFO<<"joint pose2: "<<cur_msg->frame_id;
        if(cur_msg->frame_id == "hand_pose"){
          msg_ptr->input_msg_ptr->hand_joints_pose2 =
              *cur_msg->any_ptr->AnyCast<JointPose2::Ptr>();
        }else if(cur_msg->frame_id == "left_infrared_tracker"){
          msg_ptr->input_msg_ptr->left_infrared_tracker =
              *cur_msg->any_ptr->AnyCast<JointPose2::Ptr>();
        }else if(cur_msg->frame_id == "right_infrared_tracker"){
          msg_ptr->input_msg_ptr->right_infrared_tracker =
              *cur_msg->any_ptr->AnyCast<JointPose2::Ptr>();
          msg_ptr->timestamp = cur_msg->timestamp;
        }
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
    if (msg_ptr->input_msg_ptr->hand_joints_pose) {
      double ts = rally::toSeconds(
          msg_ptr->input_msg_ptr->hand_joints_pose->timestamp);
      AINFO << std::fixed << std::setprecision(6) << ts
            << " [hand_joints_pose]";
    }
    if (msg_ptr->input_msg_ptr->hand_joints_pose2) {
      double ts = rally::toSeconds(
          msg_ptr->input_msg_ptr->hand_joints_pose2->timestamp);
      AINFO << std::fixed << std::setprecision(6) << ts
            << " [hand_joints_pose2]";
    }
    if (msg_ptr->input_msg_ptr->left_infrared_tracker) {
      double ts = rally::toSeconds(
          msg_ptr->input_msg_ptr->left_infrared_tracker->timestamp);
      AINFO << std::fixed << std::setprecision(6) << ts
            << " [left_infrared_tracker]";
    }
    if (msg_ptr->input_msg_ptr->right_infrared_tracker) {
      double ts = rally::toSeconds(
          msg_ptr->input_msg_ptr->right_infrared_tracker->timestamp);
      AINFO << std::fixed << std::setprecision(6) << ts
            << " [right_infrared_tracker]";
    }
    AINFO << "================================================= ";
    sync_lat_ptr_->lat(msg_ptr->timestamp);
    infrared_motion_capture_worker_ptr_->add(msg_ptr);
  };

  sync_ptr_->regSynCallback(sync_cb_func);
  AINFO << name() << ": finish init.";
}

void HybridInfraredInertialMotionCapture::addSensorData(
    const std::string &type, const Image::Ptr &data) {
  uint64_t timestamp = data->timestamp;
  double now = rally::getNowInSeconds();
  double delta = now - rally::toSeconds(timestamp);
  AINFO << std::fixed << std::setprecision(6) << name() << " receive " << type
        << " time: " << now << ", header time: " << rally::toSeconds(timestamp)
        << ", delta = " << delta << "s";
  AnyMsg::Ptr any_msg_ptr(new AnyMsg);
  any_msg_ptr->frame_id = type;
  any_msg_ptr->timestamp = timestamp;
  any_msg_ptr->msg_type = MsgType::IMAGE;
  any_msg_ptr->any_ptr.reset(new rally::Any(data));
  sync_ptr_->addData(any_msg_ptr);
}

void HybridInfraredInertialMotionCapture::addSensorData(
    const std::string &type, const PointCloud::Ptr &data) {
  uint64_t timestamp = data->timestamp;
  double now = rally::getNowInSeconds();
  double delta = now - rally::toSeconds(timestamp);
  AINFO << std::fixed << std::setprecision(6) << name() << " receive " << type
        << " time: " << now << ", header time: " << rally::toSeconds(timestamp)
        << ", delta = " << delta << "s";
  AnyMsg::Ptr any_msg_ptr(new AnyMsg);
  any_msg_ptr->frame_id = type;
  any_msg_ptr->timestamp = timestamp;
  any_msg_ptr->msg_type = MsgType::POINTCLOUD;
  any_msg_ptr->any_ptr.reset(new rally::Any(data));
  sync_ptr_->addData(any_msg_ptr);
}

void HybridInfraredInertialMotionCapture::addSensorData(
    const std::string &type, const JointPose::Ptr &data) {
  uint64_t timestamp = data->timestamp;
  double now = rally::getNowInSeconds();
  double delta = now - rally::toSeconds(timestamp);
  AINFO << std::fixed << std::setprecision(6) << name() << " receive " << type
        << " time: " << now << ", header time: " << rally::toSeconds(timestamp)
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

void HybridInfraredInertialMotionCapture::addSensorData(
    const std::string &type, const JointPose2::Ptr &data) {
  uint64_t timestamp = data->timestamp;
  double now = rally::getNowInSeconds();
  double delta = now - rally::toSeconds(timestamp);
  AINFO << std::fixed << std::setprecision(6) << name() << " receive " << type
        << " time: " << now << ", header time: " << rally::toSeconds(timestamp)
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

void HybridInfraredInertialMotionCapture::addTrackerData(
    const std::string &type, const JointPose2::Ptr &data) {
  uint64_t timestamp = data->timestamp;
  double now = rally::getNowInSeconds();
  double delta = now - rally::toSeconds(timestamp);
  AINFO << std::fixed << std::setprecision(6) << name() << " receive " << type
        << " time: " << now << ", header time: " << rally::toSeconds(timestamp)
        << ", delta = " << delta << "s";
  AnyMsg::Ptr any_msg_ptr(new AnyMsg);
  any_msg_ptr->frame_id = type;
  any_msg_ptr->timestamp = timestamp;
  any_msg_ptr->msg_type = MsgType::JOINT_POSE2;
  any_msg_ptr->any_ptr.reset(new rally::Any(data));
  sync_ptr_->addData(any_msg_ptr);
}

void HybridInfraredInertialMotionCapture::start() {
  sync_ptr_->start();
  infrared_motion_capture_worker_ptr_->start();
  inertial_motion_capture_worker_ptr_->start();
}

void HybridInfraredInertialMotionCapture::stop() {
  sync_ptr_->stop();
  infrared_motion_capture_worker_ptr_->stop();
  inertial_motion_capture_worker_ptr_->stop();
}

void HybridInfraredInertialMotionCapture::regOpticalCallback(
    const std::function<void(const Msg::Ptr &)> &callback) {
  const std::unique_lock<std::mutex> lock{infrared_cb_reg_mutex_};
  infrared_cbs_.emplace_back(callback);
}

void HybridInfraredInertialMotionCapture::regInertialCallback(
    const std::function<void(const Msg::Ptr &)> &callback) {
  const std::unique_lock<std::mutex> lock{inertial_cb_reg_mutex_};
  inertial_cbs_.emplace_back(callback);
}

RS_REGISTER_PIPELINE(HybridInfraredInertialMotionCapture)
} // namespace motion_capture
} // namespace robosense
