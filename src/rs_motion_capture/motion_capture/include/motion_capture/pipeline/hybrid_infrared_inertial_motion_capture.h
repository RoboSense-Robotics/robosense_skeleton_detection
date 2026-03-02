//
// Created by sti on 2025/7/29.
//

#ifndef MOTION_CAPTURE_PIPELINE_HYBRID_INFRARED_INERTIAL_MOTION_CAPTURE_H
#define MOTION_CAPTURE_PIPELINE_HYBRID_INFRARED_INERTIAL_MOTION_CAPTURE_H

#include "motion_capture/base/base_pipeline.h"
#include "motion_capture/interface/motion_capture_interface.h"
#include <memory>
namespace robosense {
namespace motion_capture {
class HybridInfraredInertialMotionCapture : public BasePipeline {
public:
  using Ptr = std::shared_ptr<HybridInfraredInertialMotionCapture>;
  ~HybridInfraredInertialMotionCapture() { stop(); }

  void init() override;
  void start() override;
  void stop() override;

  void addSensorData(const std::string &type,
                     const PointCloud::Ptr &data) override;
  void addSensorData(const std::string &type, const Image::Ptr &data) override;
  void addSensorData(const std::string &type,
                     const JointPose::Ptr &data) override;
  void addSensorData(const std::string &type,
                     const JointPose2::Ptr &data) override;

  void addTrackerData(const std::string &type, const JointPose2::Ptr &data) override;

  void regOpticalCallback(const std::function<void(const Msg::Ptr&)>& callback) override;

  void regInertialCallback(const std::function<void(const Msg::Ptr&)>& callback) override;

  std::string name() const override { return "HybridInfraredInertialMotionCapture"; }

private:
  std::vector<std::string> sync_frame_ids_ = {
          "left_infrared_tracker",
          "right_infrared_tracker",
          "hand_pose"
  };

  rally::ApproximateSynchronizer<AnyMsg>::Ptr sync_ptr_;

  MotionCaptureInterface::Ptr infrared_motion_capture_interface_ptr_;
  MotionCaptureInterface::Ptr inertial_motion_capture_interface_ptr_;

  rally::ConsumerWorker<Msg::Ptr>::Ptr infrared_motion_capture_worker_ptr_;
  rally::ConsumerWorker<Msg::Ptr>::Ptr inertial_motion_capture_worker_ptr_;
  std::mutex infrared_cb_reg_mutex_;
  std::mutex inertial_cb_reg_mutex_;

  std::vector<std::function<void(const Msg::Ptr&)>> infrared_cbs_;
  std::vector<std::function<void(const Msg::Ptr&)>> inertial_cbs_;

  uint32_t seq_ = 0;
  TimeRecorder::Ptr sync_lat_ptr_;
  TimeRecorder::Ptr infrared_lat_ptr_;

};
} // namespace motion_capture
} // namespace robosense

#endif
