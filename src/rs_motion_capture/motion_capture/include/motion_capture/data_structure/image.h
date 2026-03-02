//
// Created by sti on 2025/6/12.
//

#ifndef MOTION_CAPTURE_DATA_STRUCTURE_IMAGE_H
#define MOTION_CAPTURE_DATA_STRUCTURE_IMAGE_H

namespace robosense {
namespace motion_capture {

struct Image {
  using Ptr = std::shared_ptr<Image>;
  uint64_t timestamp;
  cv::Mat data;
};

}
}

#endif //MOTION_CAPTURE_DATA_STRUCTURE_IMAGE_H
