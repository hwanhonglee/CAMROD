#ifndef CAMROD_PERCEPTION__APRILTAG_SEARCH_POLICY_HPP_
#define CAMROD_PERCEPTION__APRILTAG_SEARCH_POLICY_HPP_

// HJ_261110 - CPU budget for the rear parking-tag detector. While no tag is
// tracked, only every Nth frame is searched (skipped frames return before
// cv_bridge/gray conversion) and full-frame searches run decimated. Once a tag
// is tracked, the ROI is searched every frame at the precise decimation, so
// near-dock pose accuracy is unchanged.

namespace camrod_perception
{

class AprilTagSearchPolicy
{
public:
  explicit AprilTagSearchPolicy(int idle_frame_stride = 2)
  : idle_frame_stride_(idle_frame_stride < 1 ? 1 : idle_frame_stride) {}

  // True when this frame should be processed. Tracking processes every frame.
  bool processFrame(bool tracking)
  {
    if (tracking) {
      idle_frames_ = 0;
      return true;
    }
    const bool process = (idle_frames_ % idle_frame_stride_) == 0;
    ++idle_frames_;
    return process;
  }

  int idleFrameStride() const {return idle_frame_stride_;}

private:
  int idle_frame_stride_{2};
  long long idle_frames_{0};
};

// Full-frame searches trade some first-acquisition range for CPU; ROI
// tracking keeps the precise decimation.
inline float quadDecimateFor(bool roi_search, double roi_decimate, double full_search_decimate)
{
  const double value = roi_search ? roi_decimate : full_search_decimate;
  return static_cast<float>(value < 1.0 ? 1.0 : value);
}

}  // namespace camrod_perception

#endif  // CAMROD_PERCEPTION__APRILTAG_SEARCH_POLICY_HPP_
