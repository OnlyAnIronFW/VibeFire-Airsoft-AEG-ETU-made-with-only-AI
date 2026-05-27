#pragma once

#include <cstdint>
#include <string>
#include <vector>

/// Normalised detection from YOLO output (cx, cy, w, h ∈ [0, 1]).
struct DetectBox {
  float cx;         // centre x (normalised 0–1)
  float cy;         // centre y (normalised 0–1)
  float w;          // width (normalised 0–1)
  float h;          // height (normalised 0–1)
  float confidence; // detection confidence [0, 1]
  int   class_id;   // 0 = person (coco 80-class)
};

// ---------------------------------------------------------------------------
// YOLOInfer — abstract interface for YOLO inference backends
// ---------------------------------------------------------------------------
class YOLOInfer {
 public:
  virtual ~YOLOInfer() = default;

  /// Load the model.  Returns true on success.
  virtual bool init(const std::string& model_path) = 0;

  /// Run inference on a 640×640×3 RGB UINT8 image (NHWC layout).
  /// Returns a list of detected bounding boxes (normalised 0–1).
  virtual std::vector<DetectBox> infer(const uint8_t* rgb_data) = 0;

  /// Returns true once init() has succeeded.
  virtual bool is_ready() const = 0;
};

// ---------------------------------------------------------------------------
// YOLOMock — deterministic mock for x86 development / unit tests
// ---------------------------------------------------------------------------
class YOLOMock : public YOLOInfer {
 public:
  YOLOMock() = default;

  bool init(const std::string& model_path) override;
  std::vector<DetectBox> infer(const uint8_t* rgb_data) override;
  bool is_ready() const override;

  /// Seed the mock with a pre-defined detection list.
  void set_detections(const std::vector<DetectBox>& detections);

 private:
  bool ready_{false};
  std::vector<DetectBox> detections_;
};

// ---------------------------------------------------------------------------
// Post-processing helpers (free functions — testable in isolation)
// ---------------------------------------------------------------------------

/// Sort by confidence descending, apply NMS (IoU threshold), then filter by
/// minimum confidence.  Returns the surviving boxes sorted by confidence.
std::vector<DetectBox> apply_nms(
    const std::vector<DetectBox>& boxes,
    float iou_threshold,
    float conf_threshold);

/// Compute Intersection-over-Union for two DetectBox instances.
float compute_iou(const DetectBox& a, const DetectBox& b);
