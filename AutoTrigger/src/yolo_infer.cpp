#include "autotrigger/yolo_infer.h"

#include <algorithm>
#include <cmath>

// ===========================================================================
// YOLOMock
// ===========================================================================

bool YOLOMock::init(const std::string& /*model_path*/) {
  ready_ = true;
  return true;
}

std::vector<DetectBox> YOLOMock::infer(const uint8_t* /*rgb_data*/) {
  return detections_;
}

bool YOLOMock::is_ready() const {
  return ready_;
}

void YOLOMock::set_detections(const std::vector<DetectBox>& detections) {
  detections_ = detections;
}

// ===========================================================================
// NMS / Post-processing
// ===========================================================================

float compute_iou(const DetectBox& a, const DetectBox& b) {
  // Convert (cx, cy, w, h) → (x1, y1, x2, y2)
  float ax1 = a.cx - a.w * 0.5f;
  float ay1 = a.cy - a.h * 0.5f;
  float ax2 = a.cx + a.w * 0.5f;
  float ay2 = a.cy + a.h * 0.5f;

  float bx1 = b.cx - b.w * 0.5f;
  float by1 = b.cy - b.h * 0.5f;
  float bx2 = b.cx + b.w * 0.5f;
  float by2 = b.cy + b.h * 0.5f;

  float inter_x1 = std::max(ax1, bx1);
  float inter_y1 = std::max(ay1, by1);
  float inter_x2 = std::min(ax2, bx2);
  float inter_y2 = std::min(ay2, by2);

  float inter_w = std::max(0.0f, inter_x2 - inter_x1);
  float inter_h = std::max(0.0f, inter_y2 - inter_y1);
  float inter_area = inter_w * inter_h;

  float area_a = a.w * a.h;
  float area_b = b.w * b.h;
  float union_area = area_a + area_b - inter_area;

  if (union_area <= 0.0f) return 0.0f;
  return inter_area / union_area;
}

std::vector<DetectBox> apply_nms(
    const std::vector<DetectBox>& boxes,
    float iou_threshold,
    float conf_threshold) {

  // 1. Filter by confidence threshold and copy survivors
  std::vector<DetectBox> filtered;
  filtered.reserve(boxes.size());
  for (const auto& box : boxes) {
    if (box.confidence >= conf_threshold) {
      filtered.push_back(box);
    }
  }

  // 2. Sort by confidence descending (NMS requires this order)
  std::sort(filtered.begin(), filtered.end(),
            [](const DetectBox& a, const DetectBox& b) {
              return a.confidence > b.confidence;
            });

  // 3. Greedy NMS: suppress overlapping boxes
  std::vector<DetectBox> kept;
  std::vector<bool> suppressed(filtered.size(), false);

  for (size_t i = 0; i < filtered.size(); ++i) {
    if (suppressed[i]) continue;

    kept.push_back(filtered[i]);

    for (size_t j = i + 1; j < filtered.size(); ++j) {
      if (suppressed[j]) continue;

      float iou = compute_iou(filtered[i], filtered[j]);
      if (iou > iou_threshold) {
        suppressed[j] = true;
      }
    }
  }

  return kept;
}
