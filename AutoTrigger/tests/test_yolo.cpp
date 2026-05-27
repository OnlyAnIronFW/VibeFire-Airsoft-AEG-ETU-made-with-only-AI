#include "autotrigger/yolo_infer.h"

#include <gtest/gtest.h>

// Free functions for post-processing (testable in isolation).
// These are declared in yolo_infer.h.

// ---------------------------------------------------------------------------
// Test 1: YOLOMock returns 1 person detection → values match known labels
// ---------------------------------------------------------------------------
TEST(YOLOMockTest, ReturnsSinglePersonBbox) {
  YOLOMock mock;
  // Seed the mock with a fake detection list
  std::vector<DetectBox> expected = {{0.5f, 0.5f, 0.3f, 0.6f, 0.85f, 0}};
  mock.set_detections(expected);

  ASSERT_TRUE(mock.init("mock_model.rknn"));
  ASSERT_TRUE(mock.is_ready());

  // nullptr → mock ignores input data
  auto result = mock.infer(nullptr);

  ASSERT_EQ(result.size(), 1u);
  EXPECT_FLOAT_EQ(result[0].cx, 0.5f);
  EXPECT_FLOAT_EQ(result[0].cy, 0.5f);
  EXPECT_FLOAT_EQ(result[0].w, 0.3f);
  EXPECT_FLOAT_EQ(result[0].h, 0.6f);
  EXPECT_FLOAT_EQ(result[0].confidence, 0.85f);
  EXPECT_EQ(result[0].class_id, 0);
}

// ---------------------------------------------------------------------------
// Test 2: YOLOMock returns 0 detections → empty vector
// ---------------------------------------------------------------------------
TEST(YOLOMockTest, ReturnsEmptyWhenNoDetections) {
  YOLOMock mock;
  mock.set_detections({});  // empty

  ASSERT_TRUE(mock.init("mock.rknn"));
  auto result = mock.infer(nullptr);
  EXPECT_TRUE(result.empty());
}

// ---------------------------------------------------------------------------
// Test 3: NMS — 2 overlapping boxes (IoU > 0.45) → only highest-conf kept
// ---------------------------------------------------------------------------
TEST(NmsTest, OverlappingBoxesKeepsHighestConfidence) {
  // Box A: confidence 0.9, Box B: confidence 0.7
  // A: (0.4, 0.4, 0.3, 0.3) → x1=0.25, y1=0.25, x2=0.55, y2=0.55
  // B: (0.5, 0.5, 0.3, 0.3) → x1=0.35, y1=0.35, x2=0.65, y2=0.65
  // Intersection: x1=max(0.25,0.35)=0.35, x2=min(0.55,0.65)=0.55 → w=0.20
  //              y1=max(0.25,0.35)=0.35, y2=min(0.55,0.65)=0.55 → h=0.20
  // Area intersection = 0.04, Area union = 0.09+0.09-0.04 = 0.14
  // IoU = 0.04 / 0.14 ≈ 0.2857

  // Let me create boxes with IoU definitely > 0.45:
  // A: (0.4, 0.4, 0.5, 0.5) → x1=0.15, y1=0.15, x2=0.65, y2=0.65  area=0.25
  // B: (0.5, 0.5, 0.5, 0.5) → x1=0.25, y1=0.25, x2=0.75, y2=0.75  area=0.25
  // Intersection: x1=0.25, x2=0.65 → w=0.40; y1=0.25, y2=0.65 → h=0.40
  // Area intersection = 0.16, Area union = 0.25+0.25-0.16 = 0.34
  // IoU = 0.16/0.34 ≈ 0.4706 > 0.45 ✓

  std::vector<DetectBox> boxes = {
      {0.4f, 0.4f, 0.5f, 0.5f, 0.9f, 0},   // high confidence
      {0.5f, 0.5f, 0.5f, 0.5f, 0.7f, 0},   // lower confidence, overlaps
  };

  float iou_threshold = 0.45f;
  float conf_threshold = 0.5f;
  auto result = apply_nms(boxes, iou_threshold, conf_threshold);

  ASSERT_EQ(result.size(), 1u);
  EXPECT_FLOAT_EQ(result[0].confidence, 0.9f);  // higher confidence kept
  EXPECT_FLOAT_EQ(result[0].cx, 0.4f);
}

// ---------------------------------------------------------------------------
// Test 4: NMS — 2 non-overlapping boxes (IoU < 0.45) → both kept
// ---------------------------------------------------------------------------
TEST(NmsTest, NonOverlappingBoxesBothKept) {
  // A: (0.2, 0.2, 0.1, 0.1) → x1=0.15, y1=0.15, x2=0.25, y2=0.25
  // B: (0.8, 0.8, 0.1, 0.1) → x1=0.75, y1=0.75, x2=0.85, y2=0.85
  // These are far apart: IoU = 0

  std::vector<DetectBox> boxes = {
      {0.2f, 0.2f, 0.1f, 0.1f, 0.9f, 0},
      {0.8f, 0.8f, 0.1f, 0.1f, 0.8f, 0},
  };

  auto result = apply_nms(boxes, 0.45f, 0.5f);

  ASSERT_EQ(result.size(), 2u);
}

// ---------------------------------------------------------------------------
// Test 5: Confidence threshold filtering
// ---------------------------------------------------------------------------
TEST(ConfidenceFilterTest, LowConfidenceFilteredHighConfidenceKept) {
  std::vector<DetectBox> boxes = {
      {0.3f, 0.3f, 0.2f, 0.2f, 0.4f, 0},   // below threshold → removed
      {0.6f, 0.6f, 0.2f, 0.2f, 0.6f, 0},   // above threshold → kept
  };

  float conf_threshold = 0.5f;
  auto result = apply_nms(boxes, 0.45f, conf_threshold);

  ASSERT_EQ(result.size(), 1u);
  EXPECT_FLOAT_EQ(result[0].confidence, 0.6f);
  EXPECT_FLOAT_EQ(result[0].cx, 0.6f);
}

// ---------------------------------------------------------------------------
// Test 6: DetectBox struct has correct format
// ---------------------------------------------------------------------------
TEST(DetectBoxTest, StructFieldsCorrect) {
  DetectBox box{0.25f, 0.75f, 0.1f, 0.2f, 0.95f, 0};

  EXPECT_FLOAT_EQ(box.cx, 0.25f);
  EXPECT_FLOAT_EQ(box.cy, 0.75f);
  EXPECT_FLOAT_EQ(box.w, 0.1f);
  EXPECT_FLOAT_EQ(box.h, 0.2f);
  EXPECT_FLOAT_EQ(box.confidence, 0.95f);
  EXPECT_EQ(box.class_id, 0);  // 0 = person

  // Verify normalized range [0, 1] semantics
  EXPECT_GE(box.cx, 0.0f);
  EXPECT_LE(box.cx, 1.0f);
  EXPECT_GE(box.cy, 0.0f);
  EXPECT_LE(box.cy, 1.0f);
}
