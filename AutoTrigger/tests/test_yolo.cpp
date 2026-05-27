#include "autotrigger/yolo_infer.h"

#include <gtest/gtest.h>

// ===== YOLO Decode Tests (RED phase) =====
// decode_raw_output is declared in yolo_infer.h but NOT YET IMPLEMENTED.
// These tests COMPILE but will FAIL at link time (unresolved symbol).

#include <cmath>

constexpr int NUM_CELLS = 25200;
constexpr int ROW_SIZE  = 85;

static inline float sigmoid(float x) {
    return 1.0f / (1.0f + std::exp(-x));
}

// ---------------------------------------------------------------------------
// Test: SigmoidActivationCorrect — sigmoid applied to tx/ty for cx/cy decode
// ---------------------------------------------------------------------------
TEST(YoloDecodeTest, SigmoidActivationCorrect) {
    std::vector<float> raw(NUM_CELLS * ROW_SIZE, 0.0f);
    // Anchor 0 on 80×80 grid, cell (gx=10, gy=5).
    // Row index = 5 * 80 + 10 = 410
    const int row  = 410;
    const int base = row * ROW_SIZE;
    raw[base + 0] = 0.0f;   // tx
    raw[base + 1] = 0.0f;   // ty
    raw[base + 2] = 0.0f;   // tw
    raw[base + 3] = 0.0f;   // th
    raw[base + 4] = 10.0f;  // obj  (sigmoid ≈ 1)
    raw[base + 5] = 10.0f;  // cls0 (sigmoid ≈ 1)

    auto boxes = decode_raw_output(raw.data(), 0.25f);
    ASSERT_EQ(boxes.size(), 1u);

    // sigmoid(0) = 0.5
    // cx = (0.5*2 − 0.5 + 10) / 80 = 10.5 / 80 = 0.13125
    float expected_cx = (sigmoid(0.0f) * 2.0f - 0.5f + 10.0f) / 80.0f;
    float expected_cy = (sigmoid(0.0f) * 2.0f - 0.5f +  5.0f) / 80.0f;
    EXPECT_NEAR(boxes[0].cx, expected_cx, 1e-5f);
    EXPECT_NEAR(boxes[0].cy, expected_cy, 1e-5f);
    EXPECT_GT(boxes[0].confidence, 0.9f);
}

// ---------------------------------------------------------------------------
// Test: AnchorScalingCorrect — anchor dimensions scale w, h correctly
// ---------------------------------------------------------------------------
TEST(YoloDecodeTest, AnchorScalingCorrect) {
    std::vector<float> raw(NUM_CELLS * ROW_SIZE, 0.0f);
    // Anchor 0 (aw=10, ah=13) on 80×80 grid, cell (0,0), row 0
    const int base = 0;
    raw[base + 0] = 0.0f;   // tx
    raw[base + 1] = 0.0f;   // ty
    raw[base + 2] = 0.0f;   // tw → sigmoid(0)=0.5, (0.5×2)^2 = 1
    raw[base + 3] = 0.0f;   // th → same
    raw[base + 4] = 10.0f;  // obj
    raw[base + 5] = 10.0f;  // cls0

    auto boxes = decode_raw_output(raw.data(), 0.25f);
    ASSERT_EQ(boxes.size(), 1u);

    // w = 1.0 × 10 / 640 = 0.015625
    EXPECT_NEAR(boxes[0].w, 10.0f / 640.0f, 1e-5f);
    // h = 1.0 × 13 / 640 = 0.0203125
    EXPECT_NEAR(boxes[0].h, 13.0f / 640.0f, 1e-5f);
    EXPECT_GT(boxes[0].confidence, 0.9f);
}

// ---------------------------------------------------------------------------
// Test: SingleDetectionDecodedCorrectly — verify all fields for a known box
// ---------------------------------------------------------------------------
TEST(YoloDecodeTest, SingleDetectionDecodedCorrectly) {
    std::vector<float> raw(NUM_CELLS * ROW_SIZE, 0.0f);
    // Anchor 3 (aw=30, ah=61) on 40×40 grid, cell (gx=0, gy=0).
    // Row: 19200 + 0×40 + 0 = 19200
    const int row  = 19200;
    const int base = row * ROW_SIZE;
    raw[base + 0] = 0.0f;   // tx
    raw[base + 1] = 0.0f;   // ty
    raw[base + 2] = 0.0f;   // tw
    raw[base + 3] = 0.0f;   // th
    raw[base + 4] = 10.0f;  // obj
    raw[base + 5] = 10.0f;  // cls0

    auto boxes = decode_raw_output(raw.data(), 0.25f);
    ASSERT_EQ(boxes.size(), 1u);

    // Grid 40×40, stride 16
    // cx = (0.5×2 − 0.5 + 0) / 40 = 0.5/40 = 0.0125
    EXPECT_NEAR(boxes[0].cx, 0.5f / 40.0f, 1e-5f);
    EXPECT_NEAR(boxes[0].cy, 0.5f / 40.0f, 1e-5f);
    // w = 1.0 × 30 / 640 = 0.046875
    EXPECT_NEAR(boxes[0].w, 30.0f / 640.0f, 1e-5f);
    // h = 1.0 × 61 / 640 = 0.0953125
    EXPECT_NEAR(boxes[0].h, 61.0f / 640.0f, 1e-5f);
    EXPECT_GT(boxes[0].confidence, 0.9f);
    EXPECT_EQ(boxes[0].class_id, 0);
}

// ---------------------------------------------------------------------------
// Test: ConfidenceFilterRemovesLow — low confidence below threshold drops
// ---------------------------------------------------------------------------
TEST(YoloDecodeTest, ConfidenceFilterRemovesLow) {
    std::vector<float> raw(NUM_CELLS * ROW_SIZE, 0.0f);
    // Cell 0 (row 0): high confidence → kept
    {
        const int base = 0;
        raw[base + 4] = 10.0f;  // obj  → sigmoid ≈ 1
        raw[base + 5] = 10.0f;  // cls0 → sigmoid ≈ 1
    }
    // Cell 1 (row 1): very low confidence → filtered
    // obj = −5 → sigmoid(−5) ≈ 0.0067
    // All class scores 0 → max sigmoid(0) = 0.5
    // confidence ≈ 0.0067 × 0.5 ≈ 0.003  << 0.25
    {
        const int base = 1 * ROW_SIZE;
        raw[base + 4] = -5.0f;  // obj
    }

    auto boxes = decode_raw_output(raw.data(), 0.25f);
    ASSERT_EQ(boxes.size(), 1u);
    EXPECT_GT(boxes[0].confidence, 0.9f);
}

// ---------------------------------------------------------------------------
// Test: ClassIdExtractedCorrectly — best class selected from 80-way scores
// ---------------------------------------------------------------------------
TEST(YoloDecodeTest, ClassIdExtractedCorrectly) {
    std::vector<float> raw(NUM_CELLS * ROW_SIZE, 0.0f);
    // Anchor 0 on 80×80 grid, cell (0,0), row 0
    const int base = 0;
    raw[base + 0]  = 0.0f;   // tx
    raw[base + 1]  = 0.0f;   // ty
    raw[base + 2]  = 0.0f;   // tw
    raw[base + 3]  = 0.0f;   // th
    raw[base + 4]  = 10.0f;  // obj  (sigmoid ≈ 1)
    raw[base + 5]  = 1.0f;   // cls0  → sigmoid(1) ≈ 0.731
    raw[base + 20] = 5.0f;   // cls15 → sigmoid(5) ≈ 0.993  ← best!

    auto boxes = decode_raw_output(raw.data(), 0.25f);
    ASSERT_EQ(boxes.size(), 1u);

    EXPECT_EQ(boxes[0].class_id, 15);
    // confidence = sigmoid(10) × sigmoid(5) ≈ 0.99995 × 0.9933 ≈ 0.993
    EXPECT_NEAR(boxes[0].confidence, sigmoid(10.0f) * sigmoid(5.0f), 1e-3f);
}

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
