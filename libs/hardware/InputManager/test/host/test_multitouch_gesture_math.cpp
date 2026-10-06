#include <cmath>
#include <cstdio>

#include "../../include/MultiTouchGestureMath.h"

namespace {

using freeink::input_detail::classifyPinch;
using freeink::input_detail::classifyRotation;
using freeink::input_detail::GesturePoint;
using freeink::input_detail::hasRotationScale;
using freeink::input_detail::PinchResult;
using freeink::input_detail::RotationResult;
using freeink::input_detail::RotationTracker;
using freeink::input_detail::RotationTrackerConfig;

int checksRun = 0;
int checksFailed = 0;

#define CHECK(condition)                                               \
  do {                                                                 \
    ++checksRun;                                                       \
    if (!(condition)) {                                                \
      ++checksFailed;                                                  \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #condition); \
    }                                                                  \
  } while (0)

bool near(const float actual, const float expected, const float tolerance = 0.1f) {
  return std::fabs(actual - expected) <= tolerance;
}

void testClockwiseAndCounterClockwise() {
  RotationResult result;
  CHECK(classifyRotation({0, 0}, {100, 0}, {50, -50}, {50, 50}, result));
  CHECK(near(result.degrees, 90.0f));
  CHECK(result.centerX == 50);
  CHECK(result.centerY == 0);

  CHECK(classifyRotation({0, 0}, {100, 0}, {50, 50}, {50, -50}, result));
  CHECK(near(result.degrees, -90.0f));
}

void testAngleWraparound() {
  RotationResult result;
  CHECK(!classifyRotation({200, 200}, {102, 217}, {200, 200}, {102, 183}, result));
  CHECK(classifyRotation({200, 200}, {102, 217}, {200, 200}, {106, 166}, result));
  CHECK(result.degrees > 20.0f && result.degrees < 30.0f);
}

void testContactOrderIndependence() {
  RotationResult forward;
  RotationResult reversed;
  CHECK(classifyRotation({0, 0}, {100, 0}, {50, -50}, {50, 50}, forward));
  CHECK(classifyRotation({100, 0}, {0, 0}, {50, 50}, {50, -50}, reversed));
  CHECK(near(forward.degrees, reversed.degrees));
}

void testRejectionThresholds() {
  RotationResult result;
  CHECK(!classifyRotation({0, 0}, {100, 0}, {2, -17}, {98, 17},
                          result));  // Below 20 degrees.
  CHECK(!classifyRotation({0, 0}, {50, 0}, {25, -25}, {25, 25},
                          result));  // Span below 60 px.
  CHECK(!classifyRotation({0, 0}, {100, 0}, {-10, -10}, {110, 10},
                          result));  // Pinch/spread over 20%.
  CHECK(!classifyRotation({0, 0}, {100, 0}, {80, 0}, {180, 0},
                          result));  // Translation only.
}

void testScaleEligibilityCanLatchIntermediatePinch() {
  CHECK(hasRotationScale({0, 0}, {100, 0}, {0, 0}, {80, 0}));
  CHECK(hasRotationScale({0, 0}, {100, 0}, {0, 0}, {120, 0}));
  CHECK(!hasRotationScale({0, 0}, {100, 0}, {0, 0}, {79, 0}));
  CHECK(!hasRotationScale({0, 0}, {100, 0}, {0, 0}, {121, 0}));
}

void testRotationWinsWithTranslation() {
  RotationResult result;
  CHECK(classifyRotation({0, 0}, {100, 0}, {130, 50}, {130, 150}, result));
  CHECK(near(result.degrees, 90.0f));
  CHECK(result.centerX == 90);
  CHECK(result.centerY == 50);
}

void testPinchInAndOut() {
  PinchResult result;
  CHECK(classifyPinch({0, 0}, {100, 0}, {10, 0}, {70, 0}, result));
  CHECK(near(result.scale, 0.6f, 0.01f));
  CHECK(result.centerX == 45);
  CHECK(result.centerY == 0);

  CHECK(classifyPinch({0, 0}, {100, 0}, {-20, 0}, {140, 0}, result));
  CHECK(near(result.scale, 1.6f, 0.01f));
  CHECK(result.centerX == 55);
}

void testPinchRejectionThresholds() {
  PinchResult result;
  CHECK(!classifyPinch({0, 0}, {100, 0}, {5, 0}, {95, 0}, result));      // Small scale change.
  CHECK(!classifyPinch({0, 0}, {50, 0}, {0, 0}, {90, 0}, result));       // Start span below 60 px.
  CHECK(!classifyPinch({0, 0}, {100, 0}, {50, -50}, {50, 90}, result));  // Rotation too large.
  CHECK(!classifyPinch({0, 0}, {100, 0}, {60, 80}, {160, 80}, result));  // Translation only.
}

// The two classifiers are gated on the same separation band from opposite
// sides, so no gesture can be accepted by both. Each of these is accepted by
// one and must be rejected by the other.
void testPinchAndRotationAreMutuallyExclusive() {
  RotationResult rotation;
  PinchResult pinch;

  // A pure 90-degree turn holds the separation, so the pinch gate rejects it.
  CHECK(classifyRotation({0, 0}, {100, 0}, {50, -50}, {50, 50}, rotation));
  CHECK(!classifyPinch({0, 0}, {100, 0}, {50, -50}, {50, 50}, pinch));

  // A pure 40% close leaves the band, so the rotation gate rejects it.
  CHECK(classifyPinch({0, 0}, {100, 0}, {10, 0}, {70, 0}, pinch));
  CHECK(!classifyRotation({0, 0}, {100, 0}, {10, 0}, {70, 0}, rotation));
}

// Contacts that converge by exactly the 20% threshold while both also travel
// 80 px. The translation path tolerates 45 px of separation change, so it would
// accept this as a two-finger swipe; finishMultiTouchGesture() tries pinch
// first, which is what makes it a pinch.
void testPinchWinsWithTranslation() {
  PinchResult result;
  CHECK(classifyPinch({0, 0}, {100, 0}, {80, 0}, {160, 0}, result));
  CHECK(near(result.scale, 0.8f, 0.01f));
  CHECK(result.centerX == 85);
  CHECK(result.centerY == 0);
}


// Two fingers on a line through (cx, cy), `halfSpan` px from it, turned
// `degrees` clockwise on screen (downward-positive Y).
struct FingerPair {
  GesturePoint first;
  GesturePoint second;
};

FingerPair twisted(const float degrees, const float halfSpan, const int cx = 300, const int cy = 300) {
  const float radians = degrees * 3.14159265f / 180.0f;
  const int dx = static_cast<int>(std::lround(std::cos(radians) * halfSpan));
  const int dy = static_cast<int>(std::lround(std::sin(radians) * halfSpan));
  return {{cx - dx, cy - dy}, {cx + dx, cy + dy}};
}

// Starts a tracker and feeds it through the settle frames at the start pose.
RotationTracker settledTracker(const FingerPair& pose) {
  RotationTracker tracker;
  tracker.start(pose.first, pose.second);
  for (uint8_t i = 0; i < RotationTracker::kSettleSamples; ++i) tracker.update(pose.first, pose.second);
  return tracker;
}

// Turns from `from` to `to` degrees in `steps` equal samples.
void sweep(RotationTracker& tracker, const float from, const float to, const int steps, const float halfSpan = 80.0f) {
  for (int i = 1; i <= steps; ++i) {
    const FingerPair pose = twisted(from + (to - from) * static_cast<float>(i) / static_cast<float>(steps), halfSpan);
    tracker.update(pose.first, pose.second);
  }
}

void testTrackerFollowsDirection() {
  const RotationTrackerConfig config;
  RotationResult result;

  RotationTracker clockwise = settledTracker(twisted(0, 80));
  sweep(clockwise, 0, 45, 15);
  CHECK(clockwise.evaluateAtRelease(config, result));
  CHECK(near(result.degrees, 45.0f, 0.5f));
  CHECK(result.centerX == 300 && result.centerY == 300);

  RotationTracker counter = settledTracker(twisted(0, 80));
  sweep(counter, 0, -45, 15);
  CHECK(counter.evaluateAtRelease(config, result));
  CHECK(near(result.degrees, -45.0f, 0.5f));
}

// The endpoint classifier folds a 200° twist into -160°; the tracker keeps
// adding and reports the direction the fingers actually turned.
void testTrackerKeepsDirectionPast180() {
  const RotationTrackerConfig config;
  RotationResult result;
  const FingerPair start = twisted(0, 80);
  const FingerPair end = twisted(200, 80);
  CHECK(classifyRotation(start.first, start.second, end.first, end.second, result));
  CHECK(result.degrees < 0.0f);

  RotationTracker tracker = settledTracker(start);
  sweep(tracker, 0, 200, 40);
  CHECK(tracker.evaluateAtRelease(config, result));
  CHECK(near(result.degrees, 200.0f, 1.0f));
}

void testTrackerTwistAndPartialReturnNets() {
  const RotationTrackerConfig config;
  RotationResult result;
  RotationTracker tracker = settledTracker(twisted(0, 80));
  sweep(tracker, 0, 40, 10);
  sweep(tracker, 40, 10, 10);
  CHECK(!tracker.evaluateAtRelease(config, result));  // net 10°: below threshold
}

// The old classifier latched the gesture off the moment the finger gap left
// 80-120%. A gap that swings out and comes back must still rotate.
void testTrackerSurvivesMidGestureGapWobble() {
  const RotationTrackerConfig config;
  RotationResult result;
  RotationTracker tracker = settledTracker(twisted(0, 80));
  sweep(tracker, 0, 15, 5, 80.0f);
  sweep(tracker, 15, 25, 3, 50.0f);  // gap shrinks to 62%
  CHECK(!tracker.evaluate(config, result));
  sweep(tracker, 25, 40, 5, 78.0f);  // and comes back
  CHECK(tracker.evaluate(config, result));
  CHECK(near(result.degrees, 40.0f, 1.0f));
}

void testTrackerScaleToleranceAtJudgement() {
  RotationTrackerConfig config;
  RotationResult result;
  RotationTracker tracker = settledTracker(twisted(0, 100));
  sweep(tracker, 0, 45, 15, 70.0f);  // ends at 70% of the starting gap
  CHECK(tracker.evaluateAtRelease(config, result));
  config.scaleTolerancePercent = 20;
  CHECK(!tracker.evaluateAtRelease(config, result));

  RotationTracker spread = settledTracker(twisted(0, 80));
  sweep(spread, 0, 45, 15, 120.0f);  // 150%: past the default 35% band
  CHECK(!spread.evaluateAtRelease(RotationTrackerConfig{}, result));
}

void testTrackerThresholdIsConfigurable() {
  RotationTrackerConfig config;
  RotationResult result;
  RotationTracker tracker = settledTracker(twisted(0, 80));
  sweep(tracker, 0, 25, 10);
  CHECK(tracker.evaluate(config, result));
  config.minDegrees = 30.0f;
  CHECK(!tracker.evaluate(config, result));
}

// The anchor is the line after the settle frames, so a jittery first report
// neither adds nor removes twist.
void testTrackerReanchorsAfterSettle() {
  const RotationTrackerConfig config;
  RotationResult result;
  RotationTracker tracker;
  const FingerPair noisy = twisted(-15, 80);
  const FingerPair settled = twisted(0, 80);
  tracker.start(noisy.first, noisy.second);
  for (uint8_t i = 0; i < RotationTracker::kSettleSamples; ++i) tracker.update(settled.first, settled.second);
  sweep(tracker, 0, 18, 6);
  CHECK(!tracker.evaluateAtRelease(config, result));  // 18° from the settled line, not 33° from the noisy one
  sweep(tracker, 18, 24, 2);
  CHECK(tracker.evaluateAtRelease(config, result));
  CHECK(near(result.degrees, 24.0f, 0.5f));
}

void testTrackerIgnoresLiftOffJump() {
  const RotationTrackerConfig config;
  RotationResult result;
  RotationTracker tracker = settledTracker(twisted(0, 80));
  sweep(tracker, 0, 30, 10);
  // The lifting finger's last report leaps back toward its start.
  const FingerPair turned = twisted(30, 80);
  tracker.update(turned.first, {turned.second.x - 10, turned.second.y - 45});
  CHECK(tracker.evaluateAtRelease(config, result));
  CHECK(near(result.degrees, 30.0f, 0.5f));
  // Mid-gesture checks still see the latest sample.
  CHECK(!tracker.evaluate(config, result) || !near(result.degrees, 30.0f, 0.5f));
}

void testTrackerRejectsSwappedContacts() {
  const RotationTrackerConfig config;
  RotationResult result;
  RotationTracker tracker = settledTracker(twisted(0, 80));
  sweep(tracker, 0, 30, 10);
  const FingerPair turned = twisted(30, 80);
  tracker.update(turned.second, turned.first);  // assignment flipped: a 180° step
  sweep(tracker, 210, 220, 2);
  CHECK(!tracker.evaluateAtRelease(config, result));
}

// Thumb fixed, index finger arcing around it.
void testTrackerPivotTwist() {
  const RotationTrackerConfig config;
  RotationResult result;
  RotationTracker tracker;
  const GesturePoint thumb = {200, 400};
  tracker.start(thumb, {350, 400});
  for (uint8_t i = 0; i < RotationTracker::kSettleSamples; ++i) tracker.update(thumb, {350, 400});
  for (int i = 1; i <= 12; ++i) {
    const float radians = -3.0f * static_cast<float>(i) * 3.14159265f / 180.0f;
    tracker.update(thumb, {200 + static_cast<int>(std::lround(150 * std::cos(radians))),
                           400 + static_cast<int>(std::lround(150 * std::sin(radians)))});
  }
  CHECK(tracker.evaluateAtRelease(config, result));
  CHECK(near(result.degrees, -36.0f, 0.5f));
}

void testTrackerShortLinesDoNotAccumulateNoise() {
  const RotationTrackerConfig config;
  RotationResult result;
  RotationTracker tracker = settledTracker(twisted(0, 80));
  sweep(tracker, 0, 10, 5);
  // Fingers pass close together: 20 px apart lines say nothing reliable.
  tracker.update({300, 300}, {300, 320});
  tracker.update({300, 300}, {320, 300});
  sweep(tracker, 10, 15, 2);
  CHECK(!tracker.evaluateAtRelease(config, result));
}

void testTrackerAndPinchStayExclusive() {
  const RotationTrackerConfig config;
  RotationResult rotation;
  PinchResult pinch;
  // A straight 40% close never turns, so the tracker rejects what pinch accepts.
  RotationTracker tracker = settledTracker({{0, 300}, {100, 300}});
  tracker.update({10, 300}, {70, 300});
  CHECK(!tracker.evaluateAtRelease(config, rotation));
  CHECK(classifyPinch({0, 300}, {100, 300}, {10, 300}, {70, 300}, pinch));
}

}  // namespace

int main() {
  testClockwiseAndCounterClockwise();
  testAngleWraparound();
  testContactOrderIndependence();
  testRejectionThresholds();
  testScaleEligibilityCanLatchIntermediatePinch();
  testRotationWinsWithTranslation();
  testPinchInAndOut();
  testPinchRejectionThresholds();
  testPinchAndRotationAreMutuallyExclusive();
  testPinchWinsWithTranslation();
  testTrackerFollowsDirection();
  testTrackerKeepsDirectionPast180();
  testTrackerTwistAndPartialReturnNets();
  testTrackerSurvivesMidGestureGapWobble();
  testTrackerScaleToleranceAtJudgement();
  testTrackerThresholdIsConfigurable();
  testTrackerReanchorsAfterSettle();
  testTrackerIgnoresLiftOffJump();
  testTrackerRejectsSwappedContacts();
  testTrackerPivotTwist();
  testTrackerShortLinesDoNotAccumulateNoise();
  testTrackerAndPinchStayExclusive();

  std::printf("%d checks, %d failures\n", checksRun, checksFailed);
  return checksFailed == 0 ? 0 : 1;
}
