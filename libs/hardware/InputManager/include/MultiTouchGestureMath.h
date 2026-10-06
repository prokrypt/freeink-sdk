#pragma once

#include <cmath>
#include <cstdint>

namespace freeink::input_detail {

struct GesturePoint {
  int32_t x;
  int32_t y;
};

struct RotationResult {
  float degrees = 0.0f;
  uint16_t centerX = 0;
  uint16_t centerY = 0;
};

struct PinchResult {
  float scale = 1.0f;
  uint16_t centerX = 0;
  uint16_t centerY = 0;
};

inline int64_t separationSquared(const GesturePoint& first, const GesturePoint& second) {
  const int64_t dx = static_cast<int64_t>(second.x) - first.x;
  const int64_t dy = static_cast<int64_t>(second.y) - first.y;
  return dx * dx + dy * dy;
}

inline bool hasRotationScale(const GesturePoint& startFirst, const GesturePoint& startSecond,
                             const GesturePoint& currentFirst, const GesturePoint& currentSecond) {
  constexpr int64_t scalePercentSq = 100LL * 100LL;
  constexpr int64_t minScalePercentSq = 80LL * 80LL;
  constexpr int64_t maxScalePercentSq = 120LL * 120LL;
  const int64_t startSeparationSq = separationSquared(startFirst, startSecond);
  const int64_t currentSeparationSq = separationSquared(currentFirst, currentSecond);
  return currentSeparationSq * scalePercentSq >= startSeparationSq * minScalePercentSq &&
         currentSeparationSq * scalePercentSq <= startSeparationSq * maxScalePercentSq;
}

inline bool classifyRotation(const GesturePoint& startFirst, const GesturePoint& startSecond,
                             const GesturePoint& endFirst, const GesturePoint& endSecond, RotationResult& result) {
  constexpr int64_t minSeparationPxSq = 60LL * 60LL;
  constexpr float minRotationDegrees = 20.0f;
  constexpr double radiansToDegrees = 57.295779513082320876;

  const int64_t startDx = static_cast<int64_t>(startSecond.x) - startFirst.x;
  const int64_t startDy = static_cast<int64_t>(startSecond.y) - startFirst.y;
  const int64_t endDx = static_cast<int64_t>(endSecond.x) - endFirst.x;
  const int64_t endDy = static_cast<int64_t>(endSecond.y) - endFirst.y;
  const int64_t startSeparationSq = separationSquared(startFirst, startSecond);
  const int64_t endSeparationSq = separationSquared(endFirst, endSecond);

  if (startSeparationSq < minSeparationPxSq || endSeparationSq < minSeparationPxSq) return false;
  if (!hasRotationScale(startFirst, startSecond, endFirst, endSecond)) return false;

  // Panel coordinates use a downward-positive Y axis, so a positive cross
  // product is a clockwise visual rotation.
  const int64_t cross = startDx * endDy - startDy * endDx;
  const int64_t dot = startDx * endDx + startDy * endDy;
  const float degrees =
      static_cast<float>(std::atan2(static_cast<double>(cross), static_cast<double>(dot)) * radiansToDegrees);
  if (std::fabs(degrees) < minRotationDegrees) return false;

  result.degrees = degrees;
  result.centerX =
      static_cast<uint16_t>((static_cast<int64_t>(startFirst.x) + startSecond.x + endFirst.x + endSecond.x) / 4);
  result.centerY =
      static_cast<uint16_t>((static_cast<int64_t>(startFirst.y) + startSecond.y + endFirst.y + endSecond.y) / 4);
  return true;
}

inline bool classifyPinch(const GesturePoint& startFirst, const GesturePoint& startSecond, const GesturePoint& endFirst,
                          const GesturePoint& endSecond, PinchResult& result) {
  constexpr int64_t minSeparationPxSq = 60LL * 60LL;
  constexpr int64_t scalePercentSq = 100LL * 100LL;
  constexpr int64_t minPinchPercentSq = 80LL * 80LL;
  constexpr int64_t maxPinchPercentSq = 120LL * 120LL;
  constexpr float maxRotationDegrees = 15.0f;
  constexpr double radiansToDegrees = 57.295779513082320876;

  const int64_t startDx = static_cast<int64_t>(startSecond.x) - startFirst.x;
  const int64_t startDy = static_cast<int64_t>(startSecond.y) - startFirst.y;
  const int64_t endDx = static_cast<int64_t>(endSecond.x) - endFirst.x;
  const int64_t endDy = static_cast<int64_t>(endSecond.y) - endFirst.y;
  const int64_t startSeparationSq = separationSquared(startFirst, startSecond);
  const int64_t endSeparationSq = separationSquared(endFirst, endSecond);

  if (startSeparationSq < minSeparationPxSq || endSeparationSq < minSeparationPxSq) return false;

  const bool pinchedIn = endSeparationSq * scalePercentSq <= startSeparationSq * minPinchPercentSq;
  const bool pinchedOut = endSeparationSq * scalePercentSq >= startSeparationSq * maxPinchPercentSq;
  if (!pinchedIn && !pinchedOut) return false;

  const int64_t cross = startDx * endDy - startDy * endDx;
  const int64_t dot = startDx * endDx + startDy * endDy;
  const float degrees =
      static_cast<float>(std::atan2(static_cast<double>(cross), static_cast<double>(dot)) * radiansToDegrees);
  if (std::fabs(degrees) > maxRotationDegrees) return false;

  result.scale =
      static_cast<float>(std::sqrt(static_cast<double>(endSeparationSq) / static_cast<double>(startSeparationSq)));
  result.centerX =
      static_cast<uint16_t>((static_cast<int64_t>(startFirst.x) + startSecond.x + endFirst.x + endSecond.x) / 4);
  result.centerY =
      static_cast<uint16_t>((static_cast<int64_t>(startFirst.y) + startSecond.y + endFirst.y + endSecond.y) / 4);
  return true;
}

struct RotationTrackerConfig {
  float minDegrees = 20.0f;
  // The finger gap may differ from its settled starting length by this many
  // percent when the rotation is judged. Only the judged sample counts, so a
  // gap that wobbles out of band mid-twist and comes back still rotates.
  int scaleTolerancePercent = 35;
};

// Follows one two-contact sequence sample by sample. classifyRotation() only
// compares the first and last finger lines, which folds a twist past 180° back
// into the wrong direction and lets one noisy end sample decide the gesture;
// this adds up the small turn between consecutive samples instead. Fixed-size
// and allocation-free; the owner keeps the contact order stable between calls.
class RotationTracker {
 public:
  // Samples whose line replaces the anchor: the controller's first frames of a
  // fresh contact pair are the noisiest, and a deliberate twist has barely
  // begun by then.
  static constexpr uint8_t kSettleSamples = 2;
  // A finger moving farther than this in the last frame before a lift is the
  // controller's lift-off jump, not motion; release judges the frame before it.
  static constexpr int64_t kLiftJumpPx = 32;
  // No real twist turns this far between two controller frames; a larger step
  // means the contact assignment swapped, so the sequence stops being a rotation.
  static constexpr float kMaxStepDegrees = 90.0f;
  static constexpr int64_t kMinSeparationPx = 60;

  void start(const GesturePoint& first, const GesturePoint& second) {
    anchor_ = {toLine(first, second), 0.0f};
    previous_ = anchor_;
    current_ = anchor_;
    hasReference_ = isUsable(anchor_.line);
    referenceDx_ = dx(anchor_.line);
    referenceDy_ = dy(anchor_.line);
    samples_ = 0;
    valid_ = true;
  }

  // Not a rotation candidate until the next start().
  void cancel() { valid_ = false; }

  void update(const GesturePoint& first, const GesturePoint& second) {
    if (!valid_) return;
    if (samples_ < kSettleSamples) {
      const uint8_t settled = static_cast<uint8_t>(samples_ + 1);
      start(first, second);
      samples_ = settled;
      return;
    }

    Sample next = {toLine(first, second), current_.degrees};
    // A line shorter than the minimum separation has a noisy direction: skip
    // it and measure the next step from the last usable line.
    if (isUsable(next.line)) {
      if (hasReference_) {
        const float step = turnDegrees(referenceDx_, referenceDy_, dx(next.line), dy(next.line));
        if (std::fabs(step) > kMaxStepDegrees) {
          valid_ = false;
          return;
        }
        next.degrees += step;
      }
      hasReference_ = true;
      referenceDx_ = dx(next.line);
      referenceDy_ = dy(next.line);
    }
    previous_ = current_;
    current_ = next;
  }

  // Mid-gesture check against the latest sample.
  bool evaluate(const RotationTrackerConfig& config, RotationResult& result) const {
    return evaluateSample(current_, config, result);
  }

  // Check once the first contact has lifted. The lifting finger's final
  // report often jumps; judge the sample before such a jump instead.
  bool evaluateAtRelease(const RotationTrackerConfig& config, RotationResult& result) const {
    const bool jumped = movedFarther(previous_.line.x1, previous_.line.y1, current_.line.x1, current_.line.y1) ||
                        movedFarther(previous_.line.x2, previous_.line.y2, current_.line.x2, current_.line.y2);
    return evaluateSample(jumped ? previous_ : current_, config, result);
  }

 private:
  // 16-bit panel coordinates: InputManager copies this tracker into every
  // queued input sample, so it is kept small.
  struct Line {
    int16_t x1;
    int16_t y1;
    int16_t x2;
    int16_t y2;
  };
  struct Sample {
    Line line;
    float degrees;  // accumulated turn from the anchor; positive is clockwise
  };

  static Line toLine(const GesturePoint& first, const GesturePoint& second) {
    return {static_cast<int16_t>(first.x), static_cast<int16_t>(first.y), static_cast<int16_t>(second.x),
            static_cast<int16_t>(second.y)};
  }
  static int32_t dx(const Line& line) { return static_cast<int32_t>(line.x2) - line.x1; }
  static int32_t dy(const Line& line) { return static_cast<int32_t>(line.y2) - line.y1; }
  static int64_t lengthSquared(const Line& line) {
    return static_cast<int64_t>(dx(line)) * dx(line) + static_cast<int64_t>(dy(line)) * dy(line);
  }
  static bool isUsable(const Line& line) { return lengthSquared(line) >= kMinSeparationPx * kMinSeparationPx; }
  static bool movedFarther(const int32_t fromX, const int32_t fromY, const int32_t toX, const int32_t toY) {
    const int64_t moveX = toX - fromX;
    const int64_t moveY = toY - fromY;
    return moveX * moveX + moveY * moveY > kLiftJumpPx * kLiftJumpPx;
  }

  static float turnDegrees(const int64_t fromDx, const int64_t fromDy, const int64_t toDx, const int64_t toDy) {
    constexpr float radiansToDegrees = 57.29577951f;
    // Panel coordinates use a downward-positive Y axis, so a positive cross
    // product is a clockwise visual rotation.
    const int64_t cross = fromDx * toDy - fromDy * toDx;
    const int64_t dot = fromDx * toDx + fromDy * toDy;
    return std::atan2(static_cast<float>(cross), static_cast<float>(dot)) * radiansToDegrees;
  }

  bool evaluateSample(const Sample& end, const RotationTrackerConfig& config, RotationResult& result) const {
    if (!valid_ || samples_ < kSettleSamples || !isUsable(anchor_.line) || !isUsable(end.line)) return false;
    const int64_t tolerance = config.scaleTolerancePercent < 0    ? 0
                              : config.scaleTolerancePercent > 99 ? 99
                                                                  : config.scaleTolerancePercent;
    const int64_t lowPercent = 100 - tolerance;
    const int64_t highPercent = 100 + tolerance;
    const int64_t startSeparationSq = lengthSquared(anchor_.line);
    const int64_t endSeparationSq = lengthSquared(end.line);
    if (endSeparationSq * 100 * 100 < startSeparationSq * lowPercent * lowPercent ||
        endSeparationSq * 100 * 100 > startSeparationSq * highPercent * highPercent) {
      return false;
    }
    if (std::fabs(end.degrees) < config.minDegrees) return false;

    const Line& a = anchor_.line;
    const Line& e = end.line;
    result.degrees = end.degrees;
    result.centerX = static_cast<uint16_t>((static_cast<int32_t>(a.x1) + a.x2 + e.x1 + e.x2) / 4);
    result.centerY = static_cast<uint16_t>((static_cast<int32_t>(a.y1) + a.y2 + e.y1 + e.y2) / 4);
    return true;
  }

  Sample anchor_{};
  Sample previous_{};
  Sample current_{};
  int16_t referenceDx_ = 0;  // direction of the last usable line; steps are measured from it
  int16_t referenceDy_ = 0;
  uint8_t samples_ = 0;
  bool hasReference_ = false;
  bool valid_ = false;
};

}  // namespace freeink::input_detail
