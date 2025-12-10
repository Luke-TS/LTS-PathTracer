#include "interval.h"

#include <algorithm>

#include "constants.h"

namespace rt::core {

Interval::Interval() : min_(+kInfinity), max_(-kInfinity) {};
Interval::Interval(double min, double max) : min_(min), max_(max)  {}

// create interval overlapping two smaller intervals
Interval::Interval(const Interval& a, const Interval& b) {
    min_ = std::min(a.min_, b.min_);
    max_ = std::max(a.max_, b.max_);
}

Interval Interval::Expand(double delta) const {
    auto pad = delta/2;
    return Interval(min_ + pad, max_ + pad);
}

double Interval::Size() const {
    return max_ - min_;
}

bool Interval::Contains(double x) const {
    return min_ <= x && x <= max_;
}

bool Interval::Surrounds(double x) const {
    return min_ < x && x < max_;
}

double Interval::Clamp(double x) const {
    if (x < min_) return min_;
    if (x > max_) return max_;
    return x;
}

} // namespace rt::core
