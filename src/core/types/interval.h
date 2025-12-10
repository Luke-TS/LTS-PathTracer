#pragma once

namespace rt::core {

class Interval {
public:
    double min_, max_;

    Interval();
    Interval(double min, double max);

    // overlapping interval
    Interval(const Interval& a, const Interval& b);

    Interval Expand(double delta) const;

    double Size() const;

    bool Contains(double x) const;

    bool Surrounds(double x) const;

    double Clamp(double x) const;
};

} // namespace rt::core
