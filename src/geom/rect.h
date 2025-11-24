#pragma once

#include "core/interval.h"
#include "hittable.h"
#include "aabb.h"

#include <memory>

namespace rt::geom {

class xy_rect : public Hittable {
public:
    xy_rect();
    xy_rect(double x0, double x1, double y0, double y1, double k, std::shared_ptr<material::Material> mat);

    bool Hit(const core::Ray& r, core::Interval ray_t, HitRecord& rec) const override;
    Aabb BoundingBox() const override;

    int TypeId() const override { return HITTABLE_SQUARE; }
    int ObjectIndex() const override { return index_; }
    void SetObjIndex(int i) override { index_ = i; }

private:
    std::shared_ptr<material::Material> mat_;
    double x0_, x1_, y0_, y1_, k_;
    int index_{};
};

class xz_rect : public Hittable {
public:
    xz_rect();
    xz_rect(double x0, double x1, double z0, double z1, double k, std::shared_ptr<material::Material> mat);

    bool Hit(const core::Ray& r, core::Interval ray_t, HitRecord& rec) const override;
    Aabb BoundingBox() const override;

    int TypeId() const override { return HITTABLE_SQUARE; }
    int ObjectIndex() const override { return index_; }
    void SetObjIndex(int i) override { index_ = i; }

private:
    std::shared_ptr<material::Material> mat_;
    double x0_, x1_, z0_, z1_, k_;
    int index_{};
};

class yz_rect : public Hittable {
public:
    yz_rect();
    yz_rect(double y0, double y1, double z0, double z1, double k, std::shared_ptr<material::Material> mat);

    bool Hit(const core::Ray& r, core::Interval ray_t, HitRecord& rec) const override;
    Aabb BoundingBox() const override;

    int TypeId() const override { return HITTABLE_SQUARE; }
    int ObjectIndex() const override { return index_; }
    void SetObjIndex(int i) override { index_ = i; }

private:
    std::shared_ptr<material::Material> mat_;
    double y0_, y1_, z0_, z1_, k_;
    int index_{};
};

} // namespace rt::geom
