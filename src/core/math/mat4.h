#pragma once
#include <cmath>
#include "core/vec3.h"

namespace rt::core {

class Mat4 {
public:
    double m[4][4];

    // ----------------------------------------
    // Constructors
    // ----------------------------------------
    Mat4() {
        for (int i=0;i<4;i++)
            for (int j=0;j<4;j++)
                m[i][j] = (i==j ? 1.0 : 0.0);
    }

    explicit Mat4(double val) {
        for (int i=0;i<4;i++)
            for (int j=0;j<4;j++)
                m[i][j] = val;
    }

    static Mat4 Identity() {
        return Mat4();
    }

    // ----------------------------------------
    // Matrix multiplication
    // ----------------------------------------
    Mat4 operator*(const Mat4& o) const {
        Mat4 r(0.0);
        for (int i=0;i<4;i++) {
            for (int j=0;j<4;j++) {
                r.m[i][j] =
                    m[i][0] * o.m[0][j] +
                    m[i][1] * o.m[1][j] +
                    m[i][2] * o.m[2][j] +
                    m[i][3] * o.m[3][j];
            }
        }
        return r;
    }

    // ----------------------------------------
    // Transform a point (Vec3)
    // ----------------------------------------
    Vec3 operator*(const Vec3& v) const {
        double x = v.x(), y = v.y(), z = v.z();
        double xp = m[0][0]*x + m[0][1]*y + m[0][2]*z + m[0][3];
        double yp = m[1][0]*x + m[1][1]*y + m[1][2]*z + m[1][3];
        double zp = m[2][0]*x + m[2][1]*y + m[2][2]*z + m[2][3];
        double wp = m[3][0]*x + m[3][1]*y + m[3][2]*z + m[3][3];

        if (wp != 1.0 && wp != 0.0) {
            xp /= wp; yp /= wp; zp /= wp;
        }
        return Vec3(xp, yp, zp);
    }

    // ----------------------------------------
    // Transform a direction (Vec3)
    // (Ignore translation; w = 0)
    // ----------------------------------------
    Vec3 TransformDirection(const Vec3& v) const {
        double x = v.x(), y = v.y(), z = v.z();
        double xp = m[0][0]*x + m[0][1]*y + m[0][2]*z;
        double yp = m[1][0]*x + m[1][1]*y + m[1][2]*z;
        double zp = m[2][0]*x + m[2][1]*y + m[2][2]*z;
        return Vec3(xp, yp, zp);
    }

    // ----------------------------------------
    // Matrix transpose
    // ----------------------------------------
    Mat4 Transpose() const {
        Mat4 r(0.0);
        for (int i=0;i<4;i++)
            for (int j=0;j<4;j++)
                r.m[i][j] = m[j][i];
        return r;
    }

    // ----------------------------------------
    // Matrix inverse (general 4×4)
    // Optimized for transform matrices (affine)
    // ----------------------------------------
    Mat4 Inverse() const {
        Mat4 r;

        // Upper-left 3×3 (rotation+scale)
        double det =
            m[0][0] * (m[1][1]*m[2][2] - m[1][2]*m[2][1]) -
            m[0][1] * (m[1][0]*m[2][2] - m[1][2]*m[2][0]) +
            m[0][2] * (m[1][0]*m[2][1] - m[1][1]*m[2][0]);

        if (std::fabs(det) < 1e-12)
            throw std::runtime_error("Non-invertible Mat4");

        double invDet = 1.0 / det;

        // Inverse 3x3
        r.m[0][0] =  (m[1][1]*m[2][2] - m[1][2]*m[2][1]) * invDet;
        r.m[0][1] = -(m[0][1]*m[2][2] - m[0][2]*m[2][1]) * invDet;
        r.m[0][2] =  (m[0][1]*m[1][2] - m[0][2]*m[1][1]) * invDet;

        r.m[1][0] = -(m[1][0]*m[2][2] - m[1][2]*m[2][0]) * invDet;
        r.m[1][1] =  (m[0][0]*m[2][2] - m[0][2]*m[2][0]) * invDet;
        r.m[1][2] = -(m[0][0]*m[1][2] - m[0][2]*m[1][0]) * invDet;

        r.m[2][0] =  (m[1][0]*m[2][1] - m[1][1]*m[2][0]) * invDet;
        r.m[2][1] = -(m[0][0]*m[2][1] - m[0][1]*m[2][0]) * invDet;
        r.m[2][2] =  (m[0][0]*m[1][1] - m[0][1]*m[1][0]) * invDet;

        // Translation
        Vec3 t(m[0][3], m[1][3], m[2][3]);
        Vec3 ti = r.TransformDirection(-t);

        r.m[0][3] = ti.x();
        r.m[1][3] = ti.y();
        r.m[2][3] = ti.z();

        // Last row
        r.m[3][0] = r.m[3][1] = r.m[3][2] = 0.0;
        r.m[3][3] = 1.0;

        return r;
    }


    // ----------------------------------------
    // STATIC CONSTRUCTORS
    // ----------------------------------------

    static Mat4 Translate(const Vec3& t) {
        Mat4 r = Identity();
        r.m[0][3] = t.x();
        r.m[1][3] = t.y();
        r.m[2][3] = t.z();
        return r;
    }

    static Mat4 Scale(double s) {
        Mat4 r = Identity();
        r.m[0][0] = s;
        r.m[1][1] = s;
        r.m[2][2] = s;
        return r;
    }

    static Mat4 Scale(const Vec3& s) {
        Mat4 r = Identity();
        r.m[0][0] = s.x();
        r.m[1][1] = s.y();
        r.m[2][2] = s.z();
        return r;
    }

    static Mat4 RotateY(double radians) {
        Mat4 r = Identity();
        double c = std::cos(radians);
        double s = std::sin(radians);
        r.m[0][0] = c;
        r.m[0][2] = s;
        r.m[2][0] = -s;
        r.m[2][2] = c;
        return r;
    }
};

} // namespace rt::core

