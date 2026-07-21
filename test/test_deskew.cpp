// Standalone unit test for batch-LIO intra-window de-skew (paper eq 3.44-3.47).
#include <gtest/gtest.h>
#include "../src/deskew.h"
#include <Eigen/Dense>
#include <cmath>
using namespace Eigen;

static bool close(const Vector3d& a, const Vector3d& b, double e = 1e-9) {
  return (a - b).norm() < e;
}

TEST(Deskew, IdentityAtZeroDt) {
  const Matrix3d I = Matrix3d::Identity();
  EXPECT_TRUE(close(deskew_point(Vector3d(1, 2, 3), 0.0,
                                 Vector3d(0.1, 0, 0), Vector3d(1, 0, 0), I),
                    Vector3d(1, 2, 3)));
}

TEST(Deskew, ZeroMotionIsIdentity) {
  const Matrix3d I = Matrix3d::Identity();
  EXPECT_TRUE(close(deskew_point(Vector3d(1, 2, 3), -0.001,
                                 Vector3d(0, 0, 0), Vector3d(0, 0, 0), I),
                    Vector3d(1, 2, 3)));
}

TEST(Deskew, PureTranslation) {
  const Matrix3d I = Matrix3d::Identity();
  EXPECT_TRUE(close(deskew_point(Vector3d(0, 0, 0), -0.01,
                                 Vector3d(0, 0, 0), Vector3d(2, 0, 0), I),
                    Vector3d(-0.02, 0, 0)));
}

TEST(Deskew, PureRotationAboutZ) {
  const Matrix3d I = Matrix3d::Identity();
  const double wz = 1.0, dt = -0.5, th = wz * dt;
  Vector3d got = deskew_point(Vector3d(1, 0, 0), dt,
                              Vector3d(0, 0, wz), Vector3d(0, 0, 0), I);
  Vector3d exp(std::cos(th), std::sin(th), 0);
  EXPECT_TRUE(close(got, exp, 1e-9));
}

TEST(Deskew, TranslationRotatesWorldVelIntoBody) {
  Matrix3d Rz; Rz << 0, -1, 0,  1, 0, 0,  0, 0, 1;   // +90deg about z
  double dt = -0.1;
  Vector3d got = deskew_point(Vector3d(0, 0, 0), dt,
                              Vector3d(0, 0, 0), Vector3d(1, 0, 0), Rz);
  Vector3d expv = Rz.transpose() * Vector3d(1, 0, 0) * dt;
  EXPECT_TRUE(close(got, expv));
}
