// Standalone unit test for batch-LIO intra-window de-skew (paper eq 3.44-3.47).
// Build: catkin target test_deskew. Run: ./devel/lib/batch_lio/test_deskew
#include "../src/deskew.h"
#include <Eigen/Dense>
#include <cassert>
#include <cmath>
#include <cstdio>
using namespace Eigen;

static bool close(const Vector3d& a, const Vector3d& b, double e = 1e-9) {
  return (a - b).norm() < e;
}

int main() {
  const Matrix3d I = Matrix3d::Identity();

  // 1) dt == 0  -> identity (no time gap, point already at reference time)
  assert(close(deskew_point(Vector3d(1, 2, 3), 0.0,
                            Vector3d(0.1, 0, 0), Vector3d(1, 0, 0), I),
               Vector3d(1, 2, 3)));

  // 2) zero motion (omega=0, v=0) -> identity regardless of dt
  assert(close(deskew_point(Vector3d(1, 2, 3), -0.001,
                            Vector3d(0, 0, 0), Vector3d(0, 0, 0), I),
               Vector3d(1, 2, 3)));

  // 3) pure translation, R_I = I:  p' = p + (R_I^T v) dt = p + v*dt
  assert(close(deskew_point(Vector3d(0, 0, 0), -0.01,
                            Vector3d(0, 0, 0), Vector3d(2, 0, 0), I),
               Vector3d(-0.02, 0, 0)));

  // 4) pure rotation about z by theta = wz*dt applied to (1,0,0)
  {
    const double wz = 1.0, dt = -0.5, th = wz * dt;
    Vector3d got = deskew_point(Vector3d(1, 0, 0), dt,
                                Vector3d(0, 0, wz), Vector3d(0, 0, 0), I);
    Vector3d exp(std::cos(th), std::sin(th), 0);
    assert(close(got, exp, 1e-9));
  }

  // 5) translation with non-identity R_I: v is world-frame, must be rotated into body.
  //    R_I = rotation about z by +90deg. world v=(1,0,0) -> body should see (0,-1,0)*dt contribution.
  {
    Matrix3d Rz; Rz << 0, -1, 0,  1, 0, 0,  0, 0, 1;   // +90deg about z
    double dt = -0.1;
    Vector3d got = deskew_point(Vector3d(0, 0, 0), dt,
                                Vector3d(0, 0, 0), Vector3d(1, 0, 0), Rz);
    Vector3d expv = Rz.transpose() * Vector3d(1, 0, 0) * dt;  // = (0,-1,0)*(-0.1) = (0,0.1,0)
    assert(close(got, expv));
  }

  std::printf("ALL DESKEW TESTS PASSED\n");
  return 0;
}
