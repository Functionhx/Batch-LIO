#pragma once
// batch-LIO intra-window de-skew (Point-LIWO paper eq 3.44-3.47).
//
// A LiDAR point sampled at time t_j inside a batch window is motion-compensated
// to the window's reference time t_Last (the last point's sample time) under a
// constant-velocity assumption using the EKF state's angular velocity (omega,
// in the IMU/body frame) and linear velocity (vel, in the world frame).
//
//   dt   = t_j - t_Last            (<= 0 within a window)             (3.44)
//   R_j  = Exp(omega * dt)         (rotation, axis omega/|omega|)     (3.45-3.46)
//   T_j  = R_I^T * vel * dt        (world velocity rotated into body)  (3.46, derived)
//   p'_j = R_j * p_j + T_j                                            (3.47)
//
// Frame note (see plan "Open Question"): the paper writes T_j = v*dt and calls
// the motion "world-frame", but applies it to the body-frame point p_j. Deriving
// "express the point measured at t_j as if measured at t_Last" under constant
// velocity yields the R_I^T factor used here, matching FAST-LIO UndistortPcl and
// sr_lio distortFrameByImu. R_I is G_R_I at t_Last (state rotation).

#include <Eigen/Dense>
#include <cmath>

// SO(3) exponential map via Rodrigues' formula. w is a rotation vector (axis*angle).
inline Eigen::Matrix3d so3Exp(const Eigen::Vector3d& w) {
  const double th = w.norm();
  Eigen::Matrix3d K;
  K <<     0, -w.z(),  w.y(),
       w.z(),      0, -w.x(),
      -w.y(),  w.x(),      0;
  if (th < 1e-11) return Eigen::Matrix3d::Identity() + K;  // first-order for tiny angles
  const Eigen::Matrix3d Khat = K / th;                     // skew of unit axis
  return Eigen::Matrix3d::Identity()
       + std::sin(th) * Khat
       + (1.0 - std::cos(th)) * (Khat * Khat);
}

// Returns the de-skewed body-frame point p'_j expressed at the window reference time.
inline Eigen::Vector3d deskew_point(const Eigen::Vector3d& p_body, double dt,
                                    const Eigen::Vector3d& omg,    // I_omega (body)
                                    const Eigen::Vector3d& vel,    // G_v (world)
                                    const Eigen::Matrix3d& R_I) {  // G_R_I at t_Last
  const Eigen::Matrix3d Rj = so3Exp(omg * dt);
  const Eigen::Vector3d Tj = R_I.transpose() * vel * dt;
  return Rj * p_body + Tj;
}
