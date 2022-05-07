# HW_Motion_Planning_For_Mobile_Robots
<!-- ![alt text](RRT*.png) -->
<table>
  <td> <img src="mpc_1.gif" width="300" height="200" /> </td>
  <td> <img src="mpc_2.gif" width="300" height="200" /> </td>
  <td> <img src="mpc_3.gif" width="200" height="300" /> </td>
</table>

```
void linearization(const double& phi,
                     const double& v,
                     const double& delta) {
// TODO: set values to Ad_, Bd_, gd_
// ...
  Ad_ << 1,    0, -dt_*v*sin(phi),       dt_*cos(phi),
         0,    1,  dt_*v*cos(phi),       dt_*sin(phi),
         0,    0,               1, dt_*tan(delta)/ll_,
         0,    0,               0,                  1;
  Bd_ <<   0,                               0,
           0,                               0,
           0, dt_*v/ll_/cos(delta)/cos(delta),
         dt_,                               0;
  gd_ <<                    dt_*v*phi*sin(phi),
                           -dt_*v*phi*cos(phi),
        -dt_*v*delta/ll_/cos(delta)/cos(delta),
                                             0;
  return;
}
```

The computed linearized system model based on state and input

```
for (int i = 0; i < N_; ++i) {
      // TODO: set stage constraints of inputs (a, delta, ddelta)
      // -a_max <= a <= a_max for instance:
      Cu_.coeffRef(i * 3 + 0, i * m + 0) = 1;
      lu_.coeffRef(i * 3 + 0, 0) = -a_max_;
      uu_.coeffRef(i * 3 + 0, 0) = a_max_;

      Cu_.coeffRef(i * 3 + 1, i * m + 1) = 1;
      lu_.coeffRef(i * 3 + 1, 0) = -delta_max_;
      uu_.coeffRef(i * 3 + 1, 0) = delta_max_;

      if (i != 0)
      {
        Cu_.coeffRef(i * 3 + 2, (i) * m + 1) = 1;
        Cu_.coeffRef(i * 3 + 2, (i-1) * m + 1) = -1;
        lu_.coeffRef(i * 3 + 2, 0) = -ddelta_max_ * dt_;
        uu_.coeffRef(i * 3 + 2, 0) = ddelta_max_ * dt_;
      }
      else
      {
        Cu_.coeffRef(2, 1) = 1;
      }
      
      // // TODO: set stage constraints of states (v)
      // // -v_max <= v <= v_max
      Cx_.coeffRef(i, i * n + 3) = 1;
      lx_.coeffRef(i, 0) = -0.1;
      ux_.coeffRef(i, 0) = v_max_;
}
```

Setting up the constraints. Ddelta is a differential constraint, so we have to treat it differently. We also have to treat the first set of constraints of ddelta differently, since some code below sets up a unique constraints on ddelta based on the previous input.

```
if (i == 0) {
        BB.block(0, 0, n, m) = Bd_;
        AA.block(0, 0, n, n) = Ad_;
        gg.block(0, 0, n, 1) = gd_;
      } else {
        // TODO: set BB AA gg

        for (int j=0; j < i; j++)
          BB.block(i*n, j*m, n, m) = Ad_*BB.block((i-1)*n, j*m, n, m);
        BB.block(i*n, i*m, n, m) = Bd_;
        AA.block(i*n, 0, n, n) = Ad_ * AA.block(n*(i-1), 0, n, n);
        gg.block(i*n, 0, n, 1) = Ad_ * gg.block((i-1)*n, 0, n, 1) + gd_;
      }
      // TODO: set qx
      Eigen::Vector2d xy = s_(s0); // reference (x_r, y_r)
      // qx.coeffRef(...
      // ...
      qx.coeffRef(i*n, 0) = -xy[0];
      qx.coeffRef(i*n+1, 0) = -xy[1];
      qx.coeffRef(i*n+2, 0) = -last_phi;
```

Setting up the Big Matrix for QP solver from the sub matrix we got from linearization method. Since the system is non-linear, we have to use different A for constructing AA, BB and gg. e.g. instead of A^4, it should be A4 * A3* A2 * A1.

### Compensate Delay
```
VectorX compensateDelay(const VectorX& x0) {
  VectorX x0_delay = x0;
  // TODO: compensate delay
  // ...
  // reduced time step for better estimation
  // for (int i = 0; i < std::floor(delay_ / 0.001); i++)
  // {
  //   int index = std::floor(0.001*i/dt_);
  //   std::cout << "index: " << index << std::endl;
  //   step(x0_delay, historyInput_.at(index), 0.001);
  // }
  // step(x0_delay, historyInput_.back(), delay_ - 0.001 * std::floor(delay_ / 0.001));

  for (int i = 0; i < std::floor(delay_ / dt_); i++)
    step(x0_delay, historyInput_.at(i), dt_);
  step(x0_delay, historyInput_.back(), delay_ - dt_ * std::floor(delay_ / dt_));
  return x0_delay;
}
```
Bascally takes the historyInput_ from previous optimization and step the system forward to get the initial controllable state.
To make sure there are no issue with the Numerical stability / accuracy of Runge–Kutta methods, I tried to use a much smaller step than dt_, the result is similar than the basic step

### For second trajectory

I modify the ddelta_max_ since it seems to be too low for turning in this trajectory. And based on the tests, it is indeed the issue with ddelta_max. We are travelling way too fast for the robot to turn based on the original constraint of 1.3. Of course we can reduce the v_desired so delta can catch up as well.
