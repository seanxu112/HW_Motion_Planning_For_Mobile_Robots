# HW_Motion_Planning_For_Mobile_Robots

```
void samplingOnce(Eigen::Vector3d &sample, bool ball)
  {
    if (ball)
    {
      // 在spherical coordinate里面采样
      // 用acos得到角度theta，因为theta在
      double theta = uniform_rand_pi(gen_);
      double phi = uniform_rand_2pi(gen_);
      double radius = std::pow(uniform_rand_(gen_), 1/3.);
      sample[0] = radius * sin(theta) * cos(phi);
      sample[1] = radius * sin(theta) * sin(phi);
      sample[2] = radius * cos(theta);
    }
    else
    {
      sample[0] = uniform_rand_(gen_);
      sample[1] = uniform_rand_(gen_);
      sample[2] = uniform_rand_(gen_);
      sample.array() *= range_.array();
      sample += origin_;
    }
  };
```
