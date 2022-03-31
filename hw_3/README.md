# HW_Motion_Planning_For_Mobile_Robots

```
for (auto &curr_node : neighbour_nodes)
{
  if (!map_ptr_->isSegmentValid(curr_node->x, x_new))
    continue;
  double temp_dist = curr_node->cost_from_start + calDist(curr_node->x, x_new);
  if (temp_dist < min_dist_from_start)
  {
    min_node = curr_node;
    min_dist_from_start = temp_dist;
    cost_from_p = calDist(curr_node->x, x_new);
  }
}
```

```
for (auto &curr_node : neighbour_nodes)
{
  double best_cost_before_rewire = goal_node_->cost_from_start;
  // ! -------------------------------------
  if (!map_ptr_->isSegmentValid(curr_node->x, x_new))
    continue;
  double temp_dist = new_node->cost_from_start + calDist(new_node->x, curr_node->x);
  if (temp_dist < curr_node->cost_from_start)
    changeNodeParent(curr_node, new_node, calDist(new_node->x, curr_node->x));

  // ! -------------------------------------
  if (best_cost_before_rewire > goal_node_->cost_from_start)
  {
    vector<Eigen::Vector3d> curr_best_path;
    fillPath(goal_node_, curr_best_path);
    path_list_.emplace_back(curr_best_path);
    solution_cost_time_pair_list_.emplace_back(goal_node_->cost_from_start, (ros::Time::now() - rrt_start_time).toSec());
  }
}
```

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
