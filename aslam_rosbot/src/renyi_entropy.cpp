/*Active SLAM in Crowded Enviroments_MS_Thesis   -> Actve SLAM Master Thesis*/

bool GraphOptimiser::utilityCalcServiceCallback(
  crowdbot_active_slam::utility_calc::Request &request,
  crowdbot_active_slam::utility_calc::Response &response){
  // ISAM2

  // Create a path msg of the graph node estimates
  nav_msgs::Path action_path;
  action_path.header.frame_id = "/map";
  Pose2 tmp_pose2;

  // Iterate over all node estimates
  int node_size = action_estimates.size() - lc_counter;
  for (int i = 0; i < node_size; i++){
    // Cast Pose2 from Value
    tmp_pose2 = *dynamic_cast<const Pose2*>(&action_estimates.at(i));

    // Create PoseStamped variables from Pose 2 and add to path
    action_path.poses.push_back(pose2ToPoseStamped(tmp_pose2));
  }

  // Publish the graph
  action_path_pub_.publish(action_path);

  // Calculate alpha for each node
  std::vector<double> alpha;
  double sigma_temp;

  for (int i = 0; i < node_size; i++){
    // D-optimality
    Eigen::VectorXcd eivals = isam.marginalCovariance(i).eigenvalues();
    double sum_of_logs = log(eivals[0].real()) +
                         log(eivals[1].real()) +
                         log(eivals[2].real());
    sigma_temp = exp(1.0 / 3.0 * sum_of_logs);
    alpha.push_back(1.0 + sigma_norm_ / (sigma_temp));
  }

  std::map<int, int> subset;
  getSubsetOfMap(action_path, subset);

  // Calculate utility
  double utility = 0;
  for (std::map<int, int>::iterator it = subset.begin(); it != subset.end(); ++it){
    // Get probability
    int p_percent = int(occupancy_grid_msg_.data[it->first]);
    // Check if cell is unkonwn
    if (p_percent == -1) p_percent = 50;
    // Check if not 0 or 1 to avoid nan's
    if (p_percent != 0 && p_percent != 100){
      double p = double(p_percent) / 100.0;
      if (alpha[it->second] > 1000){
        utility += -(p * log2(p) + (1 - p) * log2(1 - p)) + log2(std::max(p, 1 - p));
      }
      else{
        utility += -(p * log2(p) + (1 - p) * log2(1 - p)) -
        1.0 / (1.0 - alpha[it->second]) * log2(pow(p, alpha[it->second]) + pow(1 - p, alpha[it->second]));
      }
    }
  }

  // normalize utility by path path_length
  if (request.exploration_type == "utility_normalized"){
    utility = utility / path_length;
  }
  response.utility = utility;
  return true;
}



void GraphOptimiser::updateLogOddsWithBresenham(int x0, int y0, int x1, int y1,
                               std::vector<int> end_point_index_m, bool update){
  // Bresenham's line algorithm (Get indexes between robot pose and scans)
  // starting from "https://github.com/lama-imr/lama_utilities/blob/indigo-devel/map_ray_caster/src/map_ray_caster.cpp"
  // "https://csustan.csustan.edu/~tom/Lecture-Notes/Graphics/Bresenham-Line/Bresenham-Line.pdf"

  int dx = x1 - x0;
  int dy = y1 - y0;
  int xstep = 1;
  int ystep = 1;

  // Check if dx, dy are negative (direction changes)
  if (dx < 0) {dx = -dx; xstep = -1;};
  if (dy < 0) {dy = -dy; ystep = -1;};

  // Calculate variable for performance improvement
  int twodx = 2 * dx;
  int twody = 2 * dy;

  // Check if gradient < 1
  if (dx > dy){
    int fraction_increment = 2 * dy;
    int fraction = 2 * dy - dx;
    int x = x0 + xstep;
    int y = y0;
    for (; x != x1; x += xstep){
      fraction += fraction_increment;
      if (fraction >= 0){
        y += ystep;
        fraction -= twodx;
      }
      updateLogOddsArrayAndMapAsFree(x, y, end_point_index_m, update);
    }
  }
  else {
    int fraction_increment = 2 * dx;
    int fraction = 2 * dx - dy;
    int x = x0;
    int y = y0 + ystep;
    for (; y != y1; y += ystep){
      fraction += fraction_increment;
      if (fraction >= 0){
        x += xstep;
        fraction -= twody;
      }
      updateLogOddsArrayAndMapAsFree(x, y, end_point_index_m, update);
    }
  }
}

void GraphOptimiser::updateLogOddsArrayAndMapAsFree(int x, int y,
                              std::vector<int> end_point_index_m, bool update){
  if (x != end_point_index_m[0] || y != end_point_index_m[1]){
    log_odds_array_(x, y) += l_free_ - l_0_;
    if (update){
      unsigned int temp_id = mapIndexToId(x, y, map_width_);
      occupancy_grid_msg_.data.at(temp_id) = 100 * (1.0 - 1.0 / (1.0 +
                                             exp(log_odds_array_(x, y))));
    }
  }
}