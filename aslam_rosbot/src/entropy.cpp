#include <iostream>
#include <vector>
#include <map_extract.h>
#include <visualization_msgs/Marker.h>
#include <numeric>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "aslam_rosbot/PointArray.h"
/*

iRotate Active Visual SLAM for Omnidirectional Robots


*/

void entropycallback(const aslam_rosbot::PointArray::ConstPtr& msg)
{    
   auto no_of_frontiers = msg->points.size();
   auto forntier_array = msg;

   //forntierarray->points[0].x;
   //for ( auto i=0;i<req_size;i++)
   //{    
   //   ROS_INFO("Frontier Points: x=%.2f, y=%.2f", msg->points[i].x, msg->points[i].y);   
   // }   
   
}


/**
 *  function which creates map index from position information.
 */
std::vector<int> positionToMapIndex(double x, double y,
                                    double offset_x, double offset_y, float map_resolution) {
    std::vector<int> index(2);
    index[0] = floor((x - offset_x) / map_resolution);
    index[1] = floor((y - offset_y) / map_resolution);
    return index;
}


/**
 *  function which creates map ids from index.
 */
int mapIndexToId(int x_index, int y_index, int width) {
    int id = x_index + y_index * width;
    return id;
}

/**
* function which provides id to position.
* id = x+y*width
*/
std::vector<double> idToPosition(unsigned int id,
                                 int map_width, int map_height, float map_resolution, double x_off, double y_off) {
    unsigned int iy_cell = id / map_width;
    unsigned int ix_cell = id - iy_cell * map_width;
    std::vector<double> position_cell(2);
    position_cell[0] = int(ix_cell) * map_resolution + x_off + map_resolution / 2.0;
    position_cell[1] = int(iy_cell) * map_resolution + y_off + map_resolution / 2.0;

    return position_cell;
}

/**
* function which calculates the angle from x and y.
*/
double xyDiffToYaw(double x_diff, double y_diff) {
;
}

/**
* startAngle: start orientation
* angle: angle that will be checked
* fov_angle_rad: angle between this range
*/
bool is_angle_between(double startAngle, double angle, double fov_angle_rad) {
    double anglediff = fmod((startAngle - angle + M_PI + 2 * M_PI), (2 * M_PI)) - M_PI;
    if (anglediff <= (fov_angle_rad / 2) && anglediff >= -(fov_angle_rad / 2)) {
        return true;
    } else {
        return false;
    }
}

bool is_angle_between_cell(double angle_center, double startAngle, double occ_min_angle_rad, double occ_max_angle_rad) {
    double fov_cell = abs(fmod((occ_max_angle_rad - occ_min_angle_rad + M_PI + 2 * M_PI), (2 * M_PI)) - M_PI);
    fov_cell -= 0.01 * M_PI / 180;
    return (is_angle_between(startAngle, angle_center, fov_cell));
}

// Function to calculate distance
double distance(double x_origin_fov, double y_origin_fov, double x_point, double y_point) {
    // Calculating distance
    return sqrt(pow(x_point - x_origin_fov, 2) +
                pow(y_point - y_origin_fov, 2) * 1.0);
}

enum COST_TYPE {
    NUMBER_OF_CELLS, //0
    BASELINE, //1
    CUSTOM_PROB, //2
    CUSTOM_PROB_WEIGHT_STATIC, //3
    CUSTOM_PROB_WEIGHT_DYNAMIC, //4
    CUSTOM_PROB_WEIGHT_DYNAMIC_AND_OBS, //5
    RANDOM, //6
    INTERPOLATION, //7
    CUSTOM_PROB_WEIGHT_STATIC_AND_OBS, // 8
    CUSTOM_PROB_AND_OBS // 9
};

double MapExtract::cost_function(const int &kind_cost, const int &prob, const double &dist, const double &length) {
   ;
}

/**
 * Check if cells are in the region of the field of view (fov).
 */
bool checkPoint(double radius, double x, double y, double x_start, double y_start, double angle_fov,
                double startAngle, double min_distance = 0) {
    // Calculate polar co-ordinates
    ;
}

double getTheta(geometry_msgs::Quaternion msg) {
   ;
}


MapExtract::MapExtract(ros::NodeHandle nh, ros::NodeHandle private_nh) :
        nh_(nh), private_nh_(private_nh) {
    ROS_INFO("Started MapExtract");
    initParams();
}

MapExtract::~MapExtract() {}

void MapExtract::initParams() {

    // Get ros param
    private_nh_.param<double>("depth", radius_camera_, 4);
    private_nh_.param<int>("prob_threshold_free", prob_threshold_free_, 30);
    private_nh_.param<int>("fov", fov_camera_deg_, 86);
    private_nh_.param<bool>("debug", debug_, false);
    private_nh_.param<double>("min_dist", min_distance_, 0.5);
    private_nh_.param<bool>("weighted_avg",weighted_avg, true);

//Publisher
   
    // cell within FOV - before raytracing
    //occ_cell_pub_ = nh_.advertise<nav_msgs::GridCells>("/occ_points_grid_cell", 1);
    //free_cell_pub_ = nh_.advertise<nav_msgs::GridCells>("/free_points_grid_cell", 1);
    //unk_cell_pub_ = nh_.advertise<nav_msgs::GridCells>("/unk_points_grid_cell", 1);

    // raytrace
    //raytracing_target_pub_ = nh_.advertise<nav_msgs::GridCells>("/raytracing_target", 1);
    //raytracing_occu_pub_ = nh_.advertise<nav_msgs::GridCells>("/raytracing_occu", 1);
    //raytracing_free_pub_ = nh_.advertise<nav_msgs::GridCells>("/raytracing_free", 1);
    //raytracing_unk_pub_ = nh_.advertise<nav_msgs::GridCells>("/raytracing_unk", 1);

    // origin of he FOV + BB
    //origin_fov_pub_ = nh_.advertise<nav_msgs::GridCells>("/cell", 1);
    //submap_bounding_box_cell_pub_ = nh_.advertise<nav_msgs::GridCells>("submap_bounding_box_grid_cell", 1);

    
    //opt_heading_pub_ = nh_.advertise<nav_msgs::GridCells>("opt_heading_cells", 1);
    //marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // Init Client -
    //get_map_prob_client_ = nh_.serviceClient<nav_msgs::GetMap>("/rtabmap/get_prob_map");

// Init Service
    //set_path_service_ = nh_.advertiseService("get_best_path", &MapExtract::getBestPathServiceCallback,
    //                                         this);
    //set_point_service_ = nh_.advertiseService("get_best_head", &MapExtract::getBestHeadServiceCallback,
    //                                          this);
    //ROS_INFO("get_best_path started!");

}

void MapExtract::getBoundingBox(const double &theta, const double &fov, const double &x_start, const double &y_start,
                                double &x_max, double &x_min, double &y_max, double &y_min) {
   ;
}

void MapExtract::cellInFov(double const &theta_start, double const &fov, double const &x_start, double const &y_start,
                           std::vector<std::pair<double, double>> &occuVector, // angle, distance
                           std::vector<std::pair<double, double>> &min_max_angle_occu_vec, // min and max angles for each occupied cell
                           std::vector<std::pair<double, double>> &cellPos, // x,y
                           std::vector<std::pair<double, double>> &min_max_angle_vec,
                           std::vector<std::pair<double, double>> &cellVector,
                           std::vector<int> &prob_vec, const std::vector<bool> &added_before) {
    ;
}

template<typename T, typename Compare>
std::vector<std::size_t> sort_permutation(
        const std::vector<T> &vec,
        Compare &compare) {
    std::vector<std::size_t> p(vec.size());
    std::iota(p.begin(), p.end(), 0);
    std::sort(p.begin(), p.end(),
              [&](std::size_t i, std::size_t j) { return compare(vec[i], vec[j]); });
    return p;
}

template<typename T>
std::vector<T> apply_permutation(
        const std::vector<T> &vec,
        const std::vector<std::size_t> &p) {
    std::vector<T> sorted_vec(vec.size());
    std::transform(p.begin(), p.end(), sorted_vec.begin(),
                   [&](std::size_t i) { return vec[i]; });
    return sorted_vec;
}

bool sortasc(const std::pair<double, double> &a,
             const std::pair<double, double> &b) {
    return (a.second < b.second);
}

bool sortdesc(const std::pair<double, double> &a,
              const std::pair<double, double> &b) {
    return (a.second > b.second);
}

void MapExtract::raytracing(std::vector<std::pair<double, double>> &occuVector, // angle, distance
                            std::vector<std::pair<double, double>> &min_max_angle_occu_vec, // min and max angles for each occupied cell
                            const std::vector<std::pair<double, double>> &cellPos, // x,y
                            const std::vector<std::pair<double, double>> &min_max_angle_vec,
                            std::vector<std::pair<double, double>> &cellVector,
                            const std::vector<int> &prob_vec,
                            std::vector<std::pair<int, int>> &visible_cells,
                            std::vector<std::pair<double, double>> &visibleCellsPos,
                            const std::vector<bool> &added_before) {

    
 ;
}

double MapExtract::getMaxUtility(std::vector<std::pair<int, int>> &visible_cells,
                                 const std::vector<std::pair<double, double>> &visibleCellsPos,
                                 const double &x_start, const double &y_start, std::vector<int> &max_utility_angles,
                                 const double &wp_dist, const double &path_length, const double &inter_angle) {
    ;
}

double MapExtract::normalizeToResolution(const double &coord) {
    double res = coord;
    if (fmod(coord, map_resolution_) != 0) {
        if (fmod(coord, map_resolution_) > map_resolution_ / 2) {
            res = coord + (map_resolution_ - fmod(coord, map_resolution_));
        } else {
            res = coord - fmod(coord, map_resolution_);
        }
    }
    return res;
}

bool MapExtract::getMap() {
;
}

bool MapExtract::getBestHeadServiceCallback(aslam_rosbot::get_best_head::Request &request,
                                            aslam_rosbot::get_best_head::Response &response) {
   ;
}

bool MapExtract::getBestPathServiceCallback(aslam_rosbot::get_best_path::Request &request,
                                            aslam_rosbot::get_best_path::Response &response) {

    ROS_INFO_STREAM("req received");
    // available fov span


    // fov that can be seen by the cam


    // type of cost function
    kind_cost_ = request.kind_cost;

    // get map
    if (!getMap()) {
        return false;
    }

    std::vector<bool> added_before;

    std::vector<int> all_costs = {COST_TYPE::NUMBER_OF_CELLS, COST_TYPE::BASELINE, COST_TYPE::CUSTOM_PROB,
                                  COST_TYPE::CUSTOM_PROB_WEIGHT_STATIC, COST_TYPE::CUSTOM_PROB_WEIGHT_DYNAMIC,
                                  COST_TYPE::CUSTOM_PROB_WEIGHT_DYNAMIC_AND_OBS, COST_TYPE::RANDOM,
                                  COST_TYPE::INTERPOLATION, COST_TYPE::CUSTOM_PROB_WEIGHT_STATIC_AND_OBS,
                                  COST_TYPE::CUSTOM_PROB_AND_OBS};
    std::vector<double> max_utility, max_entropy;
    max_utility.resize(all_costs.size(), 0);
     for (int i = 0; i < max_utility.size(); i++) {
        
         ROS_INFO("max_utility.resize %d = ",max_utility[i]);
     }

    max_entropy.resize(all_costs.size(), 0);
    std::vector<int> final_location_index;
    final_location_index.resize(all_costs.size(), -1);

    std::vector<geometry_msgs::Pose> final_location;
    final_location.resize(all_costs.size());

    int wanted_kind = kind_cost_;
    int cnt = 0;
    int index = 0;

    // create a minimum bounding box

    for (auto l:request.paths.data) {
        std::vector<double> entropy, tot_utility;
        entropy.resize(all_costs.size(), 0);
        tot_utility.resize(all_costs.size(), 0);

        double dist = 0;

        ROS_INFO_STREAM("begin path");
        double total_weight = 0;

        // compute utility every meter
        for (int k = 0; k < l.waypoints.size(); k += 2) {
            if (!request.only_last || k + 1 == l.waypoints.size()) {
                geometry_msgs::Point point;
                double x_start = normalizeToResolution(l.waypoints.at(k).position.x);
                double y_start = normalizeToResolution(l.waypoints.at(k).position.y);
                std::vector<std::pair<int, int>> visible_cells;
                std::vector<std::pair<double, double>> visibleCellsPos;
                double cur_head;
                for (auto cost : all_costs) {
                    std::vector<int> max_utility_angles;
                    kind_cost_ = cost;
                    double min_diff = 360;
                    if (cost == wanted_kind) {
                        total_weight += std::exp(-0.25 * dist);
                       }

                    if (debug_ || cost == wanted_kind) {                        
                        cnt += 1;
                        int counter = 0;
                        for (auto i:visibleCellsPos) {                                             
                            point.x = i.first;
                            point.y = i.second;
                            point.z = 0;
                            //std::vector<int> submap_id = positionToMapIndex(i.first, i.second, x_origin, y_origin,
                                                                            //map_resolution_);
                            int id = 0;//mapIndexToId(submap_id.at(0), submap_id.at(1), map_width_);
                            double prob = int(latest_map_msg_.data.at(id));
                            prob == -1 ? prob = 50 : prob;
                            prob == 0 ? prob = 0.00000000001 : (prob == 100 ? prob = 0.999999999 : prob = prob /100.0);
                            entropy[cost] +=
                                    -(prob * std::log2(prob) + (1 - prob) * std::log2(1 - prob)) *
                                    std::exp(-0.25 * (dist));
                            counter += 1;                           
                        }
//                    entropy /= counter;
                        // Publish GridCells msg of optimal heading
                        opt_heading_pub_.publish(opt_heading_msg);
                    }
                }
            }
            
            clear_messages();
        }
        index++;
    }
    ROS_INFO_STREAM("------------------------------------------------------------------");
    for (auto kind_considered: all_costs) {
        if (debug_ || kind_considered == wanted_kind) {
            ROS_INFO_STREAM("FOR kind of cost " << kind_considered << " we have: ");
            ROS_INFO_STREAM("We get " << max_utility[kind_considered] << " utility");
            ROS_INFO_STREAM("We get " << max_entropy[kind_considered] << " entropy");
            ROS_INFO_STREAM("Toward " << final_location_index[kind_considered] << "\n");
        }
    }

    return true;
}

void MapExtract::clear_messages() {
;
}

void MapExtract::prepare_messages(double map_resolution_) {
  ;
}

void MapExtract::publish_bb(double x_max, double x_min, double y_max, double y_min, double x_origin, double y_origin,
                            double map_resolution_) {
    geometry_msgs::Point point;

   ;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "entropy");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Subscriber sub = nh.subscribe("/filtered_points", 10, entropycallback);
    MapExtract map_reader(nh, private_nh);


    ros::spin();

    return 0;
}



