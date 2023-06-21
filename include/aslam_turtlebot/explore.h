#ifndef NAV_EXPLORE_H_
#define NAV_EXPLORE_H_

#include "aslam_turtlebot/costmap_client.h"
#include "aslam_turtlebot/frontier_search.h"

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <memory>
#include <move_base_msgs/MoveBaseAction.h>
#include <mutex>
#include <ros/ros.h>
#include <string>
#include <vector>
#include <visualization_msgs/MarkerArray.h>
#include <std_srvs/Empty.h>

namespace frontier_exploration {
    class Explore
    {
    public:
        Explore(ros::NodeHandle* nh, ros::NodeHandle* pnh);
        ~Explore();

        void start();
        void stop();

    private:
        void makePlan();
        void mytimercallback();

        void visualizeFrontiers(const std::vector<Frontier>& frontiers);

        void reachedGoal(const actionlib::SimpleClientGoalState& status,
                         const move_base_msgs::MoveBaseResultConstPtr& result,
                         const geometry_msgs::Point& frontier_goal);

        bool goalOnBlacklist(const geometry_msgs::Point& goal);

        bool onExplorationStart(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
        bool onExplorationAbort(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    private:
        ros::NodeHandle* _pnh;
        ros::NodeHandle* _nh;
        ros::Publisher _markerPub;
        tf::TransformListener _tf;
        std::atomic<bool> _active;

        Costmap2DClient _costmapClient;
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> _mbClient;
        frontier_exploration::FrontierSearch _search;
        ros::Timer _explorationTimer;
        ros::Timer _mytimer;
        ros::Timer _oneShotTimer;

        ros::Time _ExpstartTime;
        ros::Time _ExpendTime;
        ros::Duration _Expduration;




        ros::ServiceServer _explorationStartSrv, _explorationAbortSrv;
        std::vector<geometry_msgs::Point> _frontierBlacklist;
        geometry_msgs::Point _prevGoal;
        double _prevDistance;
        ros::Time _lastProgress;

        // parameters
        double _plannerFrequency;
        double _potentialScale, _gainScale;
        ros::Duration _progressTimeout;
        bool _visualize;
        double _explorationTime;
    };
} // namespace frontier_exploration

#endif
