/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include "particle_potential.h"
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>

#include "global_planner/astar.h"
#include "global_planner/grid_path.h"
#include "global_planner/dijkstra.h"
static int ct = 0;
//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(Pso_pot_planner::PsoPotPlannerROS, nav_core::BaseGlobalPlanner)

int clock_gettime(clockid_t clk_id, struct timespect *tp);

timespec diff(timespec start, timespec end)
{
    timespec temp;
    if ((end.tv_nsec - start.tv_nsec) < 0)
    {
        temp.tv_sec = end.tv_sec - start.tv_sec - 1;
        temp.tv_nsec = 1000000000 + end.tv_nsec - start.tv_nsec;
    }
    else
    {
        temp.tv_sec = end.tv_sec - start.tv_sec;
        temp.tv_nsec = end.tv_nsec - start.tv_nsec;
    }
    return temp;
}

namespace Pso_pot_planner
{

void PsoPotPlannerROS::outlineMap(unsigned char *costarr, int nx, int ny, unsigned char value)
{
    unsigned char *pc = costarr;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr + (ny - 1) * nx;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
    pc = costarr + nx - 1;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
}

PsoPotPlannerROS::PsoPotPlannerROS() : costmap_(NULL), initialized_(false), allow_unknown_(true)
{
}

PsoPotPlannerROS::PsoPotPlannerROS(std::string name, costmap_2d::Costmap2D *costmap, std::string frame_id) : costmap_(NULL), initialized_(false), allow_unknown_(true)
{
    //initialize the planner
    initialize(name, costmap, frame_id);
}

PsoPotPlannerROS::~PsoPotPlannerROS()
{
    if (p_calc_)
        delete p_calc_;
    if (planner_)
        delete planner_;
    if (path_maker_)
        delete path_maker_;
    if (dsrv_)
        delete dsrv_;
}

void PsoPotPlannerROS::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{
    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}

void PsoPotPlannerROS::initialize(std::string name, costmap_2d::Costmap2D *costmap, std::string frame_id)
{
    if (!initialized_)
    {
        ros::NodeHandle private_nh("~/" + name);
        costmap_ = costmap;
        frame_id_ = frame_id;

        unsigned int cx = costmap->getSizeInCellsX(), cy = costmap->getSizeInCellsY();
        originX = costmap_->getOriginX();
        originY = costmap_->getOriginY();

        width = costmap_->getSizeInCellsX();
        height = costmap_->getSizeInCellsY();
        resolution = costmap_->getResolution();

        convert_offset_ = 0.5;
        p_calc_ = new PotentialCalculator(cx, cy);

        planner_ = new AStarExpansion(p_calc_, cx, cy);
        // DijkstraExpansion *de = new DijkstraExpansion(p_calc_, cx, cy);
        // planner_ = de;

        path_maker_ = new GridPath(p_calc_);

        orientation_filter_ = new OrientationFilter();

        plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
        potential_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("potential", 1);
        //New added markers
        marker_pub1 = private_nh.advertise<visualization_msgs::Marker>("visualization_marker1", 10);
        marker_pub2 = private_nh.advertise<visualization_msgs::Marker>("visualization_marker2", 10);

        private_nh.param("allow_unknown", allow_unknown_, true);
        planner_->setHasUnknown(allow_unknown_);
        private_nh.param("planner_window_x", planner_window_x_, 0.0);
        private_nh.param("planner_window_y", planner_window_y_, 0.0);
        private_nh.param("default_tolerance", default_tolerance_, 0.0);
        private_nh.param("publish_scale", publish_scale_, 100);

        make_plan_srv_ = private_nh.advertiseService("make_plan", &PsoPotPlannerROS::makePlanService, this);

        dsrv_ = new dynamic_reconfigure::Server<Pso_pot_planner::PsoPotPlannerROSConfig>(ros::NodeHandle("~/" + name));
        dynamic_reconfigure::Server<Pso_pot_planner::PsoPotPlannerROSConfig>::CallbackType cb = boost::bind(
            &PsoPotPlannerROS::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);

        ROS_INFO("PSO Pot Planner initialized successfully");
        initialized_ = true;
    }
    else
        ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
}

void PsoPotPlannerROS::reconfigureCB(Pso_pot_planner::PsoPotPlannerROSConfig &config, uint32_t level)
{
    planner_->setLethalCost(config.lethal_cost);
    path_maker_->setLethalCost(config.lethal_cost);
    planner_->setNeutralCost(config.neutral_cost);
    planner_->setFactor(config.cost_factor);
    publish_potential_ = config.publish_potential;
    orientation_filter_->setMode(config.orientation_mode);
    orientation_filter_->setWindowSize(config.orientation_window_size);
}

void PsoPotPlannerROS::clearRobotCell(const geometry_msgs::PoseStamped &global_pose, unsigned int mx, unsigned int my)
{
    if (!initialized_)
    {
        ROS_ERROR(
            "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //set the associated costs in the cost map to be free
    costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
}

bool PsoPotPlannerROS::makePlanService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp)
{
    makePlan(req.start, req.goal, resp.plan.poses);

    resp.plan.header.stamp = ros::Time::now();
    resp.plan.header.frame_id = frame_id_;

    return true;
}

void PsoPotPlannerROS::mapToWorld(double mx, double my, double &wx, double &wy)
{
    wx = costmap_->getOriginX() + (mx + convert_offset_) * costmap_->getResolution();
    wy = costmap_->getOriginY() + (my + convert_offset_) * costmap_->getResolution();
}

bool PsoPotPlannerROS::worldToMap(double wx, double wy, double &mx, double &my)
{
    double origin_x = costmap_->getOriginX(), origin_y = costmap_->getOriginY();
    double resolution = costmap_->getResolution();

    if (wx < origin_x || wy < origin_y)
        return false;

    mx = (wx - origin_x) / resolution - convert_offset_;
    my = (wy - origin_y) / resolution - convert_offset_;

    // cout << "convert_offset:: " << convert_offset_ << endl;
    if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY())
        return true;

    return false;
}

bool PsoPotPlannerROS::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                                std::vector<geometry_msgs::PoseStamped> &plan)
{
    return makePlan(start, goal, default_tolerance_, plan);
}

// Big Plan::
// Make-Plan called repeatedly.. Create seperate function to run PSO(Multi particle, single robot), get all global bests
// feed each gbest as goals in make-plan.
// or
// Use multi-bot setup, each running pso_planner move_base, and another PSO algo running above, giving each bot
// its personal gbest as goals at each iteration using move_base.

bool PsoPotPlannerROS::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                                double tolerance, std::vector<geometry_msgs::PoseStamped> &plan)
{
    cout << "Make Plan: " << ct++ << endl;
    boost::mutex::scoped_lock lock(mutex_);
    if (!initialized_)
    {
        ROS_ERROR(
            "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    //clear the plan, just in case
    plan.clear();

    ros::NodeHandle n;
    std::string global_frame = frame_id_;

    //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if (goal.header.frame_id != global_frame)
    {
        ROS_ERROR(
            "The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", global_frame.c_str(), goal.header.frame_id.c_str());
        return false;
    }

    if (start.header.frame_id != global_frame)
    {
        ROS_ERROR(
            "The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", global_frame.c_str(), start.header.frame_id.c_str());
        return false;
    }

    unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;
    double start_x, start_y, goal_x, goal_y;

    double wx = start.pose.position.x;
    double wy = start.pose.position.y;

    if (!costmap_->worldToMap(wx, wy, start_x_i, start_y_i))
    {
        ROS_WARN(
            "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
        return false;
    }

    worldToMap(wx, wy, start_x, start_y);
    cout << "Startx, Starty ::" << start_x << ", " << start_y << endl;

    wx = goal.pose.position.x;
    wy = goal.pose.position.y;

    if (!costmap_->worldToMap(wx, wy, goal_x_i, goal_y_i))
    {
        ROS_WARN_THROTTLE(1.0,
                          "The goal sent to the global planner is off the global costmap. Planning will always fail to this goal.");
        return false;
    }

    worldToMap(wx, wy, goal_x, goal_y);
    cout << "goalx, goaly ::" << goal_x << ", " << goal_y << endl;

    //clear the starting cell within the costmap because we know it can't be an obstacle
    clearRobotCell(start, start_x_i, start_y_i);

    int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();

    //make sure to resize the underlying array that Navfn uses
    p_calc_->setSize(nx, ny);

    planner_->setSize(nx, ny);

    path_maker_->setSize(nx, ny);

    potential_array_ = new float[nx * ny];

    outlineMap(costmap_->getCharMap(), nx, ny, costmap_2d::LETHAL_OBSTACLE);

    //=======================================================================================================================//
    // PSO Implementation (Sort of....)

    // COSTMAP OBSTACLE CHECK::

    vector<int> bestPath;
    vector<vector<double>> particle_pos_x;
    vector<vector<double>> particle_pos_y;
    bestPath.clear();
    // bestPath = PSOPlanner(costmap_->getCharMap(), start_x, start_y, goal_x, goal_y, (nx * ny * 2), particle_pos_x, particle_pos_y);
    int ct = 0;
    do
    {
        bestPath.clear();
        bestPath = PSOPlanner(costmap_->getCharMap(), start_x, start_y, goal_x, goal_y, (nx * ny * 2), particle_pos_x, particle_pos_y);
        cout << "Failed.. Replanning.." << endl;
        cout << "count:" << ct << endl;
        ct++;
    } while (bestPath.back() == 1 && ct <= 50);

    cout << "BestPath from PSO::" << endl;
    for (int i = 0; i < bestPath.size() - 1; i += 2)
    {
        cout << "(" << bestPath[i] << "," << bestPath[i + 1] << ") ";
    }
    // publish the points for visualization purposes
    publishMarkers1(bestPath);
    publishMarkers2(particle_pos_x, particle_pos_y);

    vector<float> gbest_distance;
    double goal_new_x = bestPath[0];
    double goal_new_y = bestPath[1];

    for (int i = 0; i < bestPath.size(); i += 2)
    {
        float gbest_d = sqrt(pow((int)bestPath[i] - (int)goal_x, 2.0) + pow((int)bestPath[i + 1] - (int)goal_y, 2.0));
        gbest_distance.insert(gbest_distance.begin() + gbest_distance.size(), gbest_d);
    }
    for (int i = 1; i < gbest_distance.size(); i++)
    {
        float start_d = sqrt(pow((int)start_x - (int)goal_x, 2.0) + pow((int)start_y - (int)goal_y, 2.0));
        if (start_d > gbest_distance[i])
        {
            goal_new_x = bestPath[i * 2];
            goal_new_y = bestPath[i * 2 + 1];
            break;
        }
    }

    // bool found_legal = planner_->calculatePotentials(costmap_->getCharMap(), start_x, start_y, goal_new_x, goal_new_y,
    //                                                  (nx * ny * 2), potential_array_);

    // planner_->clearEndpoint(costmap_->getCharMap(), potential_array_, goal_new_x, goal_new_y, 2);
    // if (publish_potential_)
    //     publishPotential(potential_array_);

    // if (found_legal)
    // {
    //     //extract the plan
    //     if (getPlanFromPotential(potential_array_, nx, ny, start_x, start_y, goal_new_x, goal_new_y, goal, plan))
    //     {
    //         geometry_msgs::PoseStamped goal_copy;
    //         goal_copy.pose.position.x = goal_new_x;
    //         goal_copy.pose.position.y = goal_new_y;
    //         goal_copy.header.stamp = ros::Time::now();
    //         plan.push_back(goal_copy);
    //         ROS_INFO("Got a path successfully");
    //     }
    //     else
    //     {
    //         ROS_ERROR("Failed to get a plan from potential when a legal potential was found. This shouldn't happen.");
    //     }
    // }
    // else
    // {
    //     ROS_ERROR("Failed to get a plan.");
    // }

    //=============================================================================================================================//

    bool found_legal = planner_->calculatePotentials(costmap_->getCharMap(), start_x, start_y, goal_x, goal_y,
                                                     (nx * ny * 2), potential_array_);

    planner_->clearEndpoint(costmap_->getCharMap(), potential_array_, goal_x, goal_y, 2);
    if (publish_potential_)
        publishPotential(potential_array_);

    if (found_legal)
    {
        // cout << "Inside found legal, without any found_legal " << endl;
        //extract the plan
        if (getPlanFromPotential(potential_array_, nx, ny, start_x, start_y, goal_x, goal_y, goal, plan))
        {
            //make sure the goal we push on has the same timestamp as the rest of the plan
            // cout << "Inside getPlanFromPotential if - else " << endl;
            geometry_msgs::PoseStamped goal_copy = goal;
            goal_copy.header.stamp = ros::Time::now();
            plan.push_back(goal_copy);
        }
        else
        {
            ROS_ERROR("Failed to get a plan from potential when a legal potential was found. This shouldn't happen.");
        }
    }
    else
    {
        ROS_ERROR("Failed to get a plan.");
    }

    // add orientations if needed
    orientation_filter_->processPath(start, plan);

    //publish the plan for visualization purposes
    publishPlan(plan);
    delete potential_array_;
    return !plan.empty();
}

void PsoPotPlannerROS::publishPlan(const std::vector<geometry_msgs::PoseStamped> &path)
{
    if (!initialized_)
    {
        ROS_ERROR(
            "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    gui_path.header.frame_id = frame_id_;
    gui_path.header.stamp = ros::Time::now();

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++)
    {
        gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
}

bool PsoPotPlannerROS::getPlanFromPotential(float *potential_array, int nx, int ny, double start_x, double start_y, double goal_x, double goal_y,
                                            const geometry_msgs::PoseStamped &goal,
                                            std::vector<geometry_msgs::PoseStamped> &plan)
{
    // cout << "Inside getPlan from potential function" << endl;
    if (!initialized_)
    {
        ROS_ERROR(
            "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    std::string global_frame = frame_id_;

    //clear the plan, just in case
    plan.clear();
    std::vector<std::pair<float, float>> path;

    if (!path_maker_->getPath(potential_array, start_x, start_y, goal_x, goal_y, path))
    {
        ROS_ERROR("NO PATH!");
        return false;
    }

    ros::Time plan_time = ros::Time::now();
    for (int i = path.size() - 1; i >= 0; i--)
    {
        std::pair<float, float> point = path[i];
        //convert the plan to world coordinates
        double world_x, world_y;
        mapToWorld(point.first, point.second, world_x, world_y);

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = plan_time;
        pose.header.frame_id = global_frame;
        pose.pose.position.x = world_x;
        pose.pose.position.y = world_y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        plan.push_back(pose);
    }
    return !plan.empty();
}

void PsoPotPlannerROS::publishPotential(float *potential)
{
    int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();
    double resolution = costmap_->getResolution();
    nav_msgs::OccupancyGrid grid;
    // Publish Whole Grid
    grid.header.frame_id = frame_id_;
    grid.header.stamp = ros::Time::now();
    grid.info.resolution = resolution;

    grid.info.width = nx;
    grid.info.height = ny;

    double wx, wy;
    costmap_->mapToWorld(0, 0, wx, wy);
    grid.info.origin.position.x = wx - resolution / 2;
    grid.info.origin.position.y = wy - resolution / 2;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    grid.data.resize(nx * ny);

    float max = 0.0;
    for (unsigned int i = 0; i < grid.data.size(); i++)
    {
        float potential = potential_array_[i];
        if (potential < POT_HIGH)
        {
            if (potential > max)
            {
                max = potential;
            }
        }
    }

    for (unsigned int i = 0; i < grid.data.size(); i++)
    {
        if (potential_array_[i] >= POT_HIGH)
        {
            grid.data[i] = -1;
        }
        else
            grid.data[i] = potential_array_[i] * publish_scale_ / max;
    }
    potential_pub_.publish(grid);
}

//New PSO planner function
vector<int> PsoPotPlannerROS::PSOPlanner(unsigned char *costs, double start_x, double start_y, double end_x, double end_y, int cycles,
                                         vector<vector<double>> &particle_pos_x, vector<vector<double>> &particle_pos_y)
{
    vector<int> bestPath;
    timespec time1, time2;
    /* take current time here */
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);

    PSO_depend *pso = new PSO_depend(originX, originY, resolution, width, height, costmap_->getCharMap());
    bestPath = pso->PsoPath(start_x, start_y, end_x, end_y, particle_pos_x, particle_pos_y);

    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);
    cout << "Time to generate best global path by PSO = " << (diff(time1, time2).tv_sec) * 1e3 + (diff(time1, time2).tv_nsec) * 1e-6 << " microseconds" << endl;

    return bestPath;
}

void PsoPotPlannerROS::publishMarkers1(vector<int> &bestPath)
{
    if (!initialized_)
    {
        ROS_ERROR(
            "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = frame_id_;
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "points";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = 0;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    // POINTS markers use x and y scale for width/height respectively
    line_strip.scale.x = 0.1;
    line_strip.scale.y = 0.1;

    // Points are blue
    line_strip.color.r = 0.0f;
    line_strip.color.g = 0.0f;
    line_strip.color.b = 1.0f;
    line_strip.color.a = 1.0;

    // Create the vertices for the points
    for (int i = 0; i < bestPath.size() - 1; i += 2)
    {
        geometry_msgs::Point p;
        double world_x, world_y;
        mapToWorld(bestPath[i], bestPath[i + 1], world_x, world_y);
        p.x = world_x;
        p.y = world_y;
        p.z = 0.0;
        line_strip.points.push_back(p);
    }

    marker_pub1.publish(line_strip);
}

void PsoPotPlannerROS::publishMarkers2(vector<vector<double>> &particle_pos_x, vector<vector<double>> &particle_pos_y)
{
    if (!initialized_)
    {
        ROS_ERROR(
            "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    visualization_msgs::Marker points;
    points.header.frame_id = frame_id_;
    points.header.stamp = ros::Time::now();
    points.ns = "points";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;

    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;
    // points.scale.z = 0.1;

    // Points are blue
    points.color.r = 0.0f;
    points.color.g = 1.0f;
    points.color.b = 0.0f;
    points.color.a = 1.0;

    // Create the vertices for the points
    for (int i = 0; i < particle_pos_x.size(); i++)
    {
        for (int j = 0; j < particle_pos_x[0].size(); j++)
        {
            geometry_msgs::Point p;
            double world_x, world_y;
            mapToWorld(particle_pos_x[i][j], particle_pos_y[i][j], world_x, world_y);
            p.x = world_x;
            p.y = world_y;
            p.z = 0.0;
            points.points.push_back(p);
        }
    }

    marker_pub2.publish(points);
}

} // namespace Pso_pot_planner
