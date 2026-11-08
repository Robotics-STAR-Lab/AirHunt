#include <exploration_manager/selective_sop.h>

#include <exploration_manager/fast_exploration_manager.h>
#include <thread>
#include <iostream>
#include <fstream>

#include <active_perception/graph_node.h>
#include <active_perception/graph_search.h>
#include <active_perception/perception_utils.h>
#include <plan_env/raycast.h>
#include <plan_env/sdf_map.h>
#include <plan_env/edt_environment.h>
#include <active_perception/frontier_finder.h>
#include <plan_manage/planner_manager.h>

#include <exploration_manager/expl_data.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>

using namespace Eigen;

namespace fast_planner
{

    FastExplorationManager::FastExplorationManager()
    {
    }

    FastExplorationManager::~FastExplorationManager()
    {
        ViewNode::astar_.reset();
        ViewNode::caster_.reset();
        ViewNode::map_.reset();
    }

    void FastExplorationManager::initialize(ros::NodeHandle &nh)
    {
        planner_manager_.reset(new FastPlannerManager);
        planner_manager_->initPlanModules(nh);
        edt_environment_ = planner_manager_->edt_environment_;
        sdf_map_ = edt_environment_->sdf_map_;
        frontier_finder_.reset(new FrontierFinder(edt_environment_, nh));

        ed_.reset(new ExplorationData);
        ep_.reset(new ExplorationParam);

        nh.param("exploration/refine_local", ep_->refine_local_, true);
        nh.param("exploration/refined_num", ep_->refined_num_, -1);
        nh.param("exploration/refined_radius", ep_->refined_radius_, -1.0);
        nh.param("exploration/top_view_num", ep_->top_view_num_, -1);
        nh.param("exploration/max_decay", ep_->max_decay_, -1.0);
        nh.param("exploration/relax_time", ep_->relax_time_, 1.0);
        nh.param("exploration/sop_rho", sop_rho_, 0.15);
        if (!std::isfinite(sop_rho_) || sop_rho_ < 0.0)
            throw std::invalid_argument("Invalid sop_rho");

        nh.param("exploration/vm", ViewNode::vm_, -1.0);
        nh.param("exploration/am", ViewNode::am_, -1.0);
        nh.param("exploration/yd", ViewNode::yd_, -1.0);
        nh.param("exploration/ydd", ViewNode::ydd_, -1.0);
        nh.param("exploration/w_dir", ViewNode::w_dir_, -1.0);

        ViewNode::astar_.reset(new Astar);
        ViewNode::astar_->init(nh, edt_environment_);
        ViewNode::map_ = sdf_map_;

        double resolution_ = sdf_map_->getResolution();
        Eigen::Vector3d origin, size;
        sdf_map_->getRegion(origin, size);
        ViewNode::caster_.reset(new RayCaster);
        ViewNode::caster_->setParams(resolution_, origin);

        planner_manager_->path_finder_->lambda_heu_ = 1.0;

        planner_manager_->path_finder_->max_search_time_ = 1.0;

    }

    int FastExplorationManager::planExploreMotion(
        const Vector3d &pos, const Vector3d &vel, const Vector3d &acc, const Vector3d &yaw)
    {
        ros::Time t1 = ros::Time::now();
        auto t2 = t1;
        ed_->views_.clear();
        ed_->global_tour_.clear();

        std::cout << "start pos: " << pos.transpose() << ", vel: " << vel.transpose()
                  << ", acc: " << acc.transpose() << std::endl;

        frontier_finder_->searchFrontiers();

        ros::Time t3 = ros::Time::now();

        frontier_finder_->updateFrontierValue();
        ROS_WARN("update frontier value time: %lf", (t3 - t1).toSec());

        double frontier_time = (ros::Time::now() - t1).toSec();
        t1 = ros::Time::now();

        frontier_finder_->computeFrontiersToVisit();
        frontier_finder_->getFrontiers(ed_->frontiers_);
        frontier_finder_->getFrontierBoxes(ed_->frontier_boxes_);
        frontier_finder_->getDormantFrontiers(ed_->dead_frontiers_);

        if (ed_->frontiers_.empty())
        {
            ROS_WARN("No coverable frontier.");
            return NO_FRONTIER;
        }
        frontier_finder_->getTopViewpointsInfo(pos, ed_->points_, ed_->yaws_, ed_->averages_);
        for (int i = 0; i < ed_->points_.size(); ++i)
            ed_->views_.push_back(
                ed_->points_[i] + 2.0 * Vector3d(cos(ed_->yaws_[i]), sin(ed_->yaws_[i]), 0));

        double view_time = (ros::Time::now() - t1).toSec();
        ROS_WARN(
            "Frontier: %d, t: %lf, viewpoint: %d, t: %lf", ed_->frontiers_.size(), frontier_time,
            ed_->points_.size(), view_time);

        Vector3d next_pos;
        double next_yaw;
        if (ed_->points_.size() > 1)
        {

            vector<int> indices;
            findGlobalTour(pos, vel, yaw, indices);

            if (ep_->refine_local_)
            {

                t1 = ros::Time::now();

                ed_->refined_ids_.clear();
                ed_->unrefined_points_.clear();
                int knum = min(int(indices.size()), ep_->refined_num_);
                for (int i = 0; i < knum; ++i)
                {
                    auto tmp = ed_->points_[indices[i]];
                    ed_->unrefined_points_.push_back(tmp);
                    ed_->refined_ids_.push_back(indices[i]);
                    if ((tmp - pos).norm() > ep_->refined_radius_ && ed_->refined_ids_.size() >= 2)
                        break;
                }

                ed_->n_points_.clear();
                vector<vector<double>> n_yaws;
                frontier_finder_->getViewpointsInfo(
                    pos, ed_->refined_ids_, ep_->top_view_num_, ep_->max_decay_, ed_->n_points_, n_yaws);

                ed_->refined_points_.clear();
                ed_->refined_views_.clear();
                vector<double> refined_yaws;
                refineLocalTour(pos, vel, yaw, ed_->n_points_, n_yaws, ed_->refined_points_, refined_yaws);
                next_pos = ed_->refined_points_[0];
                next_yaw = refined_yaws[0];

                for (int i = 0; i < ed_->refined_points_.size(); ++i)
                {
                    Vector3d view =
                        ed_->refined_points_[i] + 2.0 * Vector3d(cos(refined_yaws[i]), sin(refined_yaws[i]), 0);
                    ed_->refined_views_.push_back(view);
                }
                ed_->refined_views1_.clear();
                ed_->refined_views2_.clear();
                for (int i = 0; i < ed_->refined_points_.size(); ++i)
                {
                    vector<Vector3d> v1, v2;
                    frontier_finder_->percep_utils_->setPose(ed_->refined_points_[i], refined_yaws[i]);
                    frontier_finder_->percep_utils_->getFOV(v1, v2);
                    ed_->refined_views1_.insert(ed_->refined_views1_.end(), v1.begin(), v1.end());
                    ed_->refined_views2_.insert(ed_->refined_views2_.end(), v2.begin(), v2.end());
                }
                double local_time = (ros::Time::now() - t1).toSec();
                ROS_WARN("Local refine time: %lf", local_time);
            }
            else
            {

                next_pos = ed_->points_[indices[0]];
                next_yaw = ed_->yaws_[indices[0]];
            }
        }
        else if (ed_->points_.size() == 1)
        {

            frontier_finder_->updateFrontierCostMatrix();
            ed_->global_tour_ = {pos, ed_->points_[0]};
            ed_->refined_tour_.clear();
            ed_->refined_views1_.clear();
            ed_->refined_views2_.clear();

            if (ep_->refine_local_)
            {

                ed_->refined_ids_ = {0};
                ed_->unrefined_points_ = {ed_->points_[0]};
                ed_->n_points_.clear();
                vector<vector<double>> n_yaws;
                frontier_finder_->getViewpointsInfo(
                    pos, {0}, ep_->top_view_num_, ep_->max_decay_, ed_->n_points_, n_yaws);

                double min_cost = 100000;
                int min_cost_id = -1;
                vector<Vector3d> tmp_path;
                for (int i = 0; i < ed_->n_points_[0].size(); ++i)
                {
                    auto tmp_cost = ViewNode::computeCost(
                        pos, ed_->n_points_[0][i], yaw[0], n_yaws[0][i], vel, yaw[1], tmp_path);
                    if (tmp_cost < min_cost)
                    {
                        min_cost = tmp_cost;
                        min_cost_id = i;
                    }
                }
                next_pos = ed_->n_points_[0][min_cost_id];
                next_yaw = n_yaws[0][min_cost_id];
                ed_->refined_points_ = {next_pos};
                ed_->refined_views_ = {next_pos + 2.0 * Vector3d(cos(next_yaw), sin(next_yaw), 0)};
            }
            else
            {
                next_pos = ed_->points_[0];
                next_yaw = ed_->yaws_[0];
            }
        }
        else
            ROS_ERROR("Empty destination.");

        std::cout << "Next view: " << next_pos.transpose() << ", " << next_yaw << std::endl;

        t1 = ros::Time::now();

        double diff = fabs(next_yaw - yaw[0]);
        double time_lb = min(diff, 2 * M_PI - diff) / ViewNode::yd_;

        planner_manager_->path_finder_->reset();
        if (planner_manager_->path_finder_->search(pos, next_pos) != Astar::REACH_END)
        {
            ROS_ERROR("No path to next viewpoint");
            return FAIL;
        }
        ed_->path_next_goal_ = planner_manager_->path_finder_->getPath();
        shortenPath(ed_->path_next_goal_);

        const double radius_far = 5.0;
        const double radius_close = 1.5;
        const double len = Astar::pathLength(ed_->path_next_goal_);
        if (len < radius_close)
        {

            planner_manager_->planExploreTraj(ed_->path_next_goal_, vel, acc, time_lb);
            ed_->next_goal_ = next_pos;
        }
        else if (len > radius_far)
        {

            std::cout << "Far goal." << std::endl;
            double len2 = 0.0;
            vector<Eigen::Vector3d> truncated_path = {ed_->path_next_goal_.front()};
            for (int i = 1; i < ed_->path_next_goal_.size() && len2 < radius_far; ++i)
            {
                auto cur_pt = ed_->path_next_goal_[i];
                len2 += (cur_pt - truncated_path.back()).norm();
                truncated_path.push_back(cur_pt);
            }
            ed_->next_goal_ = truncated_path.back();
            planner_manager_->planExploreTraj(truncated_path, vel, acc, time_lb);

        }
        else
        {

            std::cout << "Mid goal" << std::endl;
            ed_->next_goal_ = next_pos;

            if (!planner_manager_->kinodynamicReplan(
                    pos, vel, acc, ed_->next_goal_, Vector3d(0, 0, 0), time_lb))
                return FAIL;
        }

        if (planner_manager_->local_data_.position_traj_.getTimeSum() < time_lb - 0.1)
            ROS_ERROR("Lower bound not satified!");

        planner_manager_->planYawExplore(yaw, next_yaw, true, ep_->relax_time_);

        double traj_plan_time = (ros::Time::now() - t1).toSec();
        t1 = ros::Time::now();

        double yaw_time = (ros::Time::now() - t1).toSec();
        ROS_WARN("Traj: %lf, yaw: %lf", traj_plan_time, yaw_time);
        double total = (ros::Time::now() - t2).toSec();
        ROS_WARN("Total time: %lf", total);
        ROS_ERROR_COND(total > 0.1, "Total time too long!!!");

        return SUCCEED;
    }

    void FastExplorationManager::shortenPath(vector<Vector3d> &path)
    {
        if (path.empty())
        {
            ROS_ERROR("Empty path to shorten");
            return;
        }

        const double dist_thresh = 3.0;
        vector<Vector3d> short_tour = {path.front()};
        for (int i = 1; i < path.size() - 1; ++i)
        {
            if ((path[i] - short_tour.back()).norm() > dist_thresh)
                short_tour.push_back(path[i]);
            else
            {

                ViewNode::caster_->input(short_tour.back(), path[i + 1]);
                Eigen::Vector3i idx;
                while (ViewNode::caster_->nextId(idx) && ros::ok())
                {
                    if (edt_environment_->sdf_map_->getInflateOccupancy(idx) == 1 ||
                        edt_environment_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN)
                    {
                        short_tour.push_back(path[i]);
                        break;
                    }
                }
            }
        }
        if ((path.back() - short_tour.back()).norm() > 1e-3)
            short_tour.push_back(path.back());

        if (short_tour.size() == 2)
            short_tour.insert(short_tour.begin() + 1, 0.5 * (short_tour[0] + short_tour[1]));
        path = short_tour;
    }

    void FastExplorationManager::findGlobalTour(
        const Vector3d& pos, const Vector3d& vel, const Vector3d yaw,
        vector<int>& indices)
    {
        frontier_finder_->updateFrontierCostMatrix();
        Eigen::MatrixXd geometry;
        vector<double> scores;
        frontier_finder_->getGeometricCostMatrix(pos, yaw, geometry, scores);
        const auto started = ros::WallTime::now();
        const auto result = solveSelectiveSOP(geometry, scores, sop_rho_);
        indices = result.order;
        ROS_INFO("SOP: frontiers=%zu constraints=%d cost=%.3f time_ms=%.3f fallback=%d",
                 scores.size(), result.constraints, result.cost,
                 (ros::WallTime::now()-started).toSec()*1000.0, result.fallback);
        frontier_finder_->getPathForTour(pos, indices, ed_->global_tour_);
    }

    void FastExplorationManager::refineLocalTour(
        const Vector3d &cur_pos, const Vector3d &cur_vel, const Vector3d &cur_yaw,
        const vector<vector<Vector3d>> &n_points, const vector<vector<double>> &n_yaws,
        vector<Vector3d> &refined_pts, vector<double> &refined_yaws)
    {
        double create_time, search_time, parse_time;
        auto t1 = ros::Time::now();

        GraphSearch<ViewNode> g_search;
        vector<ViewNode::Ptr> last_group, cur_group;

        ViewNode::Ptr first(new ViewNode(cur_pos, cur_yaw[0]));
        first->vel_ = cur_vel;
        g_search.addNode(first);
        last_group.push_back(first);
        ViewNode::Ptr final_node;

        std::cout << "Local tour graph: ";
        for (int i = 0; i < n_points.size(); ++i)
        {

            for (int j = 0; j < n_points[i].size(); ++j)
            {
                ViewNode::Ptr node(new ViewNode(n_points[i][j], n_yaws[i][j]));
                g_search.addNode(node);

                for (auto nd : last_group)
                    g_search.addEdge(nd->id_, node->id_);
                cur_group.push_back(node);

                if (i == n_points.size() - 1)
                {
                    final_node = node;
                    break;
                }
            }

            std::cout << cur_group.size() << ", ";
            last_group = cur_group;
            cur_group.clear();
        }
        std::cout << "" << std::endl;
        create_time = (ros::Time::now() - t1).toSec();
        t1 = ros::Time::now();

        vector<ViewNode::Ptr> path;
        g_search.DijkstraSearch(first->id_, final_node->id_, path);

        search_time = (ros::Time::now() - t1).toSec();
        t1 = ros::Time::now();

        for (int i = 1; i < path.size(); ++i)
        {
            refined_pts.push_back(path[i]->pos_);
            refined_yaws.push_back(path[i]->yaw_);
        }

        ed_->refined_tour_.clear();
        ed_->refined_tour_.push_back(cur_pos);
        ViewNode::astar_->lambda_heu_ = 1.0;
        ViewNode::astar_->setResolution(0.2);
        for (auto pt : refined_pts)
        {
            vector<Vector3d> path;
            if (ViewNode::searchPath(ed_->refined_tour_.back(), pt, path))
                ed_->refined_tour_.insert(ed_->refined_tour_.end(), path.begin(), path.end());
            else
                ed_->refined_tour_.push_back(pt);
        }
        ViewNode::astar_->lambda_heu_ = 10000;

        parse_time = (ros::Time::now() - t1).toSec();

    }
    std::vector<Vector3d> FastExplorationManager::sampleProperViewpoints(const Vector3d &target_point, const Vector3d &cur_point, int num_points)
    {
        std::vector<Vector3d> observe_points;

        Vector3d box_min, box_max;
        sdf_map_->getBox(box_min, box_max);

        static double current_radius = 3;
        static double max_radius = 12.0;
        static double radius_increment = 0.2;
        static int failed_attempts = 0;

        const double dphi = M_PI / 15.0;
        const double height_range = 3.0;
        const double height_step = 0.1;

        if (observe_points.empty() && failed_attempts == 0)
        {
            current_radius = 2.0;
        }

        ROS_INFO("Sampling viewpoints around target [%.2f, %.2f, %.2f] with radius %.2f",
                 target_point[0], target_point[1], target_point[2], current_radius);

        for (double height = -height_range; height <= height_range + 1e-3; height += height_step)
        {
            for (double phi = -M_PI; phi < M_PI; phi += dphi)
            {
                Vector3d sample_pos = target_point + Vector3d(
                                                         current_radius * cos(phi),
                                                         current_radius * sin(phi),
                                                         height);

                if (!sdf_map_->isInBox(sample_pos) ||
                    sdf_map_->getInflateOccupancy(sample_pos) == 1)
                {
                    continue;
                }

                if (sample_pos[2] < 0)
                {
                    continue;
                }

                bool near_unknown = false;
                const double clearance_check = 0.3;
                int vox_num = floor(clearance_check / sdf_map_->getResolution());
                for (int x = -vox_num; x <= vox_num && !near_unknown; ++x)
                {
                    for (int y = -vox_num; y <= vox_num && !near_unknown; ++y)
                    {
                        for (int z = -1; z <= 1 && !near_unknown; ++z)
                        {
                            Vector3d check_pos = sample_pos + Vector3d(
                                                                  x * sdf_map_->getResolution(),
                                                                  y * sdf_map_->getResolution(),
                                                                  z * sdf_map_->getResolution());
                            if (sdf_map_->getOccupancy(check_pos) == SDFMap::UNKNOWN)
                            {
                                near_unknown = true;
                            }
                        }
                    }
                }

                if (near_unknown)
                    continue;

                observe_points.push_back(sample_pos);

                if (observe_points.size() >= num_points)
                {
                    failed_attempts = 0;
                    ROS_INFO("Successfully found %d observation points at radius %.2f",
                             (int)observe_points.size(), current_radius);
                    return observe_points;
                }
            }
        }

        if (observe_points.size() < num_points)
        {
            failed_attempts++;
            if (current_radius < max_radius)
            {
                current_radius += radius_increment;
                ROS_WARN("Not enough observation points found, expanding radius to %.2f for next attempt", current_radius);
            }
            else
            {
                ROS_WARN("Reached maximum search radius %.2f, returning %d points", max_radius, (int)observe_points.size());
            }
        }

        if (observe_points.empty())
        {
            Vector3d fallback = target_point + Vector3d(current_radius, 0, 0);
            fallback = getNearestTruePoint(fallback);
            observe_points.push_back(fallback);
            ROS_WARN("Using fallback observation point [%.2f, %.2f, %.2f]",
                     fallback[0], fallback[1], fallback[2]);
            ROS_WARN("Using fallback observation point [%.2f, %.2f, %.2f]",
                     fallback[0], fallback[1], fallback[2]);
            ROS_WARN("Using fallback observation point [%.2f, %.2f, %.2f]",
                     fallback[0], fallback[1], fallback[2]);
            ROS_WARN("Using fallback observation point [%.2f, %.2f, %.2f]",
                     fallback[0], fallback[1], fallback[2]);
        }

        return observe_points;
    }

    int FastExplorationManager::planTask2Path(const Vector3d &pos, const Vector3d &vel, const Vector3d &acc, const Vector3d &yaw, const Vector3d &task2_pose, Vector3d &selected_observe_point)
    {
        ros::Time t1 = ros::Time::now();

        Vector3d box_min, box_max;
        sdf_map_->getBox(box_min, box_max);

        if (task2_pose[2] < box_min[2])
        {
            const_cast<Vector3d &>(task2_pose)[2] = box_min[2];
            ROS_WARN("Task2 pose z-coordinate below box min %.2f, clamped to box min", box_min[2]);
        }
        else if (task2_pose[2] > box_max[2])
        {
            const_cast<Vector3d &>(task2_pose)[2] = box_max[2];
            ROS_WARN("Task2 pose z-coordinate above box max %.2f, clamped to box max", box_max[2]);
        }

        std::cout << "Task2 path planning - start pos: " << pos.transpose()
                  << ", target pos: " << task2_pose.transpose() << std::endl;

        ed_->views_.clear();

        std::cout << "Start sample proper viewpoints " << std::endl;
        std::vector<Vector3d> observe_points = sampleProperViewpoints(task2_pose, pos, 5);
        std::cout << "End sample proper viewpoints " << std::endl;

        Vector3d next_pos;
        double target_yaw = 0.0;
        bool path_found = false;

        for (const Vector3d &observe_point : observe_points)
        {

            Vector3d direction_to_target = task2_pose - observe_point;
            target_yaw = atan2(direction_to_target.y(), direction_to_target.x());

            ROS_INFO("Task2: Trying observe point [%.2f, %.2f, %.2f], Target yaw: %.2f rad",
                     observe_point[0], observe_point[1], observe_point[2], target_yaw);

            planner_manager_->path_finder_->reset();

            if (planner_manager_->path_finder_->search(pos, observe_point) == Astar::REACH_END)
            {
                next_pos = observe_point;
                selected_observe_point = observe_point;
                path_found = true;
                ROS_INFO("Task2: Successfully found path to observe point [%.2f, %.2f, %.2f]",
                         observe_point[0], observe_point[1], observe_point[2]);
                break;
            }

            ROS_WARN("Task2: Failed to find path to observe point [%.2f, %.2f, %.2f], trying next candidate",
                     observe_point[0], observe_point[1], observe_point[2]);
        }

        if (!path_found)
        {
            ROS_ERROR("No path to any task2 observe point");
            return FAIL;
        }

        t1 = ros::Time::now();

        vector<Vector3d> path_to_goal = planner_manager_->path_finder_->getPath();
        shortenPath(path_to_goal);

        const double len = Astar::pathLength(path_to_goal);
        double pos_time_lb = len / ViewNode::vm_;
        double yaw_time_lb = 0.0;
        double time_lb = pos_time_lb;

        std::cout << "Task2 path length: " << len << ", time_lb: " << time_lb << std::endl;

        const double radius_far = 5.0;
        Vector3d path_last_point;
        if (len < radius_far)
        {

            planner_manager_->planExploreTraj(path_to_goal, vel, acc, time_lb);
            ed_->next_goal_ = next_pos;
            path_last_point = path_to_goal.back();
        }
        else
        {

            std::cout << "Task2 target is far, using intermediate goal." << std::endl;
            double len2 = 0.0;
            vector<Eigen::Vector3d> truncated_path = {path_to_goal.front()};
            for (int i = 1; i < path_to_goal.size() && len2 < radius_far; ++i)
            {
                auto cur_pt = path_to_goal[i];
                len2 += (cur_pt - truncated_path.back()).norm();
                truncated_path.push_back(cur_pt);
            }
            ed_->next_goal_ = truncated_path.back();
            planner_manager_->planExploreTraj(truncated_path, vel, acc, time_lb);
            path_last_point = truncated_path.back();
        }

        if (planner_manager_->local_data_.position_traj_.getTimeSum() < time_lb - 0.1)
        {
            ROS_ERROR("Task2 planning: Lower bound not satisfied!");
            ROS_ERROR("Pos_tlb:%f, Traj_time:%f", pos_time_lb,
                      planner_manager_->local_data_.position_traj_.getTimeSum());
        }

        Vector3d direction_to_target = task2_pose - path_last_point;
        target_yaw = atan2(direction_to_target.y(), direction_to_target.x());

        planner_manager_->planYawExplore(yaw, target_yaw, false, ep_->relax_time_);

        double total_time = (ros::Time::now() - t1).toSec();
        ROS_INFO("Task2 path planning completed in: %lf seconds", total_time);

        return SUCCEED;
    }

    Vector3d FastExplorationManager::getNearestTruePoint(const Vector3d &point)
    {

        Eigen::Vector3i grid_point;
        sdf_map_->posToIndex(point, grid_point);

        int x = grid_point[0], y = grid_point[1], z = grid_point[2];

        Eigen::Vector3d origin, size;
        sdf_map_->getRegion(origin, size);
        double resolution = sdf_map_->getResolution();
        int max_x = (int)(size[0] / resolution);
        int max_y = (int)(size[1] / resolution);
        int max_z = (int)(size[2] / resolution);

        if (sdf_map_->getInflateOccupancy(grid_point) == 0)
        {
            return point;
        }

        int max_search = std::max({max_x, max_y, max_z});

        for (int radius = 1; radius < max_search; radius++)
        {

            for (int dx = -radius; dx <= radius; dx++)
            {
                for (int dy = -radius; dy <= radius; dy++)
                {
                    for (int dz = -radius; dz <= radius; dz++)
                    {

                        if (abs(dx) == radius || abs(dy) == radius || abs(dz) == radius)
                        {
                            Eigen::Vector3i check_point(x + dx, y + dy, z + dz);

                            if (check_point[0] >= 0 && check_point[0] < max_x &&
                                check_point[1] >= 0 && check_point[1] < max_y &&
                                check_point[2] >= 0 && check_point[2] < max_z &&
                                sdf_map_->getInflateOccupancy(check_point) == 0)
                            {
                                Vector3d world_point;
                                sdf_map_->indexToPos(check_point, world_point);
                                return world_point;
                            }
                        }
                    }
                }
            }
        }

        ROS_ERROR("getNearestTruePoint: No unoccupied point found");
        return point;
    }

}
