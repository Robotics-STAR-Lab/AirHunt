
#include <plan_manage/planner_manager.h>
#include <exploration_manager/fast_exploration_manager.h>
#include <traj_utils/planning_visualization.h>

#include <exploration_manager/fast_exploration_fsm.h>
#include <exploration_manager/expl_data.h>
#include <plan_env/edt_environment.h>
#include <plan_env/sdf_map.h>

using Eigen::Vector4d;

namespace fast_planner
{
    void FastExplorationFSM::init(ros::NodeHandle &nh)
    {
        fp_.reset(new FSMParam);
        fd_.reset(new FSMData);

        nh.param("fsm/thresh_replan1", fp_->replan_thresh1_, -1.0);
        nh.param("fsm/thresh_replan2", fp_->replan_thresh2_, -1.0);
        nh.param("fsm/thresh_replan3", fp_->replan_thresh3_, -1.0);
        nh.param("fsm/replan_time", fp_->replan_time_, -1.0);

        expl_manager_.reset(new FastExplorationManager);
        expl_manager_->initialize(nh);
        visualization_.reset(new PlanningVisualization(nh));

        planner_manager_ = expl_manager_->planner_manager_;
        state_ = EXPL_STATE::INIT;
        fd_->have_odom_ = false;
        fd_->state_str_ = {"INIT", "SPIN", "WAIT_TRIGGER", "PLAN_TRAJ", "PUB_TRAJ", "EXEC_TRAJ", "FINISH"};
        fd_->static_state_ = true;
        fd_->trigger_ = false;
        task2_mode_ = false;
        task2_pose_received_ = false;
        task2_target_pose_.setZero();
        task2_observe_point_.setZero();
        is_spinning_ = false;
        spin_start_time_ = ros::Time(0);
        spin_count_ = 0;

        exec_timer_ = nh.createTimer(ros::Duration(0.01), &FastExplorationFSM::FSMCallback, this);
        safety_timer_ = nh.createTimer(ros::Duration(0.05), &FastExplorationFSM::safetyCallback, this);
        frontier_timer_ = nh.createTimer(ros::Duration(0.5), &FastExplorationFSM::frontierCallback, this);

        trigger_sub_ =
            nh.subscribe("/waypoint_generator/waypoints", 1, &FastExplorationFSM::triggerCallback, this);
        odom_sub_ = nh.subscribe("/odom_world", 1, &FastExplorationFSM::odometryCallback, this);
        task2_pose_sub_ = nh.subscribe("/task2/pose", 1,
                                       &FastExplorationFSM::task2PoseCallback, this);
        task1_pose_sub_ = nh.subscribe("/task1/pose", 1,
                                       &FastExplorationFSM::task1PoseCallback, this);
        replan_pub_ = nh.advertise<std_msgs::Empty>("/planning/replan", 10);
        new_pub_ = nh.advertise<std_msgs::Empty>("/planning/new", 10);
        bspline_pub_ = nh.advertise<bspline::Bspline>("/planning/bspline", 10);

        fsm_state_pub_ = nh.advertise<std_msgs::String>("/fsm_state", 10, true);
    }

    void FastExplorationFSM::FSMCallback(const ros::TimerEvent &e)
    {
        ROS_INFO_STREAM_THROTTLE(1.0, "[FSM]: state: " << fd_->state_str_[int(state_)]);

        switch (state_)
        {
        case INIT:
        {

            if (!fd_->have_odom_)
            {
                ROS_WARN_THROTTLE(1.0, "no odom.");
                return;
            }

            transitState(SPIN, "FSM");
            break;
        }

        case SPIN:
        {
            if (fd_->static_state_)
            {

                if (spin_count_ == 0 && spin_start_time_.isZero())
                {
                    spin_start_time_ = ros::Time::now();
                    ROS_INFO("Entered SPIN state, waiting 1 second for stabilization...");
                    return;
                }

                if (spin_count_ == 0)
                {
                    double elapsed_time = (ros::Time::now() - spin_start_time_).toSec();
                    if (elapsed_time < 1.0)
                        return;
                    spin_count_ = 1;
                    ROS_INFO("Starting first 180-degree rotation (1/2)");
                }

                fd_->start_pt_ = fd_->odom_pos_;
                fd_->start_vel_ = fd_->odom_vel_;
                fd_->start_acc_.setZero();

                fd_->start_yaw_(0) = fd_->odom_yaw_;
                fd_->start_yaw_(1) = fd_->start_yaw_(2) = 0.0;

                replan_pub_.publish(std_msgs::Empty());
                int res = callSpinPlanner();
                if (res == SUCCEED)
                {
                    is_spinning_ = true;
                    spin_start_time_ = ros::Time(0);
                    transitState(PUB_TRAJ, "FSM");
                    ROS_INFO("Spinning trajectory planned successfully (rotation %d/2)", spin_count_);
                }
                else if (res == FAIL)
                {
                    ROS_WARN("Spin plan failed, transitioning to WAIT_TRIGGER");
                    spin_start_time_ = ros::Time(0);
                    spin_count_ = 0;
                    transitState(WAIT_TRIGGER, "FSM");
                    fd_->static_state_ = true;
                }
            }
            break;
        }

        case WAIT_TRIGGER:
        {

            ROS_WARN_THROTTLE(1.0, "wait for trigger.");
            break;
        }

        case FINISH:
        {
            ROS_INFO_THROTTLE(1.0, "finish exploration.");
            break;
        }

        case PLAN_TRAJ:
        {
            if (fd_->static_state_)
            {

                fd_->start_pt_ = fd_->odom_pos_;
                fd_->start_vel_ = fd_->odom_vel_;
                fd_->start_acc_.setZero();

                fd_->start_yaw_(0) = fd_->odom_yaw_;
                fd_->start_yaw_(1) = fd_->start_yaw_(2) = 0.0;
            }
            else
            {

                LocalTrajData *info = &planner_manager_->local_data_;
                double t_r = (ros::Time::now() - info->start_time_).toSec() + fp_->replan_time_;

                fd_->start_pt_ = info->position_traj_.evaluateDeBoorT(t_r);
                fd_->start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_r);
                fd_->start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_r);
                fd_->start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_r)[0];
                fd_->start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_r)[0];
                fd_->start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_r)[0];
            }

            replan_pub_.publish(std_msgs::Empty());
            int res = callExplorationPlanner();
            if (res == SUCCEED)
            {
                transitState(PUB_TRAJ, "FSM");
            }
            else if (res == NO_FRONTIER)
            {
                transitState(FINISH, "FSM");
                fd_->static_state_ = true;

            }
            else if (res == FAIL)
            {

                ROS_WARN("plan fail");
                fd_->static_state_ = true;
            }
            break;
        }

        case PUB_TRAJ:
        {
            double dt = (ros::Time::now() - fd_->newest_traj_.start_time).toSec();
            if (dt > 0)
            {
                bspline_pub_.publish(fd_->newest_traj_);
                fd_->static_state_ = false;
                transitState(EXEC_TRAJ, "FSM");

                thread vis_thread(&FastExplorationFSM::visualize, this);
                vis_thread.detach();
            }
            break;
        }

        case EXEC_TRAJ:
        {
            LocalTrajData *info = &planner_manager_->local_data_;
            double t_cur = (ros::Time::now() - info->start_time_).toSec();

            if (is_spinning_)
            {
                double time_to_end = info->duration_ - t_cur;
                if (time_to_end <= 0.0)
                {
                    ROS_INFO("Spin trajectory %d/2 completed", spin_count_);
                    is_spinning_ = false;
                    fd_->static_state_ = true;

                    if (spin_count_ < 2)
                    {

                        spin_count_++;
                        ROS_INFO("Starting second 180-degree rotation (2/2)");
                        transitState(SPIN, "FSM");
                        return;
                    }
                    else
                    {

                        ROS_INFO("Full 360-degree rotation completed, transitioning to WAIT_TRIGGER");
                        spin_count_ = 0;
                        transitState(WAIT_TRIGGER, "FSM");
                        return;
                    }
                }

                ROS_INFO_THROTTLE(1.0, "Spinning... Time remaining: %.2f seconds (rotation %d/2)", time_to_end, spin_count_);
                return;
            }

            if (task2_mode_)
            {
                double dist_to_observe_point = (fd_->odom_pos_ - task2_observe_point_).norm();
                if (dist_to_observe_point < 1.0)
                {
                    ROS_INFO("Task2: Reached observation point [%.2f, %.2f, %.2f]. Mission completed!",
                             task2_observe_point_[0], task2_observe_point_[1], task2_observe_point_[2]);
                    transitState(FINISH, "FSM");
                    fd_->static_state_ = true;
                    return;
                }
            }

            double time_to_end = info->duration_ - t_cur;
            if (time_to_end < fp_->replan_thresh1_)
            {
                transitState(PLAN_TRAJ, "FSM");
                ROS_WARN("Replan: traj fully executed=================================");
                return;
            }

            if (t_cur > fp_->replan_thresh2_ && expl_manager_->frontier_finder_->isFrontierCovered())
            {
                transitState(PLAN_TRAJ, "FSM");
                ROS_WARN("Replan: cluster covered=====================================");
                return;
            }

            if (t_cur > fp_->replan_thresh3_ && !classic_)
            {
                transitState(PLAN_TRAJ, "FSM");
                ROS_WARN("Replan: periodic call=======================================");
            }
            break;
        }
        }
    }

    int FastExplorationFSM::callExplorationPlanner()
    {
        ros::Time time_r = ros::Time::now() + ros::Duration(fp_->replan_time_);

        int res;
        if (task2_mode_)
        {

            ROS_INFO("Using task2 path planning logic to target: [%.2f, %.2f, %.2f]",
                     task2_target_pose_(0), task2_target_pose_(1), task2_target_pose_(2));
            res = expl_manager_->planTask2Path(
                fd_->start_pt_, fd_->start_vel_, fd_->start_acc_, fd_->start_yaw_, task2_target_pose_, task2_observe_point_);
        }
        else
        {

            ROS_INFO("Using original exploration logic");
            res = expl_manager_->planExploreMotion(fd_->start_pt_, fd_->start_vel_, fd_->start_acc_,
                                                   fd_->start_yaw_);
        }

        classic_ = false;

        if (res == SUCCEED)
        {
            auto info = &planner_manager_->local_data_;
            info->start_time_ = (ros::Time::now() - time_r).toSec() > 0 ? ros::Time::now() : time_r;

            bspline::Bspline bspline;
            bspline.order = planner_manager_->pp_.bspline_degree_;
            bspline.start_time = info->start_time_;
            bspline.traj_id = info->traj_id_;
            Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
            for (int i = 0; i < pos_pts.rows(); ++i)
            {
                geometry_msgs::Point pt;
                pt.x = pos_pts(i, 0);
                pt.y = pos_pts(i, 1);
                pt.z = pos_pts(i, 2);
                bspline.pos_pts.push_back(pt);
            }
            Eigen::VectorXd knots = info->position_traj_.getKnot();
            for (int i = 0; i < knots.rows(); ++i)
            {
                bspline.knots.push_back(knots(i));
            }
            Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
            for (int i = 0; i < yaw_pts.rows(); ++i)
            {
                double yaw = yaw_pts(i, 0);
                bspline.yaw_pts.push_back(yaw);
            }
            bspline.yaw_dt = info->yaw_traj_.getKnotSpan();
            fd_->newest_traj_ = bspline;
        }
        return res;
    }

    int FastExplorationFSM::callSpinPlanner()
    {
        ros::Time time_r = ros::Time::now() + ros::Duration(fp_->replan_time_);

        double current_yaw = fd_->start_yaw_(0);
        double rotation_angle = M_PI - 0.05;
        double target_yaw = current_yaw + rotation_angle;

        ROS_INFO("Spin %d/2: current_yaw=%.1f°, target_yaw=%.1f° (clockwise rotation)",
                 spin_count_, current_yaw * 180.0 / M_PI, target_yaw * 180.0 / M_PI);

        bool res = planner_manager_->planInPlaceYawRotation(fd_->start_pt_, fd_->start_vel_, fd_->start_acc_,
                                                            fd_->start_yaw_, target_yaw);

        if (res)
        {
            auto info = &planner_manager_->local_data_;
            info->start_time_ = (ros::Time::now() - time_r).toSec() > 0 ? ros::Time::now() : time_r;

            bspline::Bspline bspline;
            bspline.order = planner_manager_->pp_.bspline_degree_;
            bspline.start_time = info->start_time_;
            bspline.traj_id = info->traj_id_;
            Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
            for (int i = 0; i < pos_pts.rows(); ++i)
            {
                geometry_msgs::Point pt;
                pt.x = pos_pts(i, 0);
                pt.y = pos_pts(i, 1);
                pt.z = pos_pts(i, 2);
                bspline.pos_pts.push_back(pt);
            }
            Eigen::VectorXd knots = info->position_traj_.getKnot();
            for (int i = 0; i < knots.rows(); ++i)
            {
                bspline.knots.push_back(knots(i));
            }
            Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
            for (int i = 0; i < yaw_pts.rows(); ++i)
            {
                double yaw = yaw_pts(i, 0);
                bspline.yaw_pts.push_back(yaw);
            }
            bspline.yaw_dt = info->yaw_traj_.getKnotSpan();
            fd_->newest_traj_ = bspline;
            return SUCCEED;
        }
        return FAIL;
    }

    void FastExplorationFSM::visualize()
    {
        auto info = &planner_manager_->local_data_;
        auto plan_data = &planner_manager_->plan_data_;
        auto ed_ptr = expl_manager_->ed_;

        if (task2_mode_)
        {
            clearVisMarker();

            if (task2_pose_received_)
            {
                visualization_->drawSpheres({task2_target_pose_}, 0.5, Vector4d(1, 0, 0, 1), "task2_target", 0, 6);
            }

            if (task2_observe_point_.norm() > 0.1)
            {
                visualization_->drawSpheres({task2_observe_point_}, 0.4, Vector4d(0, 0, 1, 1), "task2_observe", 0, 6);
            }
            return;
        }
        static int last_ftr_num = 0;
        for (int i = 0; i < ed_ptr->frontiers_.size(); ++i)
        {
            visualization_->drawCubes(ed_ptr->frontiers_[i], 0.1,
                                      visualization_->getColor(double(i) / ed_ptr->frontiers_.size(), 0.4),
                                      "frontier", i, 4);

        }
        for (int i = ed_ptr->frontiers_.size(); i < last_ftr_num; ++i)
        {
            visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i, 4);

        }
        last_ftr_num = ed_ptr->frontiers_.size();

        visualization_->drawSpheres(ed_ptr->points_, 0.2, Vector4d(0, 0.5, 0, 1), "points", 0, 6);
        visualization_->drawLines(ed_ptr->global_tour_, 0.07, Vector4d(0, 0.5, 0, 1), "global_tour", 0, 6);
        visualization_->drawLines(ed_ptr->points_, ed_ptr->views_, 0.05, Vector4d(0, 1, 0.5, 1), "view", 0, 6);
        visualization_->drawLines(ed_ptr->points_, ed_ptr->averages_, 0.03, Vector4d(1, 0, 0, 1),
                                  "point-average", 0, 6);

        visualization_->drawSpheres(ed_ptr->refined_points_, 0.2, Vector4d(0, 0, 1, 1), "refined_pts", 0, 6);
        visualization_->drawLines(ed_ptr->refined_points_, ed_ptr->refined_views_, 0.05,
                                  Vector4d(0.5, 0, 1, 1), "refined_view", 0, 6);
        visualization_->drawLines(ed_ptr->refined_tour_, 0.07, Vector4d(0, 0, 1, 1), "refined_tour", 0, 6);
        visualization_->drawLines(ed_ptr->refined_views1_, ed_ptr->refined_views2_, 0.04, Vector4d(0, 0, 0, 1),
                                  "refined_view", 0, 6);
        visualization_->drawLines(ed_ptr->refined_points_, ed_ptr->unrefined_points_, 0.05, Vector4d(1, 1, 0, 1),
                                  "refine_pair", 0, 6);
        for (int i = 0; i < ed_ptr->n_points_.size(); ++i)
            visualization_->drawSpheres(ed_ptr->n_points_[i], 0.1,
                                        visualization_->getColor(double(ed_ptr->refined_ids_[i]) /
                                                                 ed_ptr->frontiers_.size()),
                                        "n_points", i, 6);
        for (int i = ed_ptr->n_points_.size(); i < 15; ++i)
            visualization_->drawSpheres({}, 0.1, Vector4d(0, 0, 0, 1), "n_points", i, 6);

        visualization_->drawSpheres({ed_ptr->next_goal_}, 0.3, Vector4d(0, 1, 1, 1), "next_goal", 0, 6);
        visualization_->drawBspline(info->position_traj_, 0.1, Vector4d(1.0, 0.0, 0.0, 1), false, 0.15,
                                    Vector4d(1, 1, 0, 1));

    }

    void FastExplorationFSM::clearVisMarker()
    {
        visualization_->drawSpheres({}, 0.2, Vector4d(0, 0.5, 0, 1), "points", 0, 6);
        visualization_->drawLines({}, 0.07, Vector4d(0, 0.5, 0, 1), "global_tour", 0, 6);
        visualization_->drawSpheres({}, 0.2, Vector4d(0, 0, 1, 1), "refined_pts", 0, 6);
        visualization_->drawLines({}, {}, 0.05, Vector4d(0.5, 0, 1, 1), "refined_view", 0, 6);
        visualization_->drawLines({}, 0.07, Vector4d(0, 0, 1, 1), "refined_tour", 0, 6);
        visualization_->drawSpheres({}, 0.1, Vector4d(0, 0, 1, 1), "B-Spline", 0, 0);

    }

    void FastExplorationFSM::frontierCallback(const ros::TimerEvent &e)
    {
        static int delay = 0;
        if (++delay < 5)
            return;

        if (state_ == WAIT_TRIGGER || state_ == FINISH)
        {
            auto ft = expl_manager_->frontier_finder_;
            auto ed = expl_manager_->ed_;
            ft->searchFrontiers();
            ft->computeFrontiersToVisit();
            ft->updateFrontierCostMatrix();

            ft->getFrontiers(ed->frontiers_);
            ft->getFrontierBoxes(ed->frontier_boxes_);

            for (int i = 0; i < ed->frontiers_.size(); ++i)
            {
                visualization_->drawCubes(ed->frontiers_[i], 0.1,
                                          visualization_->getColor(double(i) / ed->frontiers_.size(), 0.4),
                                          "frontier", i, 4);

            }
            for (int i = ed->frontiers_.size(); i < 50; ++i)
            {
                visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i, 4);

            }
        }

    }

    void FastExplorationFSM::triggerCallback(const nav_msgs::PathConstPtr &msg)
    {
        if (msg->poses[0].pose.position.z < -0.1)
            return;
        if (state_ != WAIT_TRIGGER)
            return;
        fd_->trigger_ = true;
        cout << "Triggered!" << endl;
        transitState(PLAN_TRAJ, "triggerCallback");
    }

    void FastExplorationFSM::safetyCallback(const ros::TimerEvent &e)
    {
        if (state_ == EXPL_STATE::EXEC_TRAJ)
        {

            double dist;
            bool safe = planner_manager_->checkTrajCollision(dist);
            if (!safe)
            {
                ROS_WARN("Replan: collision detected==================================");
                transitState(PLAN_TRAJ, "safetyCallback");
            }
        }
    }

    void FastExplorationFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
    {
        fd_->odom_pos_(0) = msg->pose.pose.position.x;
        fd_->odom_pos_(1) = msg->pose.pose.position.y;
        fd_->odom_pos_(2) = msg->pose.pose.position.z;

        fd_->odom_vel_(0) = msg->twist.twist.linear.x;
        fd_->odom_vel_(1) = msg->twist.twist.linear.y;
        fd_->odom_vel_(2) = msg->twist.twist.linear.z;

        fd_->odom_orient_.w() = msg->pose.pose.orientation.w;
        fd_->odom_orient_.x() = msg->pose.pose.orientation.x;
        fd_->odom_orient_.y() = msg->pose.pose.orientation.y;
        fd_->odom_orient_.z() = msg->pose.pose.orientation.z;

        Eigen::Vector3d rot_x = fd_->odom_orient_.toRotationMatrix().block<3, 1>(0, 0);
        fd_->odom_yaw_ = atan2(rot_x(1), rot_x(0));

        fd_->have_odom_ = true;
    }
    void FastExplorationFSM::task2PoseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
    {

        task2_target_pose_(0) = msg->pose.position.x;
        task2_target_pose_(1) = msg->pose.position.y;
        task2_target_pose_(2) = msg->pose.position.z;

        ROS_INFO("Task2 pose received: [%.2f, %.2f, %.2f] - Switching to task2 planning mode.",
                 task2_target_pose_(0), task2_target_pose_(1), task2_target_pose_(2));

        task2_pose_received_ = true;
        task2_mode_ = true;

        if (state_ == EXEC_TRAJ || state_ == PUB_TRAJ)
        {
            ROS_WARN("Task2 pose received - interrupting current trajectory and replanning immediately");
            fd_->static_state_ = true;
            transitState(PLAN_TRAJ, "task2PoseCallback");
        }
        else if (state_ == WAIT_TRIGGER)
        {
            ROS_WARN("Task2 pose received - starting task2 planning");
            transitState(PLAN_TRAJ, "task2PoseCallback");
        }
    }
    void FastExplorationFSM::task1PoseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
    {

        if (state_ == WAIT_TRIGGER)
        {

            ROS_INFO("Task1 pose triggered FSM transition from WAIT_TRIGGER to PLAN_TRAJ");
            transitState(PLAN_TRAJ, "task1PoseCallback");
        }
        else
        {
            ROS_WARN("Task1 pose received but FSM is not in WAIT_TRIGGER state (current: %s)",
                     fd_->state_str_[int(state_)].c_str());
        }
    }

    void FastExplorationFSM::transitState(EXPL_STATE new_state, string pos_call)
    {
        int pre_s = int(state_);
        state_ = new_state;
        cout << "[" + pos_call + "]: from " + fd_->state_str_[pre_s] + " to " + fd_->state_str_[int(new_state)]
             << endl;

        std_msgs::String fsm_state_msg;
        fsm_state_msg.data = fd_->state_str_[int(new_state)];
        fsm_state_pub_.publish(fsm_state_msg);
    }

}
