#include <plan_env/sdf_map.h>
#include <plan_env/map_ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#include <opencv2/imgproc.hpp>
#include <mutex>

#include <fstream>
#include <iomanip>

namespace fast_planner
{
    MapROS::MapROS()
    {
    }

    MapROS::~MapROS()
    {
    }

    void MapROS::setMap(SDFMap *map)
    {
        this->map_ = map;
    }

    void MapROS::init()
    {
        node_.param("map_ros/fx", fx_, -1.0);
        node_.param("map_ros/fy", fy_, -1.0);
        node_.param("map_ros/cx", cx_, -1.0);
        node_.param("map_ros/cy", cy_, -1.0);
        node_.param("map_ros/depth_filter_maxdist", depth_filter_maxdist_, -1.0);
        node_.param("map_ros/value_depth_filter_maxdist", value_depth_filter_maxdist_, -1.0);
        node_.param("map_ros/depth_filter_mindist", depth_filter_mindist_, -1.0);
        node_.param("map_ros/depth_filter_margin", depth_filter_margin_, -1);
        node_.param("map_ros/k_depth_scaling_factor", k_depth_scaling_factor_, -1.0);
        node_.param("map_ros/skip_pixel", skip_pixel_, -1);

        node_.param("map_ros/esdf_slice_height", esdf_slice_height_, -0.1);
        node_.param("map_ros/visualization_truncate_height", visualization_truncate_height_, -0.1);
        node_.param("map_ros/visualization_truncate_low", visualization_truncate_low_, -0.1);
        node_.param("map_ros/show_occ_time", show_occ_time_, false);
        node_.param("map_ros/show_esdf_time", show_esdf_time_, false);
        node_.param("map_ros/show_all_map", show_all_map_, false);
        node_.param("map_ros/frame_id", frame_id_, string("world"));
        node_.param("map_ros/value_map_alpha", value_map_alpha_, 0.3);

        proj_points_.resize(640 * 480 / (skip_pixel_ * skip_pixel_));
        point_cloud_.points.resize(640 * 480 / (skip_pixel_ * skip_pixel_));
        merged_point_cloud_.points.resize(640 * 480 * 3 / (skip_pixel_ * skip_pixel_));

        proj_points_cnt = 0;

        local_updated_ = false;
        esdf_need_update_ = false;
        fuse_time_ = 0.0;
        esdf_time_ = 0.0;
        max_fuse_time_ = 0.0;
        max_esdf_time_ = 0.0;
        fuse_num_ = 0;
        esdf_num_ = 0;
        depth_image_.reset(new cv::Mat);

        rand_noise_ = normal_distribution<double>(0, 0.1);
        random_device rd;
        eng_ = default_random_engine(rd());

        esdf_timer_ = node_.createTimer(ros::Duration(0.05), &MapROS::updateESDFCallback, this);
        vis_timer_ = node_.createTimer(ros::Duration(0.05), &MapROS::visCallback, this);

        map_all_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_all", 10);
        map_local_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_local", 10);
        map_local_inflate_pub_ =
            node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_local_inflate", 10);
        unknown_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/unknown", 10);
        esdf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/esdf", 10);
        update_range_pub_ = node_.advertise<visualization_msgs::Marker>("/sdf_map/update_range", 10);
        depth_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/depth_cloud", 10);
        debug_pub_ = node_.advertise<geometry_msgs::PoseStamped>("/sdf_map/debug_pose", 10);
        value_map_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/value_map", 10);

        depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node_, "/map_ros/depth", 50));
        cloud_sub_.reset(
            new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_, "/map_ros/cloud", 50));
        pose_sub_.reset(
            new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_, "/map_ros/pose", 25));

        task1_depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(
            node_, "/task1/depth", 50));
        task1_pose_sub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(
            node_, "/task1/pose", 50));
        sync_image_pose_.reset(new message_filters::Synchronizer<MapROS::SyncPolicyImagePose>(
            MapROS::SyncPolicyImagePose(100), *depth_sub_, *pose_sub_));
        sync_image_pose_->registerCallback(boost::bind(&MapROS::depthPoseCallback, this, _1, _2));

        sync_task1_value_pose_.reset(
            new message_filters::Synchronizer<MapROS::SyncPolicyImagePose>(
                MapROS::SyncPolicyImagePose(100), *task1_depth_sub_, *task1_pose_sub_));
        sync_task1_value_pose_->registerCallback(
            boost::bind(&MapROS::valueDepthPoseCallback, this, _1, _2));

        left_depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node_, "/camera/left/depth", 100));
        left_pose_sub_.reset(
            new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_, "/camera/left/pose", 100));
        sync_left_image_pose_.reset(new message_filters::Synchronizer<MapROS::SyncPolicyImagePose>(
            MapROS::SyncPolicyImagePose(200), *left_depth_sub_, *left_pose_sub_));
        sync_left_image_pose_->registerCallback(boost::bind(&MapROS::leftDepthPoseCallback, this, _1, _2));

        right_depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node_, "/camera/right/depth", 100));
        right_pose_sub_.reset(
            new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_, "/camera/right/pose", 100));
        sync_right_image_pose_.reset(new message_filters::Synchronizer<MapROS::SyncPolicyImagePose>(
            MapROS::SyncPolicyImagePose(200), *right_depth_sub_, *right_pose_sub_));
        sync_right_image_pose_->registerCallback(boost::bind(&MapROS::rightDepthPoseCallback, this, _1, _2));

        map_start_time_ = ros::Time::now();
    }

    void MapROS::visCallback(const ros::TimerEvent &e)
    {
        publishMapLocal();
        if (show_all_map_)
        {

            static double tpass = 0.0;
            tpass += (e.current_real - e.last_real).toSec();
            if (tpass > 0.1)
            {
                publishMapAll();
                tpass = 0.0;
            }
        }
        publishValueMap();

    }

    void MapROS::updateESDFCallback(const ros::TimerEvent & )
    {
        if (!esdf_need_update_)
            return;
        auto t1 = ros::Time::now();

        map_->updateESDF3d();
        esdf_need_update_ = false;

        auto t2 = ros::Time::now();
        esdf_time_ += (t2 - t1).toSec();
        max_esdf_time_ = max(max_esdf_time_, (t2 - t1).toSec());
        esdf_num_++;
        if (show_esdf_time_)
            ROS_WARN("ESDF t: cur: %lf, avg: %lf, max: %lf", (t2 - t1).toSec(), esdf_time_ / esdf_num_,
                     max_esdf_time_);
    }

    void MapROS::depthPoseCallback(const sensor_msgs::ImageConstPtr &img,
                                   const geometry_msgs::PoseStampedConstPtr &pose)
    {
        camera_pos_(0) = pose->pose.position.x;
        camera_pos_(1) = pose->pose.position.y;
        camera_pos_(2) = pose->pose.position.z;
        if (!map_->isInMap(camera_pos_))
            return;

        camera_q_ = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
                                       pose->pose.orientation.y, pose->pose.orientation.z);
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
        if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
            (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, k_depth_scaling_factor_);
        cv_ptr->image.copyTo(*depth_image_);

        auto t1 = ros::Time::now();

        proessDepthImage();

        {
            std::lock_guard<std::mutex> lock(point_cloud_mutex_);
            for (int i = 0; i < proj_points_cnt; ++i) {
                merged_point_cloud_.points[i] = point_cloud_.points[i];
            }
            merged_point_cloud_.width = proj_points_cnt;
        }
        map_->inputPointCloud(point_cloud_, proj_points_cnt, camera_pos_);
        if (local_updated_)
        {
            map_->clearAndInflateLocalMap();
            esdf_need_update_ = true;
            local_updated_ = false;
        }

        auto t2 = ros::Time::now();
        fuse_time_ += (t2 - t1).toSec();
        max_fuse_time_ = max(max_fuse_time_, (t2 - t1).toSec());
        fuse_num_ += 1;
        if (show_occ_time_)
            ROS_WARN("Fusion t: cur: %lf, avg: %lf, max: %lf", (t2 - t1).toSec(), fuse_time_ / fuse_num_,
                     max_fuse_time_);
    }

    void MapROS::valueDepthPoseCallback(const sensor_msgs::ImageConstPtr &img,
                                        const geometry_msgs::PoseStampedConstPtr &pose)
    {

        cout << "valueDepthPoseCallback - task1_pose seq: " << pose->header.seq
             << ", current ROS time: " << ros::Time::now() << endl;

        camera_pos_(0) = pose->pose.position.x;
        camera_pos_(1) = pose->pose.position.y;
        camera_pos_(2) = pose->pose.position.z;
        if (!map_->isInMap(camera_pos_))
            return;

        double value = 0.0;
        if (!pose->header.frame_id.empty())
        {
            value = std::atof(pose->header.frame_id.c_str());
            if (value == 0.0 && pose->header.frame_id != "0" && pose->header.frame_id != "0.0")
            {
                ROS_WARN("Failed to parse value from frame_id: %s", pose->header.frame_id.c_str());
            }
        }

        camera_q_ =
            Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
                               pose->pose.orientation.y, pose->pose.orientation.z);
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
        if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
            (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, k_depth_scaling_factor_);
        cv_ptr->image.copyTo(*depth_image_);

        auto t1 = ros::Time::now();

        generateAndUpdateValueMapWithAirVoxels(value);

        if (local_updated_)
        {
            map_->clearAndInflateLocalMap();
            esdf_need_update_ = true;
            local_updated_ = false;
        }

        auto t2 = ros::Time::now();

        fuse_time_ += (t2 - t1).toSec();
        max_fuse_time_ = max(max_fuse_time_, (t2 - t1).toSec());
        fuse_num_ += 1;

        ROS_WARN("Value Fusion t: cur: %lf, avg: %lf, max: %lf", (t2 - t1).toSec(),
                    fuse_time_ / fuse_num_, max_fuse_time_);


        cout << "valueDepthPoseCallback finished: "
             << ", current ROS time: " << ros::Time::now() << endl;

    }

    void MapROS::proessDepthImage()
    {
        proj_points_cnt = 0;

        uint16_t *row_ptr;

        int cols = depth_image_->cols;
        int rows = depth_image_->rows;
        double depth;
        Eigen::Matrix3d camera_r = camera_q_.toRotationMatrix();

        Eigen::Matrix3d R_trans = Eigen::Matrix3d::Zero();
        R_trans(0, 2) = 1;
        R_trans(1, 0) = -1;
        R_trans(2, 1) = -1;

        camera_r = camera_r * R_trans;
        Eigen::Quaterniond new_q(camera_r);

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "world";
        pose.pose.position.x = camera_pos_(0);
        pose.pose.position.y = camera_pos_(1);
        pose.pose.position.z = camera_pos_(2);
        pose.pose.orientation.w = new_q.w();
        pose.pose.orientation.x = new_q.x();
        pose.pose.orientation.y = new_q.y();
        pose.pose.orientation.z = new_q.z();
        debug_pub_.publish(pose);

        Eigen::Vector3d pt_cur, pt_world;
        const double inv_factor = 1.0 / k_depth_scaling_factor_;

        for (int v = depth_filter_margin_; v < rows - depth_filter_margin_; v += skip_pixel_)
        {
            row_ptr = depth_image_->ptr<uint16_t>(v) + depth_filter_margin_;
            for (int u = depth_filter_margin_; u < cols - depth_filter_margin_; u += skip_pixel_)
            {
                depth = (*row_ptr) * inv_factor;
                row_ptr = row_ptr + skip_pixel_;

                if (*row_ptr == 0 || depth > depth_filter_maxdist_)
                    depth = depth_filter_maxdist_;
                else if (depth < depth_filter_mindist_)
                    continue;

                pt_cur(0) = (u - cx_) * depth / fx_;
                pt_cur(1) = (v - cy_) * depth / fy_;
                pt_cur(2) = depth;
                pt_world = camera_r * pt_cur + camera_pos_;
                auto &pt = point_cloud_.points[proj_points_cnt++];
                pt.x = pt_world[0];
                pt.y = pt_world[1];
                pt.z = pt_world[2];
            }
        }

        publishDepth();
    }

    void MapROS::publishMapAll()
    {
        pcl::PointXYZ pt;
        pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2;
        for (int x = map_->mp_->box_min_(0) ; x < map_->mp_->box_max_(0); ++x)
            for (int y = map_->mp_->box_min_(1) ; y < map_->mp_->box_max_(1); ++y)
                for (int z = map_->mp_->box_min_(2) ; z < map_->mp_->box_max_(2); ++z)
                {
                    if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] > map_->mp_->min_occupancy_log_)
                    {
                        Eigen::Vector3d pos;
                        map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
                        if (pos(2) > visualization_truncate_height_)
                            continue;
                        if (pos(2) < visualization_truncate_low_)
                            continue;
                        pt.x = pos(0);
                        pt.y = pos(1);
                        pt.z = pos(2);
                        cloud1.push_back(pt);
                    }
                }
        cloud1.width = cloud1.points.size();
        cloud1.height = 1;
        cloud1.is_dense = true;
        cloud1.header.frame_id = frame_id_;
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud1, cloud_msg);
        map_all_pub_.publish(cloud_msg);


    }

    void MapROS::publishMapLocal()
    {
        pcl::PointXYZ pt;
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointCloud<pcl::PointXYZ> cloud2;
        Eigen::Vector3i min_cut = map_->md_->local_bound_min_;
        Eigen::Vector3i max_cut = map_->md_->local_bound_max_;
        map_->boundIndex(min_cut);
        map_->boundIndex(max_cut);

        for (int x = min_cut(0); x <= max_cut(0); ++x)
            for (int y = min_cut(1); y <= max_cut(1); ++y)
                for (int z = map_->mp_->box_min_(2); z < map_->mp_->box_max_(2); ++z)
                {
                    if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] > map_->mp_->min_occupancy_log_)
                    {

                        Eigen::Vector3d pos;
                        map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
                        if (pos(2) > visualization_truncate_height_)
                            continue;
                        if (pos(2) < visualization_truncate_low_)
                            continue;

                        pt.x = pos(0);
                        pt.y = pos(1);
                        pt.z = pos(2);
                        cloud.push_back(pt);
                    }

                }

        cloud.width = cloud.points.size();
        cloud.height = 1;
        cloud.is_dense = true;
        cloud.header.frame_id = frame_id_;
        cloud2.width = cloud2.points.size();
        cloud2.height = 1;
        cloud2.is_dense = true;
        cloud2.header.frame_id = frame_id_;
        sensor_msgs::PointCloud2 cloud_msg;

        pcl::toROSMsg(cloud, cloud_msg);
        map_local_pub_.publish(cloud_msg);
        pcl::toROSMsg(cloud2, cloud_msg);
        map_local_inflate_pub_.publish(cloud_msg);
    }

    void MapROS::publishUnknown()
    {
        pcl::PointXYZ pt;
        pcl::PointCloud<pcl::PointXYZ> cloud;
        Eigen::Vector3i min_cut = map_->md_->local_bound_min_;
        Eigen::Vector3i max_cut = map_->md_->local_bound_max_;
        map_->boundIndex(max_cut);
        map_->boundIndex(min_cut);

        for (int x = min_cut(0); x <= max_cut(0); ++x)
            for (int y = min_cut(1); y <= max_cut(1); ++y)
                for (int z = min_cut(2); z <= max_cut(2); ++z)
                {
                    if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] < map_->mp_->clamp_min_log_ - 1e-3)
                    {
                        Eigen::Vector3d pos;
                        map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
                        if (pos(2) > visualization_truncate_height_)
                            continue;
                        if (pos(2) < visualization_truncate_low_)
                            continue;
                        pt.x = pos(0);
                        pt.y = pos(1);
                        pt.z = pos(2);
                        cloud.push_back(pt);
                    }
                }
        cloud.width = cloud.points.size();
        cloud.height = 1;
        cloud.is_dense = true;
        cloud.header.frame_id = frame_id_;
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud, cloud_msg);
        unknown_pub_.publish(cloud_msg);
    }

    void MapROS::publishDepth()
    {
        std::lock_guard<std::mutex> lock(point_cloud_mutex_);
        pcl::PointCloud<pcl::PointXYZ> cloud;

        for (size_t i = 0; i < merged_point_cloud_.width && i < merged_point_cloud_.points.size(); ++i) {
            cloud.push_back(merged_point_cloud_.points[i]);
        }

        cloud.width = cloud.points.size();
        cloud.height = 1;
        cloud.is_dense = true;
        cloud.header.frame_id = frame_id_;
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud, cloud_msg);
        depth_pub_.publish(cloud_msg);
    }

    void MapROS::publishUpdateRange()
    {
        Eigen::Vector3d esdf_min_pos, esdf_max_pos, cube_pos, cube_scale;
        visualization_msgs::Marker mk;
        map_->indexToPos(map_->md_->local_bound_min_, esdf_min_pos);
        map_->indexToPos(map_->md_->local_bound_max_, esdf_max_pos);

        cube_pos = 0.5 * (esdf_min_pos + esdf_max_pos);
        cube_scale = esdf_max_pos - esdf_min_pos;
        mk.header.frame_id = frame_id_;
        mk.header.stamp = ros::Time::now();
        mk.type = visualization_msgs::Marker::CUBE;
        mk.action = visualization_msgs::Marker::ADD;
        mk.id = 0;
        mk.pose.position.x = cube_pos(0);
        mk.pose.position.y = cube_pos(1);
        mk.pose.position.z = cube_pos(2);
        mk.scale.x = cube_scale(0);
        mk.scale.y = cube_scale(1);
        mk.scale.z = cube_scale(2);
        mk.color.a = 0.3;
        mk.color.r = 1.0;
        mk.color.g = 0.0;
        mk.color.b = 0.0;
        mk.pose.orientation.w = 1.0;
        mk.pose.orientation.x = 0.0;
        mk.pose.orientation.y = 0.0;
        mk.pose.orientation.z = 0.0;

        update_range_pub_.publish(mk);
    }

    void MapROS::publishESDF()
    {
        double dist;
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::PointXYZI pt;

        const double min_dist = 0.0;
        const double max_dist = 3.0;

        Eigen::Vector3i min_cut = map_->md_->local_bound_min_ - Eigen::Vector3i(map_->mp_->local_map_margin_,
                                                                                map_->mp_->local_map_margin_,
                                                                                map_->mp_->local_map_margin_);
        Eigen::Vector3i max_cut = map_->md_->local_bound_max_ + Eigen::Vector3i(map_->mp_->local_map_margin_,
                                                                                map_->mp_->local_map_margin_,
                                                                                map_->mp_->local_map_margin_);
        map_->boundIndex(min_cut);
        map_->boundIndex(max_cut);

        for (int x = min_cut(0); x <= max_cut(0); ++x)
            for (int y = min_cut(1); y <= max_cut(1); ++y)
            {
                Eigen::Vector3d pos;
                map_->indexToPos(Eigen::Vector3i(x, y, 1), pos);
                pos(2) = esdf_slice_height_;
                dist = map_->getDistance(pos);
                dist = min(dist, max_dist);
                dist = max(dist, min_dist);
                pt.x = pos(0);
                pt.y = pos(1);
                pt.z = -0.2;
                pt.intensity = (dist - min_dist) / (max_dist - min_dist);
                cloud.push_back(pt);
            }

        cloud.width = cloud.points.size();
        cloud.height = 1;
        cloud.is_dense = true;
        cloud.header.frame_id = frame_id_;
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud, cloud_msg);

        esdf_pub_.publish(cloud_msg);

    }

    pcl::PointCloud<pcl::PointXYZ> MapROS::generateAirPointCloud(
        const pcl::PointCloud<pcl::PointXYZ> &input_cloud,
        const Eigen::Matrix3d &R_sensor2world,
        const Eigen::Vector3d &t_sensor2world)
    {
        pcl::PointCloud<pcl::PointXYZ> air_inf_cloud;

        const double horizontal_fov = this->h_fov_;
        const double vertical_fov_min = this->v_fov_min_ + this->inf_fov_;
        const double vertical_fov_max = this->v_fov_max_ - this->inf_fov_;
        const double horizontal_resolution = this->h_res_;
        const double vertical_resolution = this->v_res_;
        const double max_depth = this->mapping_visible_dist_ + 0.5;
        const double min_depth = 0.5;

        int cols = static_cast<int>(horizontal_fov / horizontal_resolution);
        int rows = static_cast<int>((vertical_fov_max - vertical_fov_min) / vertical_resolution);

        Eigen::MatrixXd depth_map = Eigen::MatrixXd::Constant(rows, cols, max_depth);
        double horizontal_fov_min = -horizontal_fov / 2.0;

        for (const auto &pt : input_cloud.points)
        {
            Eigen::Vector3d pt_world(pt.x, pt.y, pt.z);
            Eigen::Vector3d pt_sensor = R_sensor2world.transpose() * (pt_world - t_sensor2world);

            double horizontal_angle = std::atan2(pt_sensor.y(), pt_sensor.x()) * 180.0 / M_PI;
            double vertical_angle = std::atan2(pt_sensor.z(), std::sqrt(pt_sensor.x() * pt_sensor.x() + pt_sensor.y() * pt_sensor.y())) * 180.0 / M_PI;

            if (horizontal_angle < horizontal_fov_min || horizontal_angle > -horizontal_fov_min ||
                vertical_angle < vertical_fov_min || vertical_angle > vertical_fov_max)
                continue;

            int u = static_cast<int>((horizontal_angle - horizontal_fov_min) / horizontal_resolution);
            int v = static_cast<int>((vertical_angle - vertical_fov_min) / vertical_resolution);

            if (u >= 0 && u < cols && v >= 0 && v < rows)
            {
                double depth = pt_sensor.norm();
                depth_map(v, u) = std::min(depth_map(v, u), depth);
            }
        }

        int dilation_radius = this->dil_pixels_;
        Eigen::MatrixXd depth_map_dilated = depth_map;

        for (int v = 0; v < rows; v++)
        {
            for (int u = 0; u < cols; u++)
            {
                if (depth_map(v, u) < max_depth && depth_map(v, u) >= min_depth)
                {
                    for (int dv = -dilation_radius; dv <= dilation_radius; dv++)
                    {
                        for (int du = -dilation_radius; du <= dilation_radius; du++)
                        {
                            int u_neighbor = u + du;
                            int v_neighbor = v + dv;

                            if (u_neighbor >= 0 && u_neighbor < cols && v_neighbor >= 0 && v_neighbor < rows)
                            {
                                if (depth_map(v_neighbor, u_neighbor) < max_depth && depth_map(v_neighbor, u_neighbor) >= min_depth)
                                    continue;
                                depth_map_dilated(v_neighbor, u_neighbor) = 0.9 * min_depth;
                            }
                        }
                    }
                }
            }
        }

        for (int v = 0; v < rows; v++)
        {
            for (int u = 0; u < cols; u++)
            {
                double depth = depth_map_dilated(v, u);

                if (depth < max_depth)
                    continue;

                if (depth >= max_depth)
                    depth = max_depth;

                double horizontal_angle = horizontal_fov_min + u * horizontal_resolution;
                double vertical_angle = vertical_fov_min + v * vertical_resolution;

                Eigen::Vector3d direction;
                direction.x() = std::cos(horizontal_angle * M_PI / 180.0) * std::cos(vertical_angle * M_PI / 180.0);
                direction.y() = std::sin(horizontal_angle * M_PI / 180.0) * std::cos(vertical_angle * M_PI / 180.0);
                direction.z() = std::sin(vertical_angle * M_PI / 180.0);

                Eigen::Vector3d pt_sensor = direction * depth;

                Eigen::Vector3d pt_world = R_sensor2world * pt_sensor + t_sensor2world;

                pcl::PointXYZ pt;
                pt.x = pt_world.x();
                pt.y = pt_world.y();
                pt.z = pt_world.z();
                air_inf_cloud.points.push_back(pt);
            }
        }

        air_inf_cloud.width = air_inf_cloud.points.size();
        air_inf_cloud.height = 1;
        air_inf_cloud.is_dense = true;

        return air_inf_cloud;
    }

    void MapROS::publishValueMap()
    {
        pcl::PointXYZRGBA pt;
        pcl::PointCloud<pcl::PointXYZRGBA> cloud;

        double min_value = std::numeric_limits<double>::max();
        double max_value = std::numeric_limits<double>::lowest();
        bool has_observations = false;

        for (int x = map_->mp_->box_min_(0); x < map_->mp_->box_max_(0); ++x)
            for (int y = map_->mp_->box_min_(1); y < map_->mp_->box_max_(1); ++y)
                for (int z = map_->mp_->box_min_(2); z < map_->mp_->box_max_(2); ++z)
                {
                    double value = map_->md_->valuemap_[map_->toAddress(x, y, z)];
                    double weight = map_->md_->value_weights_[map_->toAddress(x, y, z)];

                    if (weight > 1e-6)
                    {
                        min_value = std::min(min_value, value);
                        max_value = std::max(max_value, value);
                        has_observations = true;
                    }
                }

        if (!has_observations)
        {
            min_value = 0.0;
            max_value = 1.0;
        }

        else if (max_value <= min_value)
        {
            max_value = min_value + 1.0;
        }

        for (int x = map_->mp_->box_min_(0); x < map_->mp_->box_max_(0); ++x)
            for (int y = map_->mp_->box_min_(1); y < map_->mp_->box_max_(1); ++y)
                for (int z = map_->mp_->box_min_(2); z < map_->mp_->box_max_(2); ++z)
                {
                    int adr = map_->toAddress(x, y, z);
                    double value = map_->md_->valuemap_[adr];
                    double weight = map_->md_->value_weights_[adr];

                    if (weight <= 1e-6)
                        continue;

                    Eigen::Vector3d pos;
                    map_->indexToPos(Eigen::Vector3i(x, y, z), pos);

                    if (pos(2) > visualization_truncate_height_)
                        continue;
                    if (pos(2) < visualization_truncate_low_)
                        continue;

                    pt.x = pos(0);
                    pt.y = pos(1);
                    pt.z = pos(2);

                    double normalized_value = (value - min_value) / (max_value - min_value);

                    auto jet_component = [](double value, double center, double width) -> double {
                        double distance = std::abs(value - center);
                        if (distance > width) return 0.0;
                        return std::cos((distance / width) * M_PI / 2.0);
                    };

                    auto clamp = [](double val, double min_val, double max_val) -> double {
                        return std::max(min_val, std::min(max_val, val));
                    };

                    if (value <= 0.0) {
                        pt.r = 0; pt.g = 0; pt.b = 255;
                    } else if (value >= 1.0) {
                        pt.r = 255; pt.g = 0; pt.b = 0;
                    } else {

                        double r = jet_component(value, 0.75, 0.375) + jet_component(value, 1.0, 0.125);
                        double g = jet_component(value, 0.5, 0.375);
                        double b = jet_component(value, 0.25, 0.375) + jet_component(value, 0.0, 0.125);

                        pt.r = static_cast<uint8_t>(clamp(r, 0.0, 1.0) * 255);
                        pt.g = static_cast<uint8_t>(clamp(g, 0.0, 1.0) * 255);
                        pt.b = static_cast<uint8_t>(clamp(b, 0.0, 1.0) * 255);
                    }
                    pt.a = static_cast<uint8_t>(255 * value_map_alpha_);

                    cloud.push_back(pt);
                }

        cloud.width = cloud.points.size();
        cloud.height = 1;
        cloud.is_dense = true;
        cloud.header.frame_id = frame_id_;
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud, cloud_msg);
        value_map_pub_.publish(cloud_msg);

        static int debug_counter = 0;
        if (debug_counter++ % 20 == 0)
        {
            ROS_WARN("ValueMap: %lu points published, min_value=%.3f, max_value=%.3f",
                     cloud.points.size(), min_value, max_value);
        }
    }

    void MapROS::generateAndUpdateValueMapWithAirVoxels(const double &value)
    {
        uint16_t *row_ptr;
        int cols = depth_image_->cols;
        int rows = depth_image_->rows;
        double depth;
        const double inv_factor = 1.0 / k_depth_scaling_factor_;
        const double resolution = map_->mp_->resolution_;

        Eigen::Matrix3d camera_r = camera_q_.toRotationMatrix();

        Eigen::Matrix3d R_trans = Eigen::Matrix3d::Zero();
        R_trans(0, 2) = 1;
        R_trans(1, 0) = -1;
        R_trans(2, 1) = -1;
        camera_r = camera_r * R_trans;

        for (int v = depth_filter_margin_; v < rows - depth_filter_margin_; v += skip_pixel_)
        {
            row_ptr = depth_image_->ptr<uint16_t>(v) + depth_filter_margin_;
            for (int u = depth_filter_margin_; u < cols - depth_filter_margin_; u += skip_pixel_)
            {
                depth = (*row_ptr) * inv_factor;
                row_ptr = row_ptr + skip_pixel_;

                if (*row_ptr == 0 || depth > value_depth_filter_maxdist_)
                    depth = value_depth_filter_maxdist_;
                else if (depth < depth_filter_mindist_)
                    continue;

                Eigen::Vector3d pt_cur;
                pt_cur(0) = (u - cx_) * depth / fx_;
                pt_cur(1) = (v - cy_) * depth / fy_;
                pt_cur(2) = depth;
                Eigen::Vector3d pt_world = camera_r * pt_cur + camera_pos_;

                Eigen::Vector3d ray_direction = pt_world - camera_pos_;
                double ray_length = ray_direction.norm();
                ray_direction.normalize();

                double step_size = resolution * 0.5;
                int num_steps = static_cast<int>(ray_length / step_size);

                for (int step = 0; step <= num_steps; ++step)
                {
                    double distance = step * step_size;
                    if (distance > ray_length) distance = ray_length;

                    Eigen::Vector3d ray_point = camera_pos_ + ray_direction * distance;

                    if (!map_->isInMap(ray_point))
                        continue;

                    Eigen::Vector3i idx;
                    map_->posToIndex(ray_point, idx);
                    int vox_adr = map_->toAddress(idx);

                    double weight = 1.0;

                    map_->setValueWeighted(vox_adr, value, weight);
                }
            }
        }
    }

    void MapROS::processDepthImageGeneric(const sensor_msgs::ImageConstPtr& img,
                                         const geometry_msgs::PoseStampedConstPtr& pose,
                                         pcl::PointCloud<pcl::PointXYZ>& output_cloud) {
        output_cloud.clear();

        Eigen::Vector3d camera_pos(pose->pose.position.x, pose->pose.position.y, pose->pose.position.z);
        Eigen::Quaterniond camera_q(pose->pose.orientation.w, pose->pose.orientation.x,
                                   pose->pose.orientation.y, pose->pose.orientation.z);

        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
        cv::Mat depth_image;
        if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
            (cv_ptr->image).convertTo(depth_image, CV_16UC1, k_depth_scaling_factor_);
        else
            depth_image = cv_ptr->image;

        int cols = depth_image.cols;
        int rows = depth_image.rows;
        double depth;
        Eigen::Matrix3d camera_r = camera_q.toRotationMatrix();

        Eigen::Matrix3d R_trans = Eigen::Matrix3d::Zero();
        R_trans(0,2) = 1;
        R_trans(1,0) = -1;
        R_trans(2,1) = -1;

        camera_r = camera_r * R_trans;
        Eigen::Vector3d pt_cur, pt_world;

        for (int v = depth_filter_margin_; v < rows - depth_filter_margin_; v += skip_pixel_) {
            for (int u = depth_filter_margin_; u < cols - depth_filter_margin_; u += skip_pixel_) {

                if (depth_image.type() == CV_32FC1) {
                    depth = depth_image.at<float>(v, u);

                    if (depth <= 0.0 || depth > depth_filter_maxdist_)
                        depth = depth_filter_maxdist_;
                    else if (depth < depth_filter_mindist_)
                        continue;
                } else {

                    uint16_t depth_raw = depth_image.at<uint16_t>(v, u);
                    depth = depth_raw / k_depth_scaling_factor_;

                    if (depth_raw == 0 || depth > depth_filter_maxdist_)
                        depth = depth_filter_maxdist_;
                    else if (depth < depth_filter_mindist_)
                        continue;
                }

                pt_cur(0) = (u - cx_) * depth / fx_;
                pt_cur(1) = (v - cy_) * depth / fy_;
                pt_cur(2) = depth;

                pt_world = camera_r * pt_cur + camera_pos;

                pcl::PointXYZ pt;
                pt.x = pt_world[0];
                pt.y = pt_world[1];
                pt.z = pt_world[2];
                output_cloud.points.push_back(pt);
            }
        }

        output_cloud.width = output_cloud.points.size();
        output_cloud.height = 1;
        output_cloud.is_dense = true;
    }

    void MapROS::leftDepthPoseCallback(const sensor_msgs::ImageConstPtr& img,
                                      const geometry_msgs::PoseStampedConstPtr& pose) {
        Eigen::Vector3d left_camera_pos(pose->pose.position.x, pose->pose.position.y, pose->pose.position.z);
        if (!map_->isInMap(left_camera_pos))
            return;

        pcl::PointCloud<pcl::PointXYZ> left_cloud;
        processDepthImageGeneric(img, pose, left_cloud);

        if (!left_cloud.empty()) {
            std::lock_guard<std::mutex> lock(point_cloud_mutex_);
            size_t start_idx = merged_point_cloud_.width;
            for (size_t i = 0; i < left_cloud.points.size() && start_idx + i < merged_point_cloud_.points.size(); ++i) {
                merged_point_cloud_.points[start_idx + i] = left_cloud.points[i];
            }
            merged_point_cloud_.width = std::min(start_idx + left_cloud.points.size(), merged_point_cloud_.points.size());

            map_->inputPointCloud(left_cloud, left_cloud.points.size(), left_camera_pos);
            if (local_updated_) {
                map_->clearAndInflateLocalMap();
                esdf_need_update_ = true;
                local_updated_ = false;
            }
        }
    }

    void MapROS::rightDepthPoseCallback(const sensor_msgs::ImageConstPtr& img,
                                       const geometry_msgs::PoseStampedConstPtr& pose) {
        Eigen::Vector3d right_camera_pos(pose->pose.position.x, pose->pose.position.y, pose->pose.position.z);
        if (!map_->isInMap(right_camera_pos))
            return;

        pcl::PointCloud<pcl::PointXYZ> right_cloud;
        processDepthImageGeneric(img, pose, right_cloud);

        if (!right_cloud.empty()) {
            std::lock_guard<std::mutex> lock(point_cloud_mutex_);
            size_t start_idx = merged_point_cloud_.width;
            for (size_t i = 0; i < right_cloud.points.size() && start_idx + i < merged_point_cloud_.points.size(); ++i) {
                merged_point_cloud_.points[start_idx + i] = right_cloud.points[i];
            }
            merged_point_cloud_.width = std::min(start_idx + right_cloud.points.size(), merged_point_cloud_.points.size());

            map_->inputPointCloud(right_cloud, right_cloud.points.size(), right_camera_pos);
            if (local_updated_) {
                map_->clearAndInflateLocalMap();
                esdf_need_update_ = true;
                local_updated_ = false;
            }
        }
    }
}
