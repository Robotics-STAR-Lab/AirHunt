#include "plan_env/sdf_map.h"
#include "plan_env/map_ros.h"
#include <plan_env/raycast.h>

namespace fast_planner
{
    SDFMap::SDFMap()
    {
    }

    SDFMap::~SDFMap()
    {
    }

    void SDFMap::initMap(ros::NodeHandle &nh)
    {
        mp_.reset(new MapParam);
        md_.reset(new MapData);
        mr_.reset(new MapROS);

        double x_size, y_size, z_size;
        double init_x, init_y, init_z;

        nh.param("sdf_map/resolution", mp_->resolution_, -1.0);
        nh.param("sdf_map/map_size_x", x_size, -1.0);
        nh.param("sdf_map/map_size_y", y_size, -1.0);
        nh.param("sdf_map/map_size_z", z_size, -1.0);
        nh.param("sdf_map/init_x", init_x, 0.0);
        nh.param("sdf_map/init_y", init_y, 0.0);
        nh.param("sdf_map/init_z", init_z, 0.0);
        nh.param("sdf_map/obstacles_inflation", mp_->obstacles_inflation_, -1.0);
        nh.param("sdf_map/local_bound_inflate", mp_->local_bound_inflate_, 1.0);
        nh.param("sdf_map/local_map_margin", mp_->local_map_margin_, 1);
        nh.param("sdf_map/ground_height", mp_->ground_height_, 1.0);
        nh.param("sdf_map/default_dist", mp_->default_dist_, 5.0);
        nh.param("sdf_map/optimistic", mp_->optimistic_, true);
        nh.param("sdf_map/signed_dist", mp_->signed_dist_, false);

        mp_->local_bound_inflate_ = max(mp_->resolution_, mp_->local_bound_inflate_);
        mp_->resolution_inv_ = 1 / mp_->resolution_;
        mp_->map_origin_ = Eigen::Vector3d(init_x - x_size / 2.0, init_y - y_size / 2.0, mp_->ground_height_);
        mp_->map_size_ = Eigen::Vector3d(x_size, y_size, z_size);
        for (int i = 0; i < 3; ++i)
            mp_->map_voxel_num_(i) = ceil(mp_->map_size_(i) / mp_->resolution_);
        mp_->map_min_boundary_ = mp_->map_origin_;
        mp_->map_max_boundary_ = mp_->map_origin_ + mp_->map_size_;

        nh.param("sdf_map/p_hit", mp_->p_hit_, 0.70);
        nh.param("sdf_map/p_miss", mp_->p_miss_, 0.35);
        nh.param("sdf_map/p_min", mp_->p_min_, 0.12);
        nh.param("sdf_map/p_max", mp_->p_max_, 0.97);
        nh.param("sdf_map/p_occ", mp_->p_occ_, 0.80);
        nh.param("sdf_map/max_ray_length", mp_->max_ray_length_, -0.1);
        nh.param("sdf_map/virtual_ceil_height", mp_->virtual_ceil_height_, -0.1);

        auto logit = [](const double &x)
        { return log(x / (1 - x)); };
        mp_->prob_hit_log_ = logit(mp_->p_hit_);
        mp_->prob_miss_log_ = logit(mp_->p_miss_);
        mp_->clamp_min_log_ = logit(mp_->p_min_);
        mp_->clamp_max_log_ = logit(mp_->p_max_);
        mp_->min_occupancy_log_ = logit(mp_->p_occ_);
        mp_->unknown_flag_ = 0.01;
        cout << "hit: " << mp_->prob_hit_log_ << ", miss: " << mp_->prob_miss_log_
             << ", min: " << mp_->clamp_min_log_ << ", max: " << mp_->clamp_max_log_
             << ", thresh: " << mp_->min_occupancy_log_ << endl;

        int buffer_size = mp_->map_voxel_num_(0) * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2);
        md_->occupancy_buffer_ = vector<double>(buffer_size, mp_->clamp_min_log_ - mp_->unknown_flag_);
        md_->occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);
        md_->distance_buffer_neg_ = vector<double>(buffer_size, mp_->default_dist_);
        md_->distance_buffer_ = vector<double>(buffer_size, mp_->default_dist_);
        md_->valuemap_ = vector<double>(buffer_size, 0.0);
        md_->value_weights_ = vector<double>(buffer_size, 0.0);
        md_->value_count_ = vector<int>(buffer_size, 0);
        md_->count_hit_and_miss_ = vector<short>(buffer_size, 0);
        md_->count_hit_ = vector<short>(buffer_size, 0);
        md_->count_miss_ = vector<short>(buffer_size, 0);
        md_->flag_rayend_ = vector<char>(buffer_size, -1);
        md_->flag_visited_ = vector<char>(buffer_size, -1);
        md_->tmp_buffer1_ = vector<double>(buffer_size, 0);
        md_->tmp_buffer2_ = vector<double>(buffer_size, 0);
        md_->raycast_num_ = 0;
        md_->reset_updated_box_ = true;
        md_->update_min_ = md_->update_max_ = Eigen::Vector3d(0, 0, 0);

        vector<string> axis = {"x", "y", "z"};
        for (int i = 0; i < 3; ++i)
        {
            nh.param("sdf_map/box_min_" + axis[i], mp_->box_mind_[i], mp_->map_min_boundary_[i]);
            nh.param("sdf_map/box_max_" + axis[i], mp_->box_maxd_[i], mp_->map_max_boundary_[i]);
        }
        posToIndex(mp_->box_mind_, mp_->box_min_);
        posToIndex(mp_->box_maxd_, mp_->box_max_);

        mr_->setMap(this);
        mr_->node_ = nh;
        mr_->init();

        caster_.reset(new RayCaster);
        caster_->setParams(mp_->resolution_, mp_->map_origin_);
    }

    void SDFMap::resetBuffer()
    {
        resetBuffer(mp_->map_min_boundary_, mp_->map_max_boundary_);
        md_->local_bound_min_ = Eigen::Vector3i::Zero();
        md_->local_bound_max_ = mp_->map_voxel_num_ - Eigen::Vector3i::Ones();
    }

    void SDFMap::resetBuffer(const Eigen::Vector3d &min_pos, const Eigen::Vector3d &max_pos)
    {
        Eigen::Vector3i min_id, max_id;
        posToIndex(min_pos, min_id);
        posToIndex(max_pos, max_id);
        boundIndex(min_id);
        boundIndex(max_id);

        for (int x = min_id(0); x <= max_id(0); ++x)
            for (int y = min_id(1); y <= max_id(1); ++y)
                for (int z = min_id(2); z <= max_id(2); ++z)
                {
                    md_->occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
                    md_->distance_buffer_[toAddress(x, y, z)] = mp_->default_dist_;
                }
    }

    template <typename F_get_val, typename F_set_val>
    void SDFMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim)
    {
        int v[mp_->map_voxel_num_(dim)];
        double z[mp_->map_voxel_num_(dim) + 1];

        int k = start;
        v[start] = start;
        z[start] = -std::numeric_limits<double>::max();
        z[start + 1] = std::numeric_limits<double>::max();

        for (int q = start + 1; q <= end; q++)
        {
            k++;
            double s;

            do
            {
                k--;
                s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
            } while (s <= z[k]);

            k++;

            v[k] = q;
            z[k] = s;
            z[k + 1] = std::numeric_limits<double>::max();
        }

        k = start;

        for (int q = start; q <= end; q++)
        {
            while (z[k + 1] < q)
                k++;
            double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
            f_set_val(q, val);
        }
    }

    void SDFMap::updateESDF3d()
    {
        Eigen::Vector3i min_esdf = md_->local_bound_min_;
        Eigen::Vector3i max_esdf = md_->local_bound_max_;

        if (mp_->optimistic_)
        {
            for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
                for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
                {
                    fillESDF(
                        [&](int z)
                        {
                            return md_->occupancy_buffer_inflate_[toAddress(x, y, z)] == 1 ? 0 : std::numeric_limits<double>::max();
                        },
                        [&](int z, double val)
                        { md_->tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2],
                        max_esdf[2], 2);
                }
        }
        else
        {
            for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
                for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
                {
                    fillESDF(
                        [&](int z)
                        {
                            int adr = toAddress(x, y, z);
                            return (md_->occupancy_buffer_inflate_[adr] == 1 ||
                                    md_->occupancy_buffer_[adr] < mp_->clamp_min_log_ - 1e-3)
                                       ? 0
                                       : std::numeric_limits<double>::max();
                        },
                        [&](int z, double val)
                        { md_->tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2],
                        max_esdf[2], 2);
                }
        }

        for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
            for (int z = min_esdf[2]; z <= max_esdf[2]; z++)
            {
                fillESDF(
                    [&](int y)
                    { return md_->tmp_buffer1_[toAddress(x, y, z)]; },
                    [&](int y, double val)
                    { md_->tmp_buffer2_[toAddress(x, y, z)] = val; }, min_esdf[1],
                    max_esdf[1], 1);
            }
        for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
            for (int z = min_esdf[2]; z <= max_esdf[2]; z++)
            {
                fillESDF(
                    [&](int x)
                    { return md_->tmp_buffer2_[toAddress(x, y, z)]; },
                    [&](int x, double val)
                    {
                        md_->distance_buffer_[toAddress(x, y, z)] = mp_->resolution_ * std::sqrt(val);
                    },
                    min_esdf[0], max_esdf[0], 0);
            }

        if (mp_->signed_dist_)
        {

            for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
                for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
                {
                    fillESDF(
                        [&](int z)
                        {
                            return md_->occupancy_buffer_inflate_
                                               [x * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2) +
                                                y * mp_->map_voxel_num_(2) + z] == 0
                                       ? 0
                                       : std::numeric_limits<double>::max();
                        },
                        [&](int z, double val)
                        { md_->tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2],
                        max_esdf[2], 2);
                }
            for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
                for (int z = min_esdf[2]; z <= max_esdf[2]; z++)
                {
                    fillESDF(
                        [&](int y)
                        { return md_->tmp_buffer1_[toAddress(x, y, z)]; },
                        [&](int y, double val)
                        { md_->tmp_buffer2_[toAddress(x, y, z)] = val; }, min_esdf[1],
                        max_esdf[1], 1);
                }
            for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
                for (int z = min_esdf[2]; z <= max_esdf[2]; z++)
                {
                    fillESDF(
                        [&](int x)
                        { return md_->tmp_buffer2_[toAddress(x, y, z)]; },
                        [&](int x, double val)
                        {
                            md_->distance_buffer_neg_[toAddress(x, y, z)] = mp_->resolution_ * std::sqrt(val);
                        },
                        min_esdf[0], max_esdf[0], 0);
                }

            for (int x = min_esdf(0); x <= max_esdf(0); ++x)
                for (int y = min_esdf(1); y <= max_esdf(1); ++y)
                    for (int z = min_esdf(2); z <= max_esdf(2); ++z)
                    {
                        int idx = toAddress(x, y, z);
                        if (md_->distance_buffer_neg_[idx] > 0.0)
                            md_->distance_buffer_[idx] += (-md_->distance_buffer_neg_[idx] + mp_->resolution_);
                    }
        }
    }

    void SDFMap::setCacheOccupancy(const int &adr, const int &occ)
    {
        if (adr < 0 || static_cast<size_t>(adr) >= md_->count_hit_.size())
        {
            ROS_WARN_THROTTLE(5.0, "Ignoring out-of-range occupancy address: %d", adr);
            return;
        }

        if (md_->count_hit_[adr] == 0 && md_->count_miss_[adr] == 0)
            md_->cache_voxel_.push(adr);

        if (occ == 0)
            md_->count_miss_[adr] = 1;
        else if (occ == 1)
            md_->count_hit_[adr] += 1;

    }

    void SDFMap::inputPointCloud(
        const pcl::PointCloud<pcl::PointXYZ> &points, const int &point_num,
        const Eigen::Vector3d &camera_pos)
    {
        if (point_num == 0 || !camera_pos.allFinite() || !isInMap(camera_pos))
            return;
        md_->raycast_num_ += 1;

        Eigen::Vector3d update_min = camera_pos;
        Eigen::Vector3d update_max = camera_pos;
        if (md_->reset_updated_box_)
        {
            md_->update_min_ = camera_pos;
            md_->update_max_ = camera_pos;
            md_->reset_updated_box_ = false;
        }

        Eigen::Vector3d pt_w, tmp;
        Eigen::Vector3i idx;
        int vox_adr;
        double length;
        for (int i = 0; i < point_num; ++i)
        {
            auto &pt = points.points[i];
            pt_w << pt.x, pt.y, pt.z;
            if (!pt_w.allFinite())
                continue;
            int tmp_flag;

            if (!isInMap(pt_w))
            {

                pt_w = closetPointInMap(pt_w, camera_pos);
                length = (pt_w - camera_pos).norm();
                if (length > mp_->max_ray_length_)
                    pt_w = (pt_w - camera_pos) / length * mp_->max_ray_length_ + camera_pos;
                if (pt_w[2] < 0.2)
                    continue;
                tmp_flag = 0;
            }
            else
            {
                length = (pt_w - camera_pos).norm();
                if (length > mp_->max_ray_length_)
                {
                    pt_w = (pt_w - camera_pos) / length * mp_->max_ray_length_ + camera_pos;
                    if (pt_w[2] < 0.2)
                        continue;
                    tmp_flag = 0;
                }
                else
                    tmp_flag = 1;
            }
            if (!pt_w.allFinite())
                continue;
            posToIndex(pt_w, idx);
            if (!isInMap(idx))
                continue;
            vox_adr = toAddress(idx);
            setCacheOccupancy(vox_adr, tmp_flag);

            for (int k = 0; k < 3; ++k)
            {
                update_min[k] = min(update_min[k], pt_w[k]);
                update_max[k] = max(update_max[k], pt_w[k]);
            }

            if (md_->flag_rayend_[vox_adr] == md_->raycast_num_)
                continue;
            else
                md_->flag_rayend_[vox_adr] = md_->raycast_num_;

            if (!caster_->input(pt_w, camera_pos))
                continue;
            caster_->nextId(idx);
            while (caster_->nextId(idx))
            {
                if (isInMap(idx))
                    setCacheOccupancy(toAddress(idx), 0);
            }
        }

        Eigen::Vector3d bound_inf(mp_->local_bound_inflate_, mp_->local_bound_inflate_, 0);
        posToIndex(update_max + bound_inf, md_->local_bound_max_);
        posToIndex(update_min - bound_inf, md_->local_bound_min_);
        boundIndex(md_->local_bound_min_);
        boundIndex(md_->local_bound_max_);
        mr_->local_updated_ = true;

        for (int k = 0; k < 3; ++k)
        {
            md_->update_min_[k] = min(update_min[k], md_->update_min_[k]);
            md_->update_max_[k] = max(update_max[k], md_->update_max_[k]);
        }

        while (!md_->cache_voxel_.empty())
        {
            int adr = md_->cache_voxel_.front();
            md_->cache_voxel_.pop();
            double log_odds_update =
                md_->count_hit_[adr] >= md_->count_miss_[adr] ? mp_->prob_hit_log_ : mp_->prob_miss_log_;
            md_->count_hit_[adr] = md_->count_miss_[adr] = 0;
            if (md_->occupancy_buffer_[adr] < mp_->clamp_min_log_ - 1e-3)
                md_->occupancy_buffer_[adr] = mp_->min_occupancy_log_;

            md_->occupancy_buffer_[adr] = std::min(
                std::max(md_->occupancy_buffer_[adr] + log_odds_update, mp_->clamp_min_log_),
                mp_->clamp_max_log_);
        }
    }

    Eigen::Vector3d
    SDFMap::closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt)
    {
        Eigen::Vector3d diff = pt - camera_pt;
        Eigen::Vector3d max_tc = mp_->map_max_boundary_ - camera_pt;
        Eigen::Vector3d min_tc = mp_->map_min_boundary_ - camera_pt;
        double min_t = 1000000;
        for (int i = 0; i < 3; ++i)
        {
            if (fabs(diff[i]) > 0)
            {
                double t1 = max_tc[i] / diff[i];
                if (t1 > 0 && t1 < min_t)
                    min_t = t1;
                double t2 = min_tc[i] / diff[i];
                if (t2 > 0 && t2 < min_t)
                    min_t = t2;
            }
        }
        return camera_pt + (min_t - 1e-3) * diff;
    }

    void SDFMap::clearAndInflateLocalMap()
    {

        int inf_step = ceil(mp_->obstacles_inflation_ / mp_->resolution_);
        vector<Eigen::Vector3i> inf_pts(pow(2 * inf_step + 1, 3));

        for (int x = md_->local_bound_min_(0); x <= md_->local_bound_max_(0); ++x)
            for (int y = md_->local_bound_min_(1); y <= md_->local_bound_max_(1); ++y)
                for (int z = md_->local_bound_min_(2); z <= md_->local_bound_max_(2); ++z)
                {
                    md_->occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
                }

        for (int x = md_->local_bound_min_(0); x <= md_->local_bound_max_(0); ++x)
            for (int y = md_->local_bound_min_(1); y <= md_->local_bound_max_(1); ++y)
                for (int z = md_->local_bound_min_(2); z <= md_->local_bound_max_(2); ++z)
                {
                    int id1 = toAddress(x, y, z);
                    if (md_->occupancy_buffer_[id1] > mp_->min_occupancy_log_)
                    {
                        inflatePoint(Eigen::Vector3i(x, y, z), inf_step, inf_pts);

                        for (auto inf_pt : inf_pts)
                        {
                            int idx_inf = toAddress(inf_pt);
                            if (idx_inf >= 0 &&
                                idx_inf <
                                    mp_->map_voxel_num_(0) * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2))
                            {
                                md_->occupancy_buffer_inflate_[idx_inf] = 1;
                            }
                        }
                    }
                }

        if (mp_->virtual_ceil_height_ > -0.5)
        {
            int ceil_id = floor((mp_->virtual_ceil_height_ - mp_->map_origin_(2)) * mp_->resolution_inv_);
            for (int x = md_->local_bound_min_(0); x <= md_->local_bound_max_(0); ++x)
                for (int y = md_->local_bound_min_(1); y <= md_->local_bound_max_(1); ++y)
                {

                    md_->occupancy_buffer_[toAddress(x, y, ceil_id)] = mp_->clamp_max_log_;
                }
        }
    }

    double SDFMap::getResolution()
    {
        return mp_->resolution_;
    }

    int SDFMap::getVoxelNum()
    {
        return mp_->map_voxel_num_[0] * mp_->map_voxel_num_[1] * mp_->map_voxel_num_[2];
    }

    void SDFMap::getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size)
    {
        ori = mp_->map_origin_, size = mp_->map_size_;
    }

    void SDFMap::getBox(Eigen::Vector3d &bmin, Eigen::Vector3d &bmax)
    {
        bmin = mp_->box_mind_;
        bmax = mp_->box_maxd_;
    }

    void SDFMap::getUpdatedBox(Eigen::Vector3d &bmin, Eigen::Vector3d &bmax, bool reset)
    {
        bmin = md_->update_min_;
        bmax = md_->update_max_;
        if (reset)
            md_->reset_updated_box_ = true;
    }

    double SDFMap::getDistWithGrad(const Eigen::Vector3d &pos, Eigen::Vector3d &grad)
    {
        if (!isInMap(pos))
        {
            grad.setZero();
            return 0;
        }

        Eigen::Vector3d pos_m = pos - 0.5 * mp_->resolution_ * Eigen::Vector3d::Ones();
        Eigen::Vector3i idx;
        posToIndex(pos_m, idx);
        Eigen::Vector3d idx_pos, diff;
        indexToPos(idx, idx_pos);
        diff = (pos - idx_pos) * mp_->resolution_inv_;

        double values[2][2][2];
        for (int x = 0; x < 2; x++)
            for (int y = 0; y < 2; y++)
                for (int z = 0; z < 2; z++)
                {
                    Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
                    values[x][y][z] = getDistance(current_idx);
                }

        double v00 = (1 - diff[0]) * values[0][0][0] + diff[0] * values[1][0][0];
        double v01 = (1 - diff[0]) * values[0][0][1] + diff[0] * values[1][0][1];
        double v10 = (1 - diff[0]) * values[0][1][0] + diff[0] * values[1][1][0];
        double v11 = (1 - diff[0]) * values[0][1][1] + diff[0] * values[1][1][1];
        double v0 = (1 - diff[1]) * v00 + diff[1] * v10;
        double v1 = (1 - diff[1]) * v01 + diff[1] * v11;
        double dist = (1 - diff[2]) * v0 + diff[2] * v1;

        grad[2] = (v1 - v0) * mp_->resolution_inv_;
        grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * mp_->resolution_inv_;
        grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
        grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
        grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
        grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);
        grad[0] *= mp_->resolution_inv_;

        return dist;
    }
        void SDFMap::setValue(const int &adr, const double &value)
    {
        md_->valuemap_[adr] = value;
    }

    void SDFMap::setValueWeighted(const int &adr, const double &value, const double &weight)
    {

        double current_weight = md_->value_weights_[adr];
        double current_value = md_->valuemap_[adr];

        double new_weight = current_weight + weight;
        if (new_weight > 1e-6)
        {
            md_->valuemap_[adr] = (current_value * current_weight + value * weight) / new_weight;
            md_->value_weights_[adr] = new_weight;
            md_->value_count_[adr]++;
        }
        else
        {
            md_->valuemap_[adr] = value;
            md_->value_weights_[adr] = weight;
            md_->value_count_[adr] = 1;
        }
    }

        void SDFMap::inputValuePointCloud(const pcl::PointCloud<pcl::PointXYZ> &points,
                                      const int &point_num,
                                      const Eigen::Vector3d &camera_pos,
                                      const double &value)
    {
        if (point_num == 0)
            return;

        auto t1 = ros::Time::now();

        for (int i = 0; i < point_num; ++i)
        {
            Eigen::Vector3d pt_world(points.points[i].x, points.points[i].y, points.points[i].z);

            if (!isInMap(pt_world))
                continue;

            Eigen::Vector3i pt_id;
            posToIndex(pt_world, pt_id);
            int adr = toAddress(pt_id);

            double distance = (pt_world - camera_pos).norm();
            double weight = 1.0 / (1.0 + distance * 0.1);

            setValueWeighted(adr, value, weight);
        }

        auto t2 = ros::Time::now();

    }

    void SDFMap::smoothValueMap(int kernel_size, double sigma)
    {
        if (kernel_size % 2 == 0) kernel_size++;
        int half_kernel = kernel_size / 2;

        vector<vector<vector<double>>> kernel(kernel_size,
            vector<vector<double>>(kernel_size, vector<double>(kernel_size)));
        double sigma_sq_2 = 2.0 * sigma * sigma;
        double weight_sum = 0.0;

        for (int kx = -half_kernel; kx <= half_kernel; kx++)
        {
            for (int ky = -half_kernel; ky <= half_kernel; ky++)
            {
                for (int kz = -half_kernel; kz <= half_kernel; kz++)
                {
                    double dist_sq = kx*kx + ky*ky + kz*kz;
                    double weight = exp(-dist_sq / sigma_sq_2);
                    kernel[kx + half_kernel][ky + half_kernel][kz + half_kernel] = weight;
                    weight_sum += weight;
                }
            }
        }

        for (int kx = 0; kx < kernel_size; kx++)
        {
            for (int ky = 0; ky < kernel_size; ky++)
            {
                for (int kz = 0; kz < kernel_size; kz++)
                {
                    kernel[kx][ky][kz] /= weight_sum;
                }
            }
        }

        vector<double> smoothed_values(md_->valuemap_.size());

        Eigen::Vector3i min_bound = md_->local_bound_min_;
        Eigen::Vector3i max_bound = md_->local_bound_max_;

        for (int i = 0; i < 3; i++)
        {
            min_bound[i] = max(0, min_bound[i] - half_kernel);
            max_bound[i] = min(mp_->map_voxel_num_[i] - 1, max_bound[i] + half_kernel);
        }

        for (int x = min_bound[0]; x <= max_bound[0]; x++)
        {
            for (int y = min_bound[1]; y <= max_bound[1]; y++)
            {
                for (int z = min_bound[2]; z <= max_bound[2]; z++)
                {
                    int center_adr = toAddress(x, y, z);

                    if (md_->value_count_[center_adr] == 0)
                    {
                        smoothed_values[center_adr] = md_->valuemap_[center_adr];
                        continue;
                    }

                    double weighted_sum = 0.0;
                    double total_weight = 0.0;

                    for (int kx = -half_kernel; kx <= half_kernel; kx++)
                    {
                        for (int ky = -half_kernel; ky <= half_kernel; ky++)
                        {
                            for (int kz = -half_kernel; kz <= half_kernel; kz++)
                            {
                                int nx = x + kx;
                                int ny = y + ky;
                                int nz = z + kz;

                                if (nx >= 0 && nx < mp_->map_voxel_num_[0] &&
                                    ny >= 0 && ny < mp_->map_voxel_num_[1] &&
                                    nz >= 0 && nz < mp_->map_voxel_num_[2])
                                {
                                    int neighbor_adr = toAddress(nx, ny, nz);

                                    if (md_->value_count_[neighbor_adr] > 0)
                                    {
                                        double kernel_weight = kernel[kx + half_kernel][ky + half_kernel][kz + half_kernel];
                                        double confidence_weight = md_->value_weights_[neighbor_adr];
                                        double combined_weight = kernel_weight * (1.0 + confidence_weight);

                                        weighted_sum += md_->valuemap_[neighbor_adr] * combined_weight;
                                        total_weight += combined_weight;
                                    }
                                }
                            }
                        }
                    }

                    if (total_weight > 1e-6)
                    {
                        smoothed_values[center_adr] = weighted_sum / total_weight;
                    }
                    else
                    {
                        smoothed_values[center_adr] = md_->valuemap_[center_adr];
                    }
                }
            }
        }

        for (int x = min_bound[0]; x <= max_bound[0]; x++)
        {
            for (int y = min_bound[1]; y <= max_bound[1]; y++)
            {
                for (int z = min_bound[2]; z <= max_bound[2]; z++)
                {
                    int adr = toAddress(x, y, z);
                    if (md_->value_count_[adr] > 0)
                    {
                        md_->valuemap_[adr] = smoothed_values[adr];
                    }
                }
            }
        }
    }
}
