#include "sdf_map.h"

void SDFMap::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos)
{
  min_pos(0) = max(min_pos(0), min_range(0));
  min_pos(1) = max(min_pos(1), min_range(1));
  min_pos(2) = max(min_pos(2), min_range(2));

  max_pos(0) = min(max_pos(0), max_range(0));
  max_pos(1) = min(max_pos(1), max_range(1));
  max_pos(2) = min(max_pos(2), max_range(2));

  Eigen::Vector3i min_id, max_id;

  posToIndex(min_pos, min_id);
  posToIndex(max_pos - Eigen::Vector3d(resolution / 2, resolution / 2, resolution / 2), max_id);

  /* reset occ and dist buffer */
  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z)
      {
        occupancy_buffer[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z] = 0.0;
        distance_buffer[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z] = 10000;
      }
}

bool SDFMap::isInMap(Eigen::Vector3d pos)
{
  if (pos(0) < min_range(0) || pos(1) < min_range(1) || pos(2) < min_range(2))
  {
    // std::cout << "pos(0) =  " << pos(0) << "  min_range(0) =" <<  min_range(0) 
    // << " pos(1) =" << pos(1) << "  min_range(1)=" <<  min_range(1) << 
    // " pos(2) =" << pos(2) << "  min_range(2) =" <<  min_range(2) << std::endl;
    cout << "less than min range!" << endl;



    return false;
  }

  if (pos(0) > max_range(0) || pos(1) > max_range(1)  || pos(2) > max_range(2))
  {
    cout << "larger than max range!" << endl;
    return false;
  }

  return true;
}

void SDFMap::posToIndex(Eigen::Vector3d pos, Eigen::Vector3i& id)
{
  for (int i = 0; i < 3; ++i)
    id(i) = floor((pos(i) - origin(i)) * resolution_inv);
}

void SDFMap::indexToPos(Eigen::Vector3i id, Eigen::Vector3d& pos)
{
  for (int i = 0; i < 3; ++i)
    pos(i) = (id(i) + 0.5) * resolution + origin(i);
}

void SDFMap::setOccupancy(Eigen::Vector3d pos, double occ)
{
  if (occ != 1 && occ != 0)
  {
    cout << "occ value error!" << endl;
    return;
  }

  if (!isInMap(pos))
    return;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  occupancy_buffer[id(0) * grid_size(1) * grid_size(2) + id(1) * grid_size(2) + id(2)] = occ;
}

int SDFMap::getOccupancy(Eigen::Vector3d pos)
{
  if (!isInMap(pos))
    return -1;

  Eigen::Vector3i id;
  posToIndex(pos, id);
  // std::cout << "getOccupancy id =" << id << std::endl;

  // std::cout << "value = " << occupancy_buffer[id(0) * grid_size(1) * grid_size(2) + id(1) * grid_size(2) + id(2)] << std::endl;
 
  return occupancy_buffer[id(0) * grid_size(1) * grid_size(2) + id(1) * grid_size(2) + id(2)] > 0.1 ? 1 : 0;
                                                                                                         
}

int SDFMap::getInflateOccupancy(Eigen::Vector3d pos)
{
  if (!isInMap(pos))
    return -1;

  Eigen::Vector3i id;
  posToIndex(pos, id);
  // std::cout << "getInflateOccupancy =" << id(0) * grid_size(1) * grid_size(2) + id(1) * grid_size(2) + id(2) << std::endl;

  return occupancy_buffer_inflate_[id(0) * grid_size(1) * grid_size(2) + id(1) * grid_size(2) + id(2)] >
              0 ? //min_occupancy_log_ ?
             1 :
             0;
}

int SDFMap::getOccupancy(Eigen::Vector3i id)
{
  if (id(0) < 0 || id(0) >= grid_size(0) || id(1) < 0 || id(1) >= grid_size(1) || id(2) < 0 || id(2) >= grid_size(2))
    return -1;

  return occupancy_buffer[id(0) * grid_size(1) * grid_size(2) + id(1) * grid_size(2) + id(2)] > 0// min_occupancy_log_
                                                                                                                   ? 1 :
                                                                                                                     0;
}

double SDFMap::getDistance(Eigen::Vector3d pos)
{
  if (!isInMap(pos))
    return -1;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  // (x, y, z) -> x*ny*nz + y*nz + z
  return distance_buffer_all[id(0) * grid_size(1) * grid_size(2) + id(1) * grid_size(2) + id(2)];
}

double SDFMap::getDistance(Eigen::Vector3i id, int sign)
{
  id(0) = max(min(id(0), grid_size(0) - 1), 0);
  id(1) = max(min(id(1), grid_size(1) - 1), 0);
  id(2) = max(min(id(2), grid_size(2) - 1), 0);

  return distance_buffer_all[id(0) * grid_size(1) * grid_size(2) + id(1) * grid_size(2) + id(2)];
}

double SDFMap::getDistWithGradTrilinear(Eigen::Vector3d pos, Eigen::Vector3d& grad)
{
  if (!isInMap(pos))
  {
    grad.setZero();
    return 0;
  }

  /* use trilinear interpolation */
  Eigen::Vector3d pos_m = pos - 0.5 * resolution * Eigen::Vector3d::Ones();

  Eigen::Vector3i idx;
  posToIndex(pos_m, idx);

  Eigen::Vector3d idx_pos, diff;
  indexToPos(idx, idx_pos);

  diff = (pos - idx_pos) * resolution_inv;

  double values[2][2][2];
  for (int x = 0; x < 2; x++)
  {
    for (int y = 0; y < 2; y++)
    {
      for (int z = 0; z < 2; z++)
      {
        Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
        values[x][y][z] = getDistance(current_idx);
      }
    }
  }

  double v00 = (1 - diff[0]) * values[0][0][0] + diff[0] * values[1][0][0];
  double v01 = (1 - diff[0]) * values[0][0][1] + diff[0] * values[1][0][1];
  double v10 = (1 - diff[0]) * values[0][1][0] + diff[0] * values[1][1][0];
  double v11 = (1 - diff[0]) * values[0][1][1] + diff[0] * values[1][1][1];

  double v0 = (1 - diff[1]) * v00 + diff[1] * v10;
  double v1 = (1 - diff[1]) * v01 + diff[1] * v11;

  double dist = (1 - diff[2]) * v0 + diff[2] * v1;

  grad[2] = (v1 - v0) * resolution_inv;
  grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * resolution_inv;
  grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
  grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
  grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
  grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);

  grad[0] *= resolution_inv;

  return dist;
}

template <typename F_get_val, typename F_set_val>
void SDFMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim)
{
  int v[grid_size(dim)];
  double z[grid_size(dim) + 1];

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
  /* ========== compute positive DT ========== */
  for (int x = esdf_min_[0]; x <= esdf_max_[0]; x++)
  {
    for (int y = esdf_min_[1]; y <= esdf_max_[1]; y++)
    {
      fillESDF(
          [&](int z) {
            return occupancy_buffer_inflate_[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z] >
                       0 //min_occupancy_log_ 
                        ?
                       0 :
                       std::numeric_limits<double>::max();
          },
          [&](int z, double val) { tmp_buffer1[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z] = val; },
          esdf_min_[2], esdf_max_[2], 2);
    }
  }

  for (int x = esdf_min_[0]; x <= esdf_max_[0]; x++)
  {
    for (int z = esdf_min_[2]; z <= esdf_max_[2]; z++)
    {
      fillESDF([&](int y) { return tmp_buffer1[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z]; },
               [&](int y, double val) { tmp_buffer2[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z] = val; },
               esdf_min_[1], esdf_max_[1], 1);
    }
  }

  for (int y = esdf_min_[1]; y <= esdf_max_[1]; y++)
  {
    for (int z = esdf_min_[2]; z <= esdf_max_[2]; z++)
    {
      fillESDF([&](int x) { return tmp_buffer2[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z]; },
               [&](int x, double val) {
                 //  distance_buffer[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z] = resolution *
                 //  std::sqrt(val);
                 distance_buffer[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z] =
                     min(resolution * std::sqrt(val),
                         distance_buffer[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z]);
                  // std::cout << "x =" << x << " y=" << y << "z =" << z << "distance =" <<  distance_buffer[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z] << "es ="
                  // << resolution * std::sqrt(val) << std::endl; 
               },
               esdf_min_[0], esdf_max_[0], 0);
    }
  }

  /* ========== compute negative distance ========== */
  for (int x = esdf_min_(0); x <= esdf_max_(0); ++x)
    for (int y = esdf_min_(1); y <= esdf_max_(1); ++y)
      for (int z = esdf_min_(2); z <= esdf_max_(2); ++z)
      {
        int idx = x * grid_size(1) * grid_size(2) + y * grid_size(2) + z;
        if (occupancy_buffer_inflate_[idx] ==0) //no obstacle //min_occupancy_log_)
        {
          occupancy_buffer_neg[idx] = 0.1;//clamp_max_log_;
        }
        else if (occupancy_buffer_inflate_[idx] == 1) // exists obstacle // min_occupancy_log_)
        {
          occupancy_buffer_neg[idx] = -0.1;///clamp_min_log_;
        }
        else
        {
          std::cout << "what" << std::endl;
        }
      }

  tmp_buffer1.clear();
  tmp_buffer2.clear();

  //ros::Time t1, t2;

  for (int x = esdf_min_[0]; x <= esdf_max_[0]; x++)
  {
    for (int y = esdf_min_[1]; y <= esdf_max_[1]; y++)
    {
      fillESDF(
          [&](int z) {
            return occupancy_buffer_neg[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z] > 0 ?//min_occupancy_log_ ?
                       0 :
                       std::numeric_limits<double>::max();
          },
          [&](int z, double val) { tmp_buffer1[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z] = val; },
          esdf_min_[2], esdf_max_[2], 2);
    }
  }

  for (int x = esdf_min_[0]; x <= esdf_max_[0]; x++)
  {
    for (int z = esdf_min_[2]; z <= esdf_max_[2]; z++)
    {
      fillESDF([&](int y) { return tmp_buffer1[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z]; },
               [&](int y, double val) { tmp_buffer2[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z] = val; },
               esdf_min_[1], esdf_max_[1], 1);
    }
  }

  for (int y = esdf_min_[1]; y <= esdf_max_[1]; y++)
  {
    for (int z = esdf_min_[2]; z <= esdf_max_[2]; z++)
    {
      fillESDF([&](int x) { return tmp_buffer2[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z]; },
               [&](int x, double val) {
                 distance_buffer_neg[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z] =
                     resolution * std::sqrt(val);
               },
               esdf_min_[0], esdf_max_[0], 0);
    }
  }

  /* ========== combine pos and neg DT ========== */
  for (int x = esdf_min_(0); x <= esdf_max_(0); ++x)
    for (int y = esdf_min_(1); y <= esdf_max_(1); ++y)
      for (int z = esdf_min_(2); z <= esdf_max_(2); ++z)
      {
        int idx = x * grid_size(1) * grid_size(2) + y * grid_size(2) + z;

        if (distance_buffer_neg[idx] > 0.0)
          distance_buffer_all[idx] = distance_buffer[idx] - distance_buffer_neg[idx] + resolution;
        else
          distance_buffer_all[idx] = distance_buffer[idx];
      }
}

bool SDFMap::tryFillMinima(const Eigen::Vector3d& pt, Eigen::Vector3d& center, Eigen::Vector3d& cube_len)
{
  vector<Eigen::Vector3d> peaks = findPeaks(pt);
  bool fill = fillLocalMinima(peaks, center, cube_len);

  if (fill)
  {
    // setLocalRange(camera_pos_ - sensor_range_, camera_pos_ + sensor_range_);
    updateESDF3d();
  }

  return fill;
}

bool SDFMap::fillLocalMinima(const vector<Eigen::Vector3d>& peaks, Eigen::Vector3d& center, Eigen::Vector3d& cube_len)
{
  if (peaks.size() == 0)
    return false;

  /* ---------- find bounding box ---------- */
  Eigen::Vector3d box_min, box_max, pk;
  box_min(0) = box_min(1) = box_min(2) = 10000;
  box_max(0) = box_max(1) = box_max(2) = -10000;

  for (int i = 0; i < (int)peaks.size(); ++i)
  {
    pk = peaks[i];
    if (pk(0) > box_max(0))
      box_max(0) = pk(0);
    if (pk(1) > box_max(1))
      box_max(1) = pk(1);
    if (pk(2) > box_max(2))
      box_max(2) = pk(2);

    if (pk(0) < box_min(0))
      box_min(0) = pk(0);
    if (pk(1) < box_min(1))
      box_min(1) = pk(1);
    if (pk(2) < box_min(2))
      box_min(2) = pk(2);
  }
  // cout << "[sdf]: min: " << box_min.transpose() << endl;
  // cout << "[sdf]: max: " << box_max.transpose() << endl;

  /* ---------- expand slice ---------- */
  Eigen::Vector3d mid_pt = 0.5 * (box_min + box_max);
  if ((mid_pt - last_fill_pt).norm() < 0.55)
    return false;

  Eigen::Vector3d cube_scale = box_max - box_min;
  int max_idx = -1;
  double max_scale = -10000;
  for (int i = 0; i < 3; ++i)
  {
    if (cube_scale(i) > max_scale)
    {
      max_scale = cube_scale(i);
      max_idx = i;
    }
  }
  double temp_scale = max(cube_scale((max_idx + 1) % 3), cube_scale((max_idx + 2) % 3));
  cube_scale((max_idx + 1) % 3) = temp_scale;
  cube_scale((max_idx + 2) % 3) = temp_scale;

  const double shrink = 0.5 * resolution;
  cube_scale(0) = max(2 * resolution, cube_scale(0) - shrink);
  cube_scale(1) = max(2 * resolution, cube_scale(1) - shrink);
  cube_scale(2) = max(2 * resolution, cube_scale(2) - shrink);
  // box_max += infla * Eigen::Vector3d::Ones();
  // box_min -= infla * Eigen::Vector3d::Ones();

  /* ---------- fill bounding box ---------- */
  for (double x = mid_pt(0) - cube_scale(0) / 2.0; x <= mid_pt(0) + cube_scale(0) / 2.0; x += resolution)
    for (double y = mid_pt(1) - cube_scale(1) / 2.0; y <= mid_pt(1) + cube_scale(1) / 2.0; y += resolution)
      for (double z = mid_pt(2) - cube_scale(2) / 2.0; z <= mid_pt(2) + cube_scale(2) / 2.0; z += resolution)
      {
        setOccupancy(Eigen::Vector3d(x + 0.01, y + 0.01, z + 0.01));
      }

  /* ---------- set last fill pt ---------- */
  last_fill_pt = mid_pt;
  center = mid_pt;
  cube_len = cube_scale;

  return true;
}

vector<Eigen::Vector3d> SDFMap::findPeaks(const Eigen::Vector3d& pt)
{
  vector<Eigen::Vector3d> peaks;
  /* ---------- iterate surrounding to find peak ---------- */
  Eigen::Vector3i idx;
  posToIndex(pt, idx);

  const int radius = ceil(0.3 / resolution);
  const int radius_z = radius;

  Eigen::Vector3d coord;
  bool peak;
  for (int x = idx(0) - radius; x <= idx(0) + radius; ++x)
    for (int y = idx(1) - radius; y <= idx(1) + radius; ++y)
      for (int z = idx(2) - radius_z; z <= idx(2) + radius_z; ++z)
      {
        indexToPos(Eigen::Vector3i(x, y, z), coord);
        peak = checkPeak(coord);
        if (peak)
        {
          peaks.push_back(coord);
        }
      }
  return peaks;
}

bool SDFMap::checkPeak(Eigen::Vector3d pos)
{
  Eigen::Vector3d grad1, grad2, pos1, pos2;
  double grad_2nd;
  const double thresh = 0.5 * resolution_inv;
  /* ---------- check x axis ---------- */
  pos1 = pos2 = pos;
  pos1(0) = pos(0) - resolution;
  pos2(0) = pos(0) + resolution;
  getDistWithGradTrilinear(pos1, grad1);
  getDistWithGradTrilinear(pos2, grad2);

  grad_2nd = (grad2(0) - grad1(0)) / (2.0 * resolution);
  if (grad_2nd > thresh)
  {
    return true;
  }

  /* ---------- check y axis ---------- */
  pos1 = pos2 = pos;
  pos1(1) = pos(1) - resolution;
  pos2(1) = pos(1) + resolution;
  getDistWithGradTrilinear(pos1, grad1);
  getDistWithGradTrilinear(pos2, grad2);

  grad_2nd = (grad2(1) - grad1(1)) / (2.0 * resolution);
  if (grad_2nd > thresh)
  {
    return true;
  }

  /* ---------- check z axis ---------- */
  pos1 = pos2 = pos;
  pos1(2) = pos(2) - resolution;
  pos2(2) = pos(2) + resolution;
  getDistWithGradTrilinear(pos1, grad1);
  getDistWithGradTrilinear(pos2, grad2);

  grad_2nd = (grad2(2) - grad1(2)) / (2.0 * resolution);
  if (grad_2nd > thresh)
  {
    return true;
  }

  /* ---------- no peak ---------- */
  return false;
}

void SDFMap::initMap(double x_size,double y_size,double z_size,double resolution_, Eigen::Vector3d org,double inflate_values)
{

  /* ---------- get parameter ---------- */

  inflate_val_ = inflate_values;//0.15;
  
  ground_z_ = 1.0;
  resolution = resolution_;

  resolution_inv = 1 / resolution;

  origin   = org;//Eigen::Vector3d(-y_size / 2.0, -y_size / 2.0, ground_z_);
  map_size = Eigen::Vector3d(x_size, y_size, z_size);

  
  /* ---------- init map ---------- */

  for (int i = 0; i < 3; ++i)
    grid_size(i) = ceil(map_size(i) / resolution);
  //cout << "grid num:" << grid_size.transpose() << endl;
  min_range = origin;
  max_range = origin + map_size;
  
  last_fill_pt.setZero();

  // initialize size of buffer
  occupancy_buffer.resize(grid_size(0) * grid_size(1) * grid_size(2));
  distance_buffer.resize(grid_size(0) * grid_size(1) * grid_size(2));

  occupancy_buffer_inflate_.resize(grid_size(0) * grid_size(1) * grid_size(2));

  occupancy_buffer_neg.resize(grid_size(0) * grid_size(1) * grid_size(2));
  distance_buffer_neg.resize(grid_size(0) * grid_size(1) * grid_size(2));

  distance_buffer_all.resize(grid_size(0) * grid_size(1) * grid_size(2));

  tmp_buffer1.resize(grid_size(0) * grid_size(1) * grid_size(2));
  tmp_buffer2.resize(grid_size(0) * grid_size(1) * grid_size(2));

  cache_all_.resize(grid_size(0) * grid_size(1) * grid_size(2));
  cache_hit_.resize(grid_size(0) * grid_size(1) * grid_size(2));

  
  fill(occupancy_buffer.begin(), occupancy_buffer.end(), -1);
  fill(occupancy_buffer_neg.begin(), occupancy_buffer_neg.end(), -1);
  fill(occupancy_buffer_inflate_.begin(), occupancy_buffer_inflate_.end(), -1);

  fill(distance_buffer.begin(), distance_buffer.end(), 10000);
  fill(distance_buffer_neg.begin(), distance_buffer_neg.end(), 10000);
  fill(distance_buffer_all.begin(), distance_buffer_all.end(), 10000);

  fill(cache_all_.begin(), cache_all_.end(), 0);
  fill(cache_hit_.begin(), cache_hit_.end(), 0);

  std::cout << "ESDF 地图初始化完成!" << std::endl;

}

int SDFMap::setCacheOccupancy(Eigen::Vector3d pos, int occ)
{
  if (occ != 1 && occ != 0)
  {
    return INVALID_IDX;
  }

  if (!isInMap(pos))
  {
    //std::cout << "setCacheOccupancy11!isInMap" << std::endl;
    return INVALID_IDX;
    
  }
  //std::cout << "setCacheOccupancy11" << std::endl;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  int idx_ctns = id(0) * grid_size(1) * grid_size(2) + id(1) * grid_size(2) + id(2);

  cache_all_[idx_ctns] += 1;

  if (cache_all_[idx_ctns] == 1)
  {
    // cache_voxel_.push(idx_ctns);
    // std::cout << "setCacheOccupancy22" << std::endl;
    cache_voxel_.push(id);
  }

  if (occ == 1)
    cache_hit_[idx_ctns] = 1;
  else 
    cache_hit_[idx_ctns] = 0;


  return idx_ctns;
}

void SDFMap::raycastProcess()
{
  
  /* ---------- for updating esdf ---------- */
  double max_x, max_y, max_z, min_x, min_y, min_z;

  min_x = max_range(0);
  min_y = max_range(1);
  min_z = max_range(2);

  max_x = min_range(0);
  max_y = min_range(1);
  max_z = min_range(2);

  //std::cout << "proj point cnt: " << proj_points_cnt << std::endl;

  esdf_max_ = (grid_size - Eigen::Vector3i::Ones()).array();
  esdf_min_ = Eigen::Vector3i::Zero().array();


  // std::cout << "cache_voxel_ size =" << cache_voxel_.size() << std::endl;
   std::cout << "esdf_min_ =" << esdf_min_ << " esdf_max_ =" << esdf_max_ << std::endl;

  /* ---------- update occupancy in batch ---------- */
  while (!cache_voxel_.empty())
  {
    Eigen::Vector3i idx = cache_voxel_.front();
    int idx_ctns = idx(0) * grid_size(1) * grid_size(2) + idx(1) * grid_size(2) + idx(2);
    cache_voxel_.pop();
    if( cache_hit_[idx_ctns] > 0)
    {
      occupancy_buffer[idx_ctns] = 1;
      std::cout << "idx(0) =" << idx(0) << "  idx(1) =" <<  idx(1) << " idx(2) =" << idx(2) << std::endl;
    }
    else 
    {
      occupancy_buffer[idx_ctns] = 0;
    }
     
  }
}

void SDFMap::inflatePoint(const Eigen::Vector3i& pt, int step, vector<Eigen::Vector3i>& pts)
{
  int num = 0;

  // for (int x = -step; x <= step; ++x)
  // {
  //   if (x == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1), pt(2));
  // }
  // for (int y = -step; y <= step; ++y)
  // {
  //   if (y == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1) + y, pt(2));
  // }

  // for (int z = -1; z <= 1; ++z)
  // {
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1), pt(2) + z);
  // }

   for (int x = -step; x <= step; ++x)
    for (int y = -step; y <= step; ++y)
      for (int z = -step; z <= step; ++z) {
        pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2) + z);
      }
}

void SDFMap::clearAndInflateLocalMap()
{
  /*clear outside local*/
  Eigen::Vector3i min_cut = esdf_min_;
  Eigen::Vector3i max_cut = esdf_max_;

  max_cut = max_cut.array().min((grid_size - Eigen::Vector3i::Ones()).array());
  max_cut = max_cut.array().max(Eigen::Vector3i::Zero().array());

  min_cut = min_cut.array().min((grid_size - Eigen::Vector3i::Ones()).array());
  min_cut = min_cut.array().max(Eigen::Vector3i::Zero().array());

  Eigen::Vector3i min_cut_m = min_cut;
  Eigen::Vector3i max_cut_m = max_cut;

  max_cut_m = max_cut_m.array().min((grid_size - Eigen::Vector3i::Ones()).array());
  max_cut_m = max_cut_m.array().max(Eigen::Vector3i::Zero().array());

  min_cut_m = min_cut_m.array().min((grid_size - Eigen::Vector3i::Ones()).array());
  min_cut_m = min_cut_m.array().max(Eigen::Vector3i::Zero().array());

  // std::cout << "min_cut_m =" << min_cut_m << std::endl;
  // std::cout << "max_cut_m =" << max_cut_m << std::endl;

  int inf_step = ceil(inflate_val_ / resolution);

  //  int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
  // // int inf_step_z = 1;
  // vector<Eigen::Vector3i> inf_pts(pow(2 * inf_step + 1, 3));

  /* ---------- inflate map ---------- */
  // vector<Eigen::Vector3i> inf_pts;
  // inf_pts.resize(4 * inf_step + 3);
  vector<Eigen::Vector3i> inf_pts(pow(2 * inf_step + 1, 3));
  //inf_pts.resize(6 * inf_step + 3);
  Eigen::Vector3i inf_pt;

  for (int x = esdf_min_(0); x <= esdf_max_(0); ++x)
    for (int y = esdf_min_(1); y <= esdf_max_(1); ++y)
      for (int z = esdf_min_(2); z <= esdf_max_(2); ++z)
      {
        occupancy_buffer_inflate_[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z] = 0;//clamp_min_log_;
      }

  for (int x = esdf_min_(0); x <= esdf_max_(0); ++x)
    for (int y = esdf_min_(1); y <= esdf_max_(1); ++y)
      for (int z = esdf_min_(2); z <= esdf_max_(2); ++z)
      {
        /* inflate the update map */
        if (occupancy_buffer[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z] > 0)
        {
          std::cout << "x y z = " << x << " " << y << " " << z << std::endl;
          // std::cout << "occupancy_buffer inflate inf_step =" << inf_step << std::endl;
           #if 1//doghome for ground robot
           for (int z1 = 0; z1 <= 50; ++z1){
              //inflatePoint(Eigen::Vector3i(x, y, z1*mp_.resolution_*2), inf_step, inf_pts);
              inflatePoint(Eigen::Vector3i(x, y, z1), inf_step, inf_pts);
              //std::cout << 
              for (int k = 0; k < (int)inf_pts.size(); ++k)
              {
              
                  inf_pt = inf_pts[k];

                  int idx_inf = inf_pt[0] * grid_size(1) * grid_size(2) + inf_pt[1] * grid_size(2) + inf_pt[2];
                  
                   
                  if (idx_inf < 0 || idx_inf > grid_size(0) * grid_size(1) * grid_size(2))
                  {
                      std::cout << "occupancy_buffer inflate continue k =" << k << "idx_inf =" << idx_inf
                    << " x =" <<inf_pt[0] << "y =" << inf_pt[1] << "z =" << inf_pt[2]  << std::endl;

                  
                    continue;
                  }
                  std::cout << "occupancy_buffer inflate continue k =" << k << "idx_inf =" << idx_inf
                    << " x =" <<inf_pt[0] << "y =" << inf_pt[1] << "z =" << inf_pt[2]  << std::endl;

                  occupancy_buffer_inflate_[idx_inf] = 1;//clamp_max_log_;
              }
            }
         #else

          inflatePoint(Eigen::Vector3i(x, y, z), inf_step, inf_pts);

          for (int k = 0; k < (int)inf_pts.size(); ++k)
          {
           
            inf_pt = inf_pts[k];

            int idx_inf = inf_pt[0] * grid_size(1) * grid_size(2) + inf_pt[1] * grid_size(2) + inf_pt[2];

            if (idx_inf < 0 || idx_inf > grid_size(0) * grid_size(1) * grid_size(2))
            {
             
              continue;
            }
            // std::cout << "occupancy_buffer inflate continue k =" << k << "idx_inf =" << idx_inf
            // << " x =" <<inf_pt[0] << "y =" << inf_pt[1] << "z =" << inf_pt[2]  << std::endl;

            occupancy_buffer_inflate_[idx_inf] = 1;//clamp_max_log_;
          }
          #endif
        }
      }
}

void SDFMap::updateOccupancyCallback()
{
  raycastProcess();
  clearAndInflateLocalMap();
}

void SDFMap::updateESDFCallback()
{
  updateESDF3d();
}
void SDFMap::getSliceESDF(const double height, const double res, Eigen::Vector4d range, vector<Eigen::Vector3d>& slice,
                          vector<Eigen::Vector3d>& grad, int sign)
{
  double dist;
  Eigen::Vector3d gd;
  for (double x = range(0); x <= range(1); x += res)
    for (double y = range(2); y <= range(3); y += res)
    {
      dist = this->getDistWithGradTrilinear(Eigen::Vector3d(x, y, height), gd);
      slice.push_back(Eigen::Vector3d(x, y, dist));
      grad.push_back(gd);
    }
}

void SDFMap::checkDist()
{
  for (int x = 0; x < grid_size(0); ++x)
    for (int y = 0; y < grid_size(1); ++y)
      for (int z = 0; z < grid_size(2); ++z)
      {
        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);

        Eigen::Vector3d grad;
        double dist = getDistWithGradTrilinear(pos, grad);

      }
}

void SDFMap::addLaserPoints(Eigen::Vector3d pos, int occ)
{
    setCacheOccupancy(pos, occ);
}

void SDFMap::startUpdateMapInfo()
{
    updateOccupancyCallback();
    std::cout << "updateOccupancyCallback ok" << std::endl;
    updateESDFCallback();
    std::cout << "updateESDFCallback ok" << std::endl;
}
