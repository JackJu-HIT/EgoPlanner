#ifndef _SDF_MAP_H

#define _SDF_MAP_H

#include <Eigen/Eigen>
#include <iostream>
#include <tuple>
#include <queue>

using namespace std;

class SDFMap
{
private:
  // data are saved in vector
  std::vector<double> occupancy_buffer, occupancy_buffer_neg, distance_buffer, distance_buffer_neg, distance_buffer_all,
      tmp_buffer1, tmp_buffer2;

  std::vector<double> occupancy_buffer_inflate_;

  // map property
  Eigen::Vector3d origin, map_size;
  Eigen::Vector3d min_range, max_range;  // map range in pos
  Eigen::Vector3i grid_size;             // map range in index
  double resolution, resolution_inv;
  Eigen::Vector3i min_vec, max_vec;  // the min and max updated range, unit is 1

  Eigen::Vector3d last_fill_pt;

  bool isInMap(Eigen::Vector3d pos);
  void posToIndex(Eigen::Vector3d pos, Eigen::Vector3i& id);
  void indexToPos(Eigen::Vector3i id, Eigen::Vector3d& pos);

  template <typename F_get_val, typename F_set_val>
  void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);

  bool checkPeak(Eigen::Vector3d pos);

public:
  SDFMap()
  {
  }
  ~SDFMap()
  {
  }

  // occupancy management
  void resetBuffer(Eigen::Vector3d min, Eigen::Vector3d max);
  void setOccupancy(Eigen::Vector3d pos, double occ = 1);
  int getOccupancy(Eigen::Vector3d pos);
  int getOccupancy(Eigen::Vector3i id);
  int getInflateOccupancy(Eigen::Vector3d pos);

  double getResolution()
  {
    return resolution;
  }

  // distance field management
  double getDistance(Eigen::Vector3d pos);
  double getDistance(Eigen::Vector3i id, int sign = 1);
  double getDistWithGradTrilinear(Eigen::Vector3d pos, Eigen::Vector3d& grad);
  // /inline void setLocalRange(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos);

  void updateESDF3d();
  void getSliceESDF(const double height, const double res, Eigen::Vector4d range, vector<Eigen::Vector3d>& slice,
                    vector<Eigen::Vector3d>& grad,
                    int sign = 1);  // 1 pos, 2 neg, 3 combined

  /* ---------- try to fill local minima ---------- */
  vector<Eigen::Vector3d> findPeaks(const Eigen::Vector3d& pt);
  bool fillLocalMinima(const vector<Eigen::Vector3d>& peaks, Eigen::Vector3d& center, Eigen::Vector3d& scale);

  bool tryFillMinima(const Eigen::Vector3d& pt, Eigen::Vector3d& center, Eigen::Vector3d& cube_len);

  
  void initMap(double x_size,double y_size,double z_size,double resolution_, Eigen::Vector3d org,double inflate_values);


  void checkDist();

  void addLaserPoints(Eigen::Vector3d pos, int occ);

  void startUpdateMapInfo();


private:

  void updateOccupancyCallback();
  void updateESDFCallback();

  void projectDepthImage();

  void raycastProcess();

  void inflatePoint(const Eigen::Vector3i& pt, int step, vector<Eigen::Vector3i>& pts);
  void clearAndInflateLocalMap();

  int setCacheOccupancy(Eigen::Vector3d pos, int occ);

  enum
  {
    POSE_STAMPED = 1,
    ODOMETRY = 2,
    INVALID_IDX = -10000
  };


  vector<int> cache_hit_, cache_all_;

  double inflate_val_;

  queue<Eigen::Vector3i> cache_voxel_;

  /* ---------- esdf update ---------- */
  Eigen::Vector3i esdf_min_, esdf_max_;
  double ground_z_;
};

#endif