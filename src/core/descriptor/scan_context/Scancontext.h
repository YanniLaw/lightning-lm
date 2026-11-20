// Scan Context: https://github.com/gisbi-kim/scancontext
// See README.md and LICENSE in this directory for attribution and terms.

#ifndef SRC_CORE_DESCRIPTOR_SCAN_CONTEXT_SCANCONTEXT_H_
#define SRC_CORE_DESCRIPTOR_SCAN_CONTEXT_SCANCONTEXT_H_

#include <cstddef>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "KDTreeVectorOfVectorsAdaptor.h"
#include "nanoflann.hpp"

namespace lightning::scan_context {

using SCPointType = pcl::PointXYZI;
using KeyMat = std::vector<std::vector<float>>;
using InvKeyTree = KDTreeVectorOfVectorsAdaptor<KeyMat, float>;

class SCManager {
 public:
  struct Options {
    int pc_num_ring = 20;
    int pc_num_sector = 60;
    double pc_max_radius = 80.0;
    double lidar_height = 0.0;
    int num_candidates_from_tree = 10;
    double search_ratio = 0.1;
    double sc_dist_thres = 0.13;
    int tree_making_period = 1;
  };

  struct Match {
    std::size_t index;
    double distance;
    double yaw_shift_rad;
  };

  explicit SCManager(Options options);

  Eigen::MatrixXd makeScancontext(const pcl::PointCloud<SCPointType>& scan);
  Eigen::MatrixXd makeRingkeyFromScancontext(Eigen::MatrixXd& descriptor);
  Eigen::MatrixXd makeSectorkeyFromScancontext(Eigen::MatrixXd& descriptor);
  int fastAlignUsingVkey(Eigen::MatrixXd& first, Eigen::MatrixXd& second);
  double distDirectSC(Eigen::MatrixXd& first, Eigen::MatrixXd& second);
  std::pair<double, int> distanceBtnScanContext(
      Eigen::MatrixXd& first, Eigen::MatrixXd& second,
      Eigen::MatrixXd& first_sector_key, Eigen::MatrixXd& second_sector_key);

  // Search only the eligible historical prefix, then index the current scan.
  std::optional<Match> DetectAndAdd(const pcl::PointCloud<SCPointType>& scan,
                                    std::size_t reference_count);
  std::size_t GetNumDescriptors() const { return polarcontexts_.size(); }

 private:
  Options options_;
  std::vector<Eigen::MatrixXd> polarcontexts_;
  std::vector<Eigen::MatrixXd> polarcontext_vkeys_;
  KeyMat polarcontext_invkeys_mat_;
  KeyMat polarcontext_invkeys_to_search_;
  std::unique_ptr<InvKeyTree> polarcontext_tree_;
  int queries_since_tree_ = 0;
};

}  // namespace lightning::scan_context

#endif  // SRC_CORE_DESCRIPTOR_SCAN_CONTEXT_SCANCONTEXT_H_
