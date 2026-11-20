// Derived from Scan Context, fixed upstream revision 93672835bb85e9c03fedf0fbaf2d0491df12161e.
#include "core/descriptor/scan_context/Scancontext.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <utility>

namespace lightning::scan_context {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::make_pair;

namespace {

float xy2theta( const float & _x, const float & _y )
{
    float angle = std::atan2(_y, _x) * 180.0 / M_PI;
    return angle < 0.0F ? angle + 360.0F : angle;
} // xy2theta


MatrixXd circshift( MatrixXd &_mat, int _num_shift )
{
    // shift columns to right direction
    assert(_num_shift >= 0);

    if( _num_shift == 0 )
    {
        MatrixXd shifted_mat( _mat );
        return shifted_mat; // Early return
    }

    MatrixXd shifted_mat = MatrixXd::Zero( _mat.rows(), _mat.cols() );
    for ( int col_idx = 0; col_idx < _mat.cols(); col_idx++ )
    {
        int new_location = (col_idx + _num_shift) % _mat.cols();
        shifted_mat.col(new_location) = _mat.col(col_idx);
    }

    return shifted_mat;

} // circshift


std::vector<float> eig2stdvec( MatrixXd _eigmat )
{
    std::vector<float> vec( _eigmat.data(), _eigmat.data() + _eigmat.size() );
    return vec;
} // eig2stdvec


}  // namespace


double SCManager::distDirectSC ( MatrixXd &_sc1, MatrixXd &_sc2 )
{
    int num_eff_cols = 0; // i.e., to exclude all-nonzero sector
    double sum_sector_similarity = 0;
    for ( int col_idx = 0; col_idx < _sc1.cols(); col_idx++ )
    {
        VectorXd col_sc1 = _sc1.col(col_idx);
        VectorXd col_sc2 = _sc2.col(col_idx);

        if( col_sc1.norm() == 0 || col_sc2.norm() == 0 )
            continue; // don't count this sector pair.

        double sector_similarity = col_sc1.dot(col_sc2) / (col_sc1.norm() * col_sc2.norm());

        sum_sector_similarity = sum_sector_similarity + sector_similarity;
        num_eff_cols = num_eff_cols + 1;
    }

    if (num_eff_cols == 0)
        return std::numeric_limits<double>::infinity();

    double sc_sim = sum_sector_similarity / num_eff_cols;
    return 1.0 - sc_sim;

} // distDirectSC


int SCManager::fastAlignUsingVkey( MatrixXd & _vkey1, MatrixXd & _vkey2)
{
    int argmin_vkey_shift = 0;
    double min_veky_diff_norm = 10000000;
    for ( int shift_idx = 0; shift_idx < _vkey1.cols(); shift_idx++ )
    {
        MatrixXd vkey2_shifted = circshift(_vkey2, shift_idx);

        MatrixXd vkey_diff = _vkey1 - vkey2_shifted;

        double cur_diff_norm = vkey_diff.norm();
        if( cur_diff_norm < min_veky_diff_norm )
        {
            argmin_vkey_shift = shift_idx;
            min_veky_diff_norm = cur_diff_norm;
        }
    }

    return argmin_vkey_shift;

} // fastAlignUsingVkey


std::pair<double, int> SCManager::distanceBtnScanContext( MatrixXd &_sc1, MatrixXd &_sc2,
                                                          MatrixXd &vkey_sc1, MatrixXd &vkey_sc2 )
{
    // 1. fast align using the precomputed variant keys
    int argmin_vkey_shift = fastAlignUsingVkey( vkey_sc1, vkey_sc2 );

    const int SEARCH_RADIUS = round( 0.5 * options_.search_ratio * _sc1.cols() ); // a half of search range
    std::vector<int> shift_idx_search_space { argmin_vkey_shift };
    for ( int ii = 1; ii < SEARCH_RADIUS + 1; ii++ )
    {
        shift_idx_search_space.push_back( (argmin_vkey_shift + ii + _sc1.cols()) % _sc1.cols() );
        shift_idx_search_space.push_back( (argmin_vkey_shift - ii + _sc1.cols()) % _sc1.cols() );
    }
    std::sort(shift_idx_search_space.begin(), shift_idx_search_space.end());

    // 2. fast columnwise diff
    int argmin_shift = 0;
    double min_sc_dist = std::numeric_limits<double>::infinity();
    for ( int num_shift: shift_idx_search_space )
    {
        MatrixXd sc2_shifted = circshift(_sc2, num_shift);
        double cur_sc_dist = distDirectSC( _sc1, sc2_shifted );
        if( cur_sc_dist < min_sc_dist )
        {
            argmin_shift = num_shift;
            min_sc_dist = cur_sc_dist;
        }
    }

    return make_pair(min_sc_dist, argmin_shift);

} // distanceBtnScanContext


MatrixXd SCManager::makeScancontext( const pcl::PointCloud<SCPointType> & _scan_down )
{
    int num_pts_scan_down = _scan_down.points.size();

    // main
    const int NO_POINT = -1000;
    MatrixXd desc = NO_POINT * MatrixXd::Ones(options_.pc_num_ring, options_.pc_num_sector);

    SCPointType pt;
    float azim_angle, azim_range; // wihtin 2d plane
    int ring_idx, sctor_idx;
    for (int pt_idx = 0; pt_idx < num_pts_scan_down; pt_idx++)
    {
        pt.x = _scan_down.points[pt_idx].x;
        pt.y = _scan_down.points[pt_idx].y;
        pt.z = _scan_down.points[pt_idx].z + options_.lidar_height;
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
            continue;

        // xyz to ring, sector
        azim_range = sqrt(pt.x * pt.x + pt.y * pt.y);
        azim_angle = xy2theta(pt.x, pt.y);

        // if range is out of roi, pass
        if( azim_range > options_.pc_max_radius )
            continue;

        ring_idx = std::max( std::min( options_.pc_num_ring, int(ceil( (azim_range / options_.pc_max_radius) * options_.pc_num_ring )) ), 1 );
        sctor_idx = std::max( std::min( options_.pc_num_sector, int(ceil( (azim_angle / 360.0) * options_.pc_num_sector )) ), 1 );

        // taking maximum z
        if ( desc(ring_idx-1, sctor_idx-1) < pt.z ) // -1 means cpp starts from 0
            desc(ring_idx-1, sctor_idx-1) = pt.z; // update for taking maximum value at that bin
    }

    // reset no points to zero (for cosine dist later)
    for ( int row_idx = 0; row_idx < desc.rows(); row_idx++ )
        for ( int col_idx = 0; col_idx < desc.cols(); col_idx++ )
            if( desc(row_idx, col_idx) == NO_POINT )
                desc(row_idx, col_idx) = 0;

    return desc;
} // SCManager::makeScancontext


MatrixXd SCManager::makeRingkeyFromScancontext( Eigen::MatrixXd &_desc )
{
    /*
     * summary: rowwise mean vector
    */
    Eigen::MatrixXd invariant_key(_desc.rows(), 1);
    for ( int row_idx = 0; row_idx < _desc.rows(); row_idx++ )
    {
        Eigen::MatrixXd curr_row = _desc.row(row_idx);
        invariant_key(row_idx, 0) = curr_row.mean();
    }

    return invariant_key;
} // SCManager::makeRingkeyFromScancontext


MatrixXd SCManager::makeSectorkeyFromScancontext( Eigen::MatrixXd &_desc )
{
    /*
     * summary: columnwise mean vector
    */
    Eigen::MatrixXd variant_key(1, _desc.cols());
    for ( int col_idx = 0; col_idx < _desc.cols(); col_idx++ )
    {
        Eigen::MatrixXd curr_col = _desc.col(col_idx);
        variant_key(0, col_idx) = curr_col.mean();
    }

    return variant_key;
} // SCManager::makeSectorkeyFromScancontext



SCManager::SCManager(Options options) : options_(std::move(options)) {}

std::optional<SCManager::Match> SCManager::DetectAndAdd(
    const pcl::PointCloud<SCPointType>& scan, std::size_t reference_count) {
  MatrixXd context = makeScancontext(scan);
  if (context.isZero(0.0)) {
    return std::nullopt;
  }

  MatrixXd ring_key = makeRingkeyFromScancontext(context);
  MatrixXd sector_key = makeSectorkeyFromScancontext(context);
  std::vector<float> query_key = eig2stdvec(ring_key);
  std::optional<Match> match;

  if (reference_count > 0) {
    if (polarcontext_tree_) {
      ++queries_since_tree_;
    }
    if (!polarcontext_tree_ ||
        (reference_count > polarcontext_invkeys_to_search_.size() &&
         queries_since_tree_ >= options_.tree_making_period)) {
      polarcontext_tree_.reset();
      polarcontext_invkeys_to_search_.assign(
          polarcontext_invkeys_mat_.begin(),
          polarcontext_invkeys_mat_.begin() + reference_count);
      polarcontext_tree_ = std::make_unique<InvKeyTree>(
          options_.pc_num_ring, polarcontext_invkeys_to_search_, 10);
      queries_since_tree_ = 0;
    }

    const std::size_t count = std::min(
        static_cast<std::size_t>(options_.num_candidates_from_tree),
        polarcontext_invkeys_to_search_.size());
    std::vector<std::size_t> candidate_indices(count);
    std::vector<float> squared_distances(count);
    nanoflann::KNNResultSet<float> result(count);
    result.init(candidate_indices.data(), squared_distances.data());
    polarcontext_tree_->index->findNeighbors(
        result, query_key.data(), nanoflann::SearchParams(10));

    double best_distance = std::numeric_limits<double>::infinity();
    std::size_t best_index = 0;
    int best_shift = 0;
    for (std::size_t i = 0; i < result.size(); ++i) {
      const std::size_t index = candidate_indices[i];
      const auto [distance, shift] = distanceBtnScanContext(
          context, polarcontexts_[index], sector_key,
          polarcontext_vkeys_[index]);
      if (distance < best_distance) {
        best_distance = distance;
        best_index = index;
        best_shift = shift;
      }
    }
    if (best_distance < options_.sc_dist_thres) {
      match = Match{best_index, best_distance,
                    best_shift * 2.0 * M_PI / options_.pc_num_sector};
    }
  }

  polarcontexts_.push_back(std::move(context));
  polarcontext_vkeys_.push_back(std::move(sector_key));
  polarcontext_invkeys_mat_.push_back(std::move(query_key));
  return match;
}

}  // namespace lightning::scan_context
