#ifndef __G2O_TYPES_H
#define __G2O_TYPES_H

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>

#include <glog/logging.h>
#include <opencv2/core.hpp>

#include "2dNdtLIO/common/eigen_types.h"
#include "2dNdtLIO/common/math_utils.h"


namespace sad {

class VertexSE2 : public g2o::BaseVertex<3, SE2> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void setToOriginImpl() override { _estimate = SE2(); }

    void oplusImpl( const double * update ) override {
        _estimate.translation()[0] += update[0];
        _estimate.translation()[1] += update[1];
        _estimate.so2() = _estimate.so2() * SO2::exp(update[2]);
    }
    bool read(std::istream& is) override { return true; }
    bool write(std::ostream& os) const override { return true; }
};

/**
 * SE2 pose graph使用
 * error = v1.inv * v2 * meas.inv
 */
class EdgeSE2 : public g2o::BaseBinaryEdge<3, SE2, VertexSE2, VertexSE2> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSE2() {}

    void computeError() override {
        VertexSE2* v1 = (VertexSE2*)_vertices[0];
        VertexSE2* v2 = (VertexSE2*)_vertices[1];
        // l1: 历史帧  T_w_l1
        // l2: 当前帧  T_w_l2
        // T_l1_l2: measurement
        // e = Log( T_l1_w * T_w_l2 * T_l1_l2_{-1} )
        _error = ( v1->estimate().inverse() * v2->estimate() * measurement().inverse() ).log();
    }
    
    // TODO jacobian  默认使用自动求导

    bool read(std::istream& is) override { return true; }
    bool write(std::ostream& os) const override { return true; }
};


}



#endif
