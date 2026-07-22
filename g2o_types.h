#ifndef __G2O_TYPES_H
#define __G2O_TYPES_H

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/base_vertex.h>

#include <glog/logging.h>
#include <opencv2/core.hpp>

#include "common/eigen_types.h"
#include "common/math_utils.h"

#include "2dNdtLIO(preintegration)/imu_preintegration.h"


namespace sad {

/**************************************** 顶点 *********************************************/
/**************************************** 顶点 *********************************************/
/**************************************** 顶点 *********************************************/
/**************************************** 顶点 *********************************************/
/**************************************** 顶点 *********************************************/

/**
 * 2d 位姿顶点
 */
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
 * 速度顶点 2d
 */
class VertexVelocity : public g2o::BaseVertex<2, Vec2d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexVelocity() {}
    virtual bool read(std::istream& is) { return false; }
    virtual bool write(std::ostream& os) const { return false; }
    virtual void setToOriginImpl(){ _estimate.setZero(); }
    virtual void oplusImpl( const double * update_ ) { _estimate += Eigen::Map<const Vec2d>(update_); }
};


/**
 * 陀螺仪bias顶点  1d
 */
class VertexGyroBias : public g2o::BaseVertex<1, double> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexGyroBias() {}
    virtual bool read(std::istream& is) { return false; }
    virtual bool write(std::ostream& os) const { return false; }
    virtual void setToOriginImpl(){ _estimate = 0.0; }
    virtual void oplusImpl( const double * update_ ) { _estimate += update_[0]; }
};

/**
 * 加速度计bias顶点  2d
 */
class VertexAcceBias : public VertexVelocity {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexAcceBias(){}
};

/**************************************** 边 *********************************************/
/**************************************** 边 *********************************************/
/**************************************** 边 *********************************************/
/**************************************** 边 *********************************************/
/**************************************** 边 *********************************************/


/**
 * 陀螺仪零偏 bias 边
 * @param 1     残差维度
 * @param double 残差类型
 * @param VertexGyroBias gyro bias i 顶点
 * @param VertexGyroBias gyro bias j 顶点
 */
class EdgeGyroRW : public g2o::BaseBinaryEdge<1, double, VertexGyroBias, VertexGyroBias> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeGyroRW() {}
    bool read(std::istream& is) override { return false; }
    bool write(std::ostream& os) const override { return false; }

    void computeError() {
        const auto * bg_i = dynamic_cast<const VertexGyroBias *>(_vertices[0]);
        const auto * bg_j = dynamic_cast<const VertexGyroBias *>(_vertices[1]);
        _error[0] = bg_j->estimate() - bg_i->estimate();
    }

    virtual void linearizeOplus(){
        // e_bg_i / d_bg_i
        _jacobianOplusXi = - Mat1d::Identity();
        // e_bg_j / d_bg_j
        _jacobianOplusXj = Mat1d::Identity();
    }

    /**
     * 获取 hessian 阵
     */
    Eigen::Matrix<double, 2, 2> GetHessian(){
        linearizeOplus();
        Eigen::Matrix<double, 1, 2> J;  // 1x2
        J.block<1, 1>(0, 0) = _jacobianOplusXi;
        J.block<1, 1>(0, 1) = _jacobianOplusXj;
        return J.transpose() * information() * J;  // J^T * J
    }
};


/**
 * 加速度计零偏 bias 边
 * @param 2     残差维度
 * @param Vec3d 残差类型
 * @param VertexGyroBias acce bias i 顶点
 * @param VertexGyroBias acce bias j 顶点
 */
class EdgeAcceRW : public g2o::BaseBinaryEdge<2, Vec2d, VertexAcceBias, VertexAcceBias> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeAcceRW(){}

    bool read(std::istream& is) override { return false; }
    bool write(std::ostream& os) const override { return false; }

    void computeError() {
        const auto * ba_i = dynamic_cast<const VertexAcceBias *>(_vertices[0]);
        const auto * ba_j = dynamic_cast<const VertexAcceBias *>(_vertices[1]);
        _error = ba_j->estimate() - ba_i->estimate();
    }

    virtual void linearizeOplus() {
        //  顶点1  ba_i    顶点2  ba_j
        // e_ba_i  2x2    e_ba_j  2x2

        _jacobianOplusXi = - Mat2d::Identity();
        _jacobianOplusXj =   Mat2d::Identity();
    }

    Eigen::Matrix<double, 4, 4> GetHessian(){
        linearizeOplus();
        Eigen::Matrix<double, 2, 4> J;
        J.block<2, 2>(0, 0) = _jacobianOplusXi;
        J.block<2, 2>(0, 2) = _jacobianOplusXj;
        return J.transpose() * information() * J;
    }
};

/**
 * 对上一帧 IMU pvq bias 的先验（i时刻）
 * info 由外部指定，通过时间窗口边缘化给出
 * 
 * 顶点顺序: se2, v, bg, ba
 * state:   theta px py vx vy bg bax bay
 * 残差顺序: theta, p, v, bg, ba  8d
 */
class EdgePriorPoseNavState : public g2o::BaseMultiEdge<8, Vec8d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgePriorPoseNavState( const Vec8d & state, const Mat8d & info ) : state_(state) {
        this->resize(4);   // 4 个顶点
        setInformation(info);
    }
    virtual bool read(std::istream & is) { return false; }
    virtual bool write(std::ostream & os) const { return false; }

    void computeError(){
        auto * pose_i  = dynamic_cast<VertexSE2 *>(_vertices[0]);
        auto * v_i     = dynamic_cast<VertexVelocity *>(_vertices[1]);
        auto * bg_i    = dynamic_cast<VertexGyroBias *>(_vertices[2]);
        auto * ba_i    = dynamic_cast<VertexAcceBias *>(_vertices[3]);

        double er  = pose_i->estimate().so2().log()   - state_(0);
        Vec2d ep   = pose_i->estimate().translation() - state_.block<2,1>(1,0);
        Vec2d ev   = v_i->estimate()                  - state_.block<2,1>(3,0);
        double ebg = bg_i->estimate()                 - state_(5);
        Vec2d eba  = ba_i->estimate()                 - state_.block<2,1>(6,0);

        _error << er, ep, ev, ebg, eba;
    }

    virtual void linearizeOplus(){
        auto * pose_i = dynamic_cast<VertexSE2 *>(_vertices[0]);
        //      J[0] 8x3 | J[1] 8x2  | J[2] 8x1  | J[3] 8x2
        //       1Ri 2pi |    2vi    |   1bgi    |   2bai
        // 1er    1      |           |           |
        // 2ep        I  |           |           |
        // 2ev           |     I     |           |
        // 1ebg          |           |     1     |
        // 2eba          |           |           |     I

        _jacobianOplus[0].setZero();  // 8x3
        _jacobianOplus[1].setZero();  // 8x2
        _jacobianOplus[2].setZero();  // 8x1
        _jacobianOplus[3].setZero();  // 8x2
        _jacobianOplus[0](0, 0) = 1.0;                          // er / dRi
        _jacobianOplus[0].block<2, 2>(1, 0) = Mat2d::Identity();// ep / dpi
        _jacobianOplus[1].block<2, 2>(3, 0) = Mat2d::Identity();// ev / dvi
        _jacobianOplus[2](5, 0) = 1.0;                          // ebg / dbg_i
        _jacobianOplus[3].block<2, 2>(6, 0) = Mat2d::Identity();// eba / dba_i
    }

    Mat8d GetHessian() {
        linearizeOplus();
        Mat8d J;
        J.block<8, 3>(0, 0) = _jacobianOplus[0];  // de(se2) / d(se2)   8x3
        J.block<8, 2>(0, 3) = _jacobianOplus[1];  // de(v)   / d(v)     8x2
        J.block<8, 1>(0, 5) = _jacobianOplus[2];  // de(bg)  / d(bg)    8x1
        J.block<8, 2>(0, 6) = _jacobianOplus[3];  // de(ba)  / d(ba)    8x2
        return J.transpose() * information() * J;
    }

    Vec8d state_;
};


/**
 * 2d 轮速计观测边
 * 轮速观测世界速度在自车坐标系下矢量, 3维情况下假设自车不会有y和z方向速度
 */
class EdgeEncoder2D : public g2o::BaseUnaryEdge<2, Vec2d, VertexVelocity> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeEncoder2D() = default;

    /**
     * 构造函数需要知道世界系下速度
     * @param v0
     * @param speed
     */
    EdgeEncoder2D(VertexVelocity* v0, const Vec2d& speed) {
        setVertex(0, v0);
        setMeasurement(speed);
    }

    void computeError() override {
        VertexVelocity* v0 = (VertexVelocity*)_vertices[0];
        _error = v0->estimate() - _measurement;
    }

    void linearizeOplus() override { _jacobianOplusXi.setIdentity(); }
    virtual bool read(std::istream& in) { return true; }
    virtual bool write(std::ostream& out) const { return true; }
};

class EdgeNDT2d : public g2o::BaseUnaryEdge<2, Vec2d, VertexSE2> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeNDT2d() = default;

    /// 需要查询NDT内部的体素，这里用一个函数式给设置过去
    using QueryVoxelFunc = std::function<bool(const Vec2d& query_pt, Vec2d& mu, Mat2d& info)>;

    EdgeNDT2d(VertexSE2* v0, const Vec2d& pt, QueryVoxelFunc func) {
        setVertex(0, v0);
        pt_ = pt;
        query_ = func;

        Vec2d q = v0->estimate() * pt_;
        if (query_(q, mu_, info_)) {
            setInformation(info_);
            valid_ = true;
        } else {
            valid_ = false;
        }
    }

    bool isValid() const { return valid_; }

    Mat3d GetHessian() {
        linearizeOplus();
        return _jacobianOplusXi.transpose() * info_ * _jacobianOplusXi;
    }

    /// 残差计算
    void computeError() override {
        VertexSE2* v0 = (VertexSE2*)_vertices[0];
        Vec2d q = v0->estimate().so2() * pt_ + v0->estimate().translation();

        if (query_(q, mu_, info_)) {
            _error = q - mu_;
            setInformation(info_);
            valid_ = true;
        } else {
            valid_ = false;
            _error.setZero();
            setLevel(1);
        }
    }

    /// 线性化
    void linearizeOplus() override {
        if (valid_) {
            VertexSE2* v0 = (VertexSE2*)_vertices[0];
            SO2 R = v0->estimate().so2();

            _jacobianOplusXi.setZero();
            _jacobianOplusXi.block<2, 1>(0, 0) = R.matrix() * Vec2d( -pt_(1), pt_(0) );  // 对R
            _jacobianOplusXi.block<2, 2>(0, 1) = Mat2d::Identity();                   // 对p
        } else {
            _jacobianOplusXi.setZero();
        }
    }

    virtual bool read(std::istream& in) { return true; }
    virtual bool write(std::ostream& out) const { return true; }

private:
    QueryVoxelFunc query_;
    Vec2d pt_ = Vec2d::Zero();
    Vec2d mu_ = Vec2d::Zero();
    Mat2d info_ = Mat2d::Identity();
    bool valid_ = false;
};


/**
 * 3 自由度的 GNSS (2d)
 * 误差的角度在前，平移在后
 */
class EdgeGNSS : public g2o::BaseUnaryEdge<3, SE2, VertexSE2> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeGNSS() = default;
    EdgeGNSS(VertexSE2* v, const SE2& obs) {
        setVertex(0, v);
        setMeasurement(obs);
    }

    void computeError() override {
        VertexSE2* v = (VertexSE2*)_vertices[0];
        // _error[0] = (_measurement.so2().inverse() * v->estimate().so2()).log();
        _error[0]        = v->estimate().so2().log() - _measurement.so2().log();
        _error.tail<2>() = v->estimate().translation() - _measurement.translation();
    };

    void linearizeOplus() override {
        VertexSE2* v = (VertexSE2*)_vertices[0];
        // jacobian 3x3
        _jacobianOplusXi.setZero();
        // _jacobianOplusXi.block<1, 1>(0, 0) = (_measurement.so3().inverse() * v->estimate().so3()).jr_inv();  // dR/dR
        // _jacobianOplusXi(0, 0) = 1.0;  // dR/dR
        // _jacobianOplusXi.block<2, 2>(1, 1) = Mat2d::Identity();                                              // dp/dp
        _jacobianOplusXi.setIdentity();
    }

    Mat3d GetHessian() {
        linearizeOplus();
        return _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
    }

    virtual bool read(std::istream& in) { return true; }
    virtual bool write(std::ostream& out) const { return true; }

   private:
};

/// 与预积分相关的 vertex, edge
/**
 * 预积分
 * 1. 预积分的边， 约束上一时刻的 15 维状态与下一时刻的旋转、 平移、 速度。
 * 2. 零偏随机游走的边， 共两种， 连接两个时刻的零偏状态。
 * 3. GNSS 的观测边。 因为使用六自由度观测， 所以它关联单个时刻的位姿。
 * 4. 先验信息， 刻画上一时刻的状态分布， 关联上一时刻的 15 维状态。
 * 5. 轮速计的观测边。 关联上一时刻的速度顶点。
 */
class EdgeInertial : public g2o::BaseMultiEdge<5, Vec5d> {
/**
 * 关联 pose_i, v_i, bg_i, ba_i, pose_j, v_j
 * 预积分边
 * 连接6个顶点：上一帧的pose, v, bg, ba，下一帧的pose, v
 * 观测量为9维，即预积分残差, 顺序：R, v, p
 * information从预积分类中获取，构造函数中计算
 */
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * 构造函数中需要指定预积分类对象
     * @param preinteg   预积分对象指针
     * @param gravity    重力矢量
     * @param weight     权重
     */
    EdgeInertial( IMUPreintegration & preinteg, double weight = 1.0) : parent_(preinteg), dt_ij(preinteg.dt_) {
        this->resize(6);  // 关联 6 个顶点
        // 在图优化中，信息矩阵就是协方差矩阵的逆
        setInformation(preinteg.cov_.inverse() * weight);
    }

    bool read(std::istream& is) override { return false; }
    bool write(std::ostream& os) const override { return false; }

    void computeError() override {
        // 6个顶点
        auto * p_i  = dynamic_cast<const VertexSE2*>(_vertices[0]);
        auto * v_i  = dynamic_cast<const VertexVelocity*>(_vertices[1]);
        auto * bg_i = dynamic_cast<const VertexGyroBias*>(_vertices[2]);
        auto * ba_i = dynamic_cast<const VertexAcceBias*>(_vertices[3]);
        auto * p_j  = dynamic_cast<const VertexSE2*>(_vertices[4]);
        auto * v_j  = dynamic_cast<const VertexVelocity*>(_vertices[5]);

        // 公式(4.32)
        // 每次优化之后，bias 都会发生变化，响应也要不断用 estimate 修正 Delta 预测
        const SO2   dR = parent_.getDeltaR(bg_i->estimate());
        const Vec2d dv = parent_.getDeltaV(bg_i->estimate(), ba_i->estimate());
        const Vec2d dp = parent_.getDeltaP(bg_i->estimate(), ba_i->estimate());

        // 公式(4.41)
        // 计算残差项
        // const double er = ( dR.inverse() * p_i->estimate().so2().inverse() * p_j->estimate().so2() ).log();
        const double er = p_j->estimate().so2().log() - p_i->estimate().so2().log() - bg_i->estimate();
        Mat2d RiT = p_i->estimate().so2().inverse().matrix();
        const Vec2d ev = RiT * ( v_j->estimate() - v_i->estimate() ) - dv;
        const Vec2d ep = RiT * ( p_j->estimate().translation() - p_i->estimate().translation() - v_i->estimate() * dt_ij ) - dp;

        _error << er, ev, ep;
    }

    void linearizeOplus() override {
        // 6个顶点
        auto * p_i  = dynamic_cast<const VertexSE2*>(_vertices[0]);
        auto * v_i  = dynamic_cast<const VertexVelocity*>(_vertices[1]);
        auto * bg_i = dynamic_cast<const VertexGyroBias*>(_vertices[2]);
        auto * ba_i = dynamic_cast<const VertexAcceBias*>(_vertices[3]);
        auto * p_j  = dynamic_cast<const VertexSE2*>(_vertices[4]);
        auto * v_j  = dynamic_cast<const VertexVelocity*>(_vertices[5]);

        // 中间符号
        double bgi = bg_i->estimate();
        Vec2d bai = ba_i->estimate();
        double dbg = bgi - parent_.bg_;

        const SO2 Ri  = p_i->estimate().so2();
        const SO2 RiT = Ri.inverse();
        const SO2 Rj  = p_j->estimate().so2();

        // (4.41a) 与上面的 computeError 里面计算一样
        const SO2 deltaRT = parent_.getDeltaR(dbg);  // 每一步迭代，都用(4.32)修正预积分观测
        const SO2   eR = deltaRT.inverse() * RiT * Rj;
        const double er = eR.log();  // 这个其实也是 _error.block<3,3>(0,0)
        // invJr = 1

        // 估计值
        const Vec2d vi = v_i->estimate();
        const Vec2d vj = v_j->estimate();
        const Vec2d ti = p_i->estimate().translation();
        const Vec2d tj = p_j->estimate().translation();

        const double dR_dbg = parent_.dR_dbg_;
        const Vec2d dv_dbg = parent_.dv_dbg_;
        const Mat2d dv_dba = parent_.dv_dba_;
        const Vec2d dp_dbg = parent_.dp_dbg_;
        const Mat2d dp_dba = parent_.dp_dba_;

        // j[0] 5x3:   j[1] 5x2  j[2] 5x1  j[3] 5x2  j[4] 5x3  j[5] 5x2
        //     Ri  pi     vi       bg_i      ba_i      Rj  pj    vj
        // eR 1x1 1x2    1x2       1x1       1x2      1x1 1x2   1x2
        // ev 2x1 2x2    2x2       2x1       2x2      2x1 2x2   2x2
        // ep 2x1 2x2    2x2       2x1       2x2      2x1 2x2   2x2

        _jacobianOplus[0].setZero();
        _jacobianOplus[1].setZero();
        _jacobianOplus[2].setZero();
        _jacobianOplus[3].setZero();
        _jacobianOplus[4].setZero();
        _jacobianOplus[5].setZero();

        // 残差对 Ri
        // dR / dRi (4.42)
        // _jacobianOplus[0].block<1, 1>(0, 0) = - Mat1d( ( Rj.inverse() * Ri ).log() );  // - invJr * RjT * Ri
        _jacobianOplus[0].block<1, 1>(0, 0) = - Mat1d::Identity();  // - invJr * RjT * Ri   TUDO

        // dv / dRi (4.47)    // TUDO 不确定符号对不对
        Vec2d v_ij = vj - vi;
        _jacobianOplus[0].block<2, 1>(1, 0) = RiT * Vec2d( -v_ij(1), v_ij(0) ); // SO2::hat( RiT * ( vj - vi ) );
        // dp / dRi (4.48d)   // TUDO 不确定对不对
        Vec2d t_ij = tj - ti - vi * dt_ij;
        _jacobianOplus[0].block<2, 1>(3, 0) = RiT * Vec2d( -t_ij(1), t_ij(0) ); // SO2::hat( RiT * ( tj - ti - vi * dt_ij ) );

        // 残差对 pi
        // 没有 dR / dpi 和 dv / dpi
        // dp / dpi
        _jacobianOplus[0].block<2, 2>(3, 1) = - RiT.matrix();

        // 残差对 vi
        // dv / dvi (4.46a)
        _jacobianOplus[1].block<2, 2>(1, 0) = - RiT.matrix();
        // dp / dvi (4.48c)
        _jacobianOplus[1].block<2, 2>(3, 0) = - RiT.matrix() * dt_ij;

        // 残差对 bg_i
        // dR / dbg_i (4.45)
        // _jacobianOplus[2].block<1, 1>(0, 0) = - Mat1d(( eR.inverse() ).log() * dR_dbg);  // TUDO 不确定对不对
        _jacobianOplus[2].block<1, 1>(0, 0) = - Mat1d( dR_dbg );  // TUDO 不确定对不对
        // dv / dbg_i -(4.38)
        _jacobianOplus[2].block<2, 1>(1, 0) = - dv_dbg;
        // dp / dbg_i -(4.38)
        _jacobianOplus[2].block<2, 1>(3, 0) = - dp_dbg;

        // 残差对 ba_i
        // dv / dba_i -(4.38)
        _jacobianOplus[3].block<2, 2>(1, 0) = - dv_dba;
        // dp / dba_i -(4.38)
        _jacobianOplus[3].block<2, 2>(3, 0) = - dp_dba;

        // 残差对 Rj
        // dR / dRj (4.43)
        _jacobianOplus[4].block<1, 1>(0, 0) = Mat1d::Identity();  // invJr

        // 残差对 pj
        // dp / dpj (4.48b)
        _jacobianOplus[4].block<2, 2>(3, 1) = RiT.matrix();

        // 残差对 vj
        // dv / dvj (4.46b)
        _jacobianOplus[5].block<2, 2>(1, 0) = RiT.matrix();
    }

    Eigen::Matrix<double, 13, 13> GetHessian() {
        linearizeOplus();
        Eigen::Matrix<double, 5, 13> J;
        J.block<5, 3>(0, 0)  = _jacobianOplus[0];
        J.block<5, 2>(0, 3)  = _jacobianOplus[1];
        J.block<5, 1>(0, 5)  = _jacobianOplus[2];
        J.block<5, 2>(0, 6)  = _jacobianOplus[3];
        J.block<5, 3>(0, 8)  = _jacobianOplus[4];
        J.block<5, 2>(0, 11) = _jacobianOplus[5];
        return J.transpose() * information() * J;
    }

private:
    IMUPreintegration parent_;
    double dt_ij = 0.0;
};


/** loop closure
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
        _error = ( v1->estimate().inverse() * v2->estimate() * measurement().inverse() ).log();
    }
    
    // TODO jacobian  默认使用自动求导

    bool read(std::istream& is) override { return true; }
    bool write(std::ostream& os) const override { return true; }
};



/** 在 submap 中的位姿
 * 求解 T_li  |  T_si = T_sl * T_li | e = Log( T_sl * T_li * T_si.inv() )
 * @param pose_in_submap_imu   imu 名义位姿    T_si
 * @param pose_in_submap_lidar lidar scan to map 的位姿  T_sl
 */
class EdgeTliSE2 : public g2o::BaseUnaryEdge<3, SE2, VertexSE2>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeTliSE2(const SE2 pose_in_submap_imu, const SE2 pose_in_submap_lidar)
    : pose_in_submap_imu_(pose_in_submap_imu), pose_in_submap_lidar_(pose_in_submap_lidar){}

    void computeError() override {
        VertexSE2 * v = static_cast<VertexSE2 *>( _vertices[0] );
        _error = ( pose_in_submap_lidar_ * v->estimate() * pose_in_submap_imu_.inverse() ).log();
    }

    /// 自动求导

    bool read(std::istream& is) override { return true; }
    bool write(std::ostream& os) const override { return true; }

private:
    SE2 pose_in_submap_imu_;
    SE2 pose_in_submap_lidar_;
};


}



#endif
