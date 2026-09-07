#include <Gril_Calib/Gril_Calib.h>

/*
Description: Gril-Calib (Heavily adapted from LI-Init by Fangcheng Zhu)
Modifier : Taeyoung Kim (https://github.com/Taeyoung96)
*/

Gril_Calib::Gril_Calib()
        : time_delay_IMU_wtr_Lidar(0.0), time_lag_1(0.0), time_lag_2(0.0), lag_IMU_wtr_Lidar(0) {
    fout_LiDAR_meas.open(FILE_DIR("LiDAR_meas.txt"), ios::out);
    fout_IMU_meas.open(FILE_DIR("IMU_meas.txt"), ios::out);
    fout_before_filt_IMU.open(FILE_DIR("IMU_before_filter.txt"), ios::out);
    fout_before_filt_Lidar.open(FILE_DIR("Lidar_before_filter.txt"), ios::out);
    fout_acc_cost.open(FILE_DIR("acc_cost.txt"), ios::out);
    fout_after_rot.open(FILE_DIR("Lidar_omg_after_rot.txt"), ios::out);

    fout_LiDAR_ang_vel.open(FILE_DIR("Lidar_ang_vel.txt"), ios::out);
    fout_IMU_ang_vel.open(FILE_DIR("IMU_ang_vel.txt"), ios::out);
    fout_Jacob_trans.open(FILE_DIR("Jacob_trans.txt"), ios::out);

    fout_LiDAR_meas_after.open(FILE_DIR("LiDAR_meas_after.txt"), ios::out);


    data_accum_length = 300;

    trans_IL_x = 0.0;
    trans_IL_y = 0.0;
    trans_IL_z = 0.0;
    bound_th = 0.1;
    set_boundary = false;

    Rot_Grav_wrt_Init_Lidar = Eye3d;
    Trans_Lidar_wrt_IMU = Zero3d;
    Rot_Lidar_wrt_IMU = Eye3d;
    gyro_bias = Zero3d;
    acc_bias = Zero3d;
}

Gril_Calib::~Gril_Calib() = default;

void Gril_Calib::set_IMU_state(const deque<CalibState> &IMU_states) {
    IMU_state_group.assign(IMU_states.begin(), IMU_states.end() - 1);
}

void Gril_Calib::set_Lidar_state(const deque<CalibState> &Lidar_states) {
    Lidar_state_group.assign(Lidar_states.begin(), Lidar_states.end() - 1);
}

void Gril_Calib::set_states_2nd_filter(const deque<CalibState> &IMU_states, const deque<CalibState> &Lidar_states) {
    for (int i = 0; i < IMU_state_group.size(); i++) {
        IMU_state_group[i].ang_acc = IMU_states[i].ang_acc;
        Lidar_state_group[i].ang_acc = Lidar_states[i].ang_acc;
        Lidar_state_group[i].linear_acc = Lidar_states[i].linear_acc;
    }
}

void Gril_Calib::fout_before_filter() {
    for (auto it_IMU = IMU_state_group.begin(); it_IMU != IMU_state_group.end() - 1; it_IMU++) {
        fout_before_filt_IMU << setprecision(15) << it_IMU->ang_vel.transpose() << " " << it_IMU->ang_vel.norm() << " "
                             << it_IMU->linear_acc.transpose() << " " << it_IMU->timeStamp << endl;
    }
    for (auto it = Lidar_state_group.begin(); it != Lidar_state_group.end() - 1; it++) {
        fout_before_filt_Lidar << setprecision(15) << it->ang_vel.transpose() << " " << it->ang_vel.norm() << " "
                               << it->timeStamp << endl;
    }
}

void Gril_Calib::fout_check_lidar() {
    auto it_Lidar_state = Lidar_state_group.begin() + 1;
    for (; it_Lidar_state != Lidar_state_group.end() - 2; it_Lidar_state++) {
        fout_LiDAR_meas_after << setprecision(12) << it_Lidar_state->ang_vel.transpose() << " "
                        << it_Lidar_state->ang_vel.norm()
                        << " " <<
                        it_Lidar_state->linear_acc.transpose() << " "
                        << it_Lidar_state->ang_acc.transpose()
                        << " " << it_Lidar_state->timeStamp << endl;
    }
}


void Gril_Calib::push_ALL_IMU_CalibState(const sensor_msgs::Imu::ConstPtr &msg, const double &mean_acc_norm) {
    CalibState IMUstate;
    double invert = -1.0;
    IMUstate.ang_vel = V3D(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    IMUstate.linear_acc =
            V3D(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z) / mean_acc_norm *
            G_m_s2;

    IMUstate.timeStamp = msg->header.stamp.toSec();
    IMU_state_group_ALL.push_back(IMUstate);
}

void Gril_Calib::push_IMU_CalibState(const V3D &omg, const V3D &acc, const double &timestamp) {
    CalibState IMUstate;
    IMUstate.ang_vel = omg;
    IMUstate.linear_acc = acc;
    IMUstate.timeStamp = timestamp;
    IMU_state_group.push_back(IMUstate);
}

void Gril_Calib::push_Lidar_CalibState(const M3D &rot, const V3D &pos, const V3D &omg, const V3D &linear_vel, const double &timestamp) {
    CalibState Lidarstate;
    Lidarstate.rot_end = rot;
    Lidarstate.pos_end = pos;
    Lidarstate.ang_vel = omg;
    Lidarstate.linear_vel = linear_vel;
    Lidarstate.timeStamp = timestamp;
    Lidar_state_group.push_back(Lidarstate);
}

void Gril_Calib::push_Plane_Constraint(const Eigen::Quaterniond &q_lidar, const Eigen::Quaterniond &q_imu, const V3D &normal_lidar,
                                                                                                             const double &distance_lidar) {
    Lidar_wrt_ground_group.push_back(q_lidar);
    IMU_wrt_ground_group.push_back(q_imu);
    normal_vector_wrt_lidar_group.push_back(normal_lidar);
    distance_Lidar_wrt_ground_group.push_back(distance_lidar);
}

void Gril_Calib::downsample_interpolate_IMU(const double &move_start_time) {

    while (IMU_state_group_ALL.size() > 2 && IMU_state_group_ALL.front().timeStamp < move_start_time - 3.0)
        IMU_state_group_ALL.pop_front();
    while (Lidar_state_group.size() > 2 && Lidar_state_group.front().timeStamp < move_start_time - 3.0)
        Lidar_state_group.pop_front();

    // Original IMU measurements
    deque<CalibState> IMU_states_all_origin;
    IMU_states_all_origin.assign(IMU_state_group_ALL.begin(), IMU_state_group_ALL.end() - 1);

    // Mean filter to attenuate noise
    int mean_filt_size = 3;
    for (int i = mean_filt_size; i < IMU_state_group_ALL.size() - mean_filt_size; i++) {
        V3D acc_real = Zero3d;
        for (int k = -mean_filt_size; k < mean_filt_size + 1; k++)
            acc_real += (IMU_states_all_origin[i + k].linear_acc - acc_real) / (k + mean_filt_size + 1);
        IMU_state_group_ALL[i].linear_acc = acc_real;
    }

    // Down-sample and interpolation，Fig.4 in the paper
    for (int i = 0; i < Lidar_state_group.size(); i++) {
        for (int j = 1; j < IMU_state_group_ALL.size(); j++) {
            if (IMU_state_group_ALL[j - 1].timeStamp <= Lidar_state_group[i].timeStamp
                && IMU_state_group_ALL[j].timeStamp > Lidar_state_group[i].timeStamp) {
                CalibState IMU_state_interpolation;
                double delta_t = IMU_state_group_ALL[j].timeStamp - IMU_state_group_ALL[j - 1].timeStamp;
                double delta_t_right = IMU_state_group_ALL[j].timeStamp - Lidar_state_group[i].timeStamp;
                double s = delta_t_right / delta_t;

                IMU_state_interpolation.ang_vel = s * IMU_state_group_ALL[j - 1].ang_vel +
                                                  (1 - s) * IMU_state_group_ALL[j].ang_vel;

                IMU_state_interpolation.linear_acc = s * IMU_state_group_ALL[j - 1].linear_acc +
                                                     (1 - s) * IMU_state_group_ALL[j].linear_acc;
                push_IMU_CalibState(IMU_state_interpolation.ang_vel, IMU_state_interpolation.linear_acc,
                                    Lidar_state_group[i].timeStamp);
                break;
            }
        }
    }

}

namespace {

// Small self-contained B-spline least-squares fitter.  It uses a clamped,
// uniform cubic basis in normalized time and a second-difference penalty to
// suppress the high-frequency noise that makes numerical differentiation of
// LiDAR odometry particularly unstable.
class BSplineFit {
public:
    bool fit(const std::vector<double> &times, const MatrixXd &values,
             int requested_ctrl_points, int requested_degree, double smoothing) {
        if (times.size() < 4 || values.rows() != static_cast<int>(times.size()) ||
            values.cols() == 0) {
            return false;
        }
        t0_ = times.front();
        t1_ = times.back();
        duration_ = t1_ - t0_;
        if (!(duration_ > 1.0e-9)) return false;

        degree_ = std::max(1, std::min(requested_degree, 5));
        const int max_ctrl = static_cast<int>(times.size());
        ctrl_points_ = requested_ctrl_points > 0
                       ? requested_ctrl_points
                       : std::max(degree_ + 1,
                                  std::min(50, std::max(8, max_ctrl / 4)));
        ctrl_points_ = std::max(degree_ + 1, std::min(ctrl_points_, max_ctrl));
        knots_.assign(ctrl_points_ + degree_ + 1, 0.0);
        for (int i = 0; i < static_cast<int>(knots_.size()); ++i) {
            if (i <= degree_) knots_[i] = 0.0;
            else if (i >= ctrl_points_) knots_[i] = 1.0;
            else knots_[i] = static_cast<double>(i - degree_) /
                             static_cast<double>(ctrl_points_ - degree_);
        }

        MatrixXd design(times.size(), ctrl_points_);
        for (int row = 0; row < design.rows(); ++row) {
            std::vector<double> basis;
            basis_and_derivative(normalize_time(times[row]), 0, basis);
            for (int col = 0; col < ctrl_points_; ++col) design(row, col) = basis[col];
        }

        MatrixXd normal = design.transpose() * design;
        if (ctrl_points_ >= 3 && smoothing > 0.0) {
            MatrixXd second_diff = MatrixXd::Zero(ctrl_points_ - 2, ctrl_points_);
            for (int i = 0; i < second_diff.rows(); ++i) {
                second_diff(i, i) = 1.0;
                second_diff(i, i + 1) = -2.0;
                second_diff(i, i + 2) = 1.0;
            }
            normal += smoothing * second_diff.transpose() * second_diff;
        }
        // Protect the solve from a nearly singular system for very short or
        // almost stationary trajectories without materially changing the fit.
        normal.diagonal().array() += 1.0e-10;
        coefficients_ = normal.ldlt().solve(design.transpose() * values);
        return coefficients_.allFinite();
    }

    VectorXd evaluate(double time, int derivative_order) const {
        VectorXd result = VectorXd::Zero(coefficients_.cols());
        if (coefficients_.rows() == 0) return result;
        std::vector<double> basis;
        basis_and_derivative(normalize_time(time), derivative_order, basis);
        const double scale = std::pow(duration_, derivative_order);
        for (int i = 0; i < ctrl_points_; ++i) result += basis[i] * coefficients_.row(i).transpose();
        return result / scale;
    }

    double t0() const { return t0_; }
    double t1() const { return t1_; }

private:
    double normalize_time(double time) const {
        return std::max(0.0, std::min(1.0, (time - t0_) / duration_));
    }

    double basis_value(int i, int degree, double u) const {
        if (degree == 0) {
            const bool in_interval = (knots_[i] <= u && u < knots_[i + 1]);
            const bool at_right_endpoint = (u >= 1.0 &&
                                             i == ctrl_points_ - 1 &&
                                             knots_[i + 1] >= 1.0);
            return (in_interval || at_right_endpoint) ? 1.0 : 0.0;
        }
        double value = 0.0;
        const double left_den = knots_[i + degree] - knots_[i];
        const double right_den = knots_[i + degree + 1] - knots_[i + 1];
        if (left_den > 0.0) value += (u - knots_[i]) / left_den *
                                      basis_value(i, degree - 1, u);
        if (right_den > 0.0) value += (knots_[i + degree + 1] - u) / right_den *
                                       basis_value(i + 1, degree - 1, u);
        return value;
    }

    double basis_derivative(int i, int degree, int order, double u) const {
        if (order == 0) return basis_value(i, degree, u);
        if (degree == 0 || order > degree) return 0.0;
        double value = 0.0;
        const double left_den = knots_[i + degree] - knots_[i];
        const double right_den = knots_[i + degree + 1] - knots_[i + 1];
        if (left_den > 0.0) value += degree / left_den *
                                      basis_derivative(i, degree - 1, order - 1, u);
        if (right_den > 0.0) value -= degree / right_den *
                                       basis_derivative(i + 1, degree - 1, order - 1, u);
        return value;
    }

    void basis_and_derivative(double u, int order, std::vector<double> &basis) const {
        basis.assign(ctrl_points_, 0.0);
        if (order > degree_) return;
        for (int i = 0; i < ctrl_points_; ++i) basis[i] = basis_derivative(i, degree_, order, u);
    }

    double t0_ = 0.0;
    double t1_ = 0.0;
    double duration_ = 0.0;
    int degree_ = 3;
    int ctrl_points_ = 0;
    std::vector<double> knots_;
    MatrixXd coefficients_;
};

M3D right_jacobian(const V3D &phi) {
    const double theta = phi.norm();
    const M3D Omega = skew_sym_mat(phi);
    if (theta < 1.0e-6) return M3D::Identity() - 0.5 * Omega + (1.0 / 6.0) * Omega * Omega;
    const double theta2 = theta * theta;
    return M3D::Identity() - (1.0 - std::cos(theta)) / theta2 * Omega +
           (theta - std::sin(theta)) / (theta2 * theta) * Omega * Omega;
}

V3D spline_body_angular_velocity(const BSplineFit &fit, double time) {
    const V3D phi = fit.evaluate(time, 0);
    const V3D phi_dot = fit.evaluate(time, 1);
    return right_jacobian(phi) * phi_dot;
}

} // namespace

bool Gril_Calib::bspline_fit_lidar_kinematics() {
    if (Lidar_state_group.size() < 8) {
        std::cerr << "[B-spline] Not enough LiDAR poses (need at least 8)." << std::endl;
        return false;
    }

    const int n = static_cast<int>(Lidar_state_group.size());
    std::vector<double> times;
    times.reserve(n);
    MatrixXd positions(n, 3);
    MatrixXd rotation_vectors(n, 3);
    const M3D initial_rotation = Lidar_state_group.front().rot_end;
    V3D previous_phi = Zero3d;
    double previous_time = -std::numeric_limits<double>::infinity();

    for (int i = 0; i < n; ++i) {
        const double time = Lidar_state_group[i].timeStamp;
        if (i > 0 && !(time > previous_time)) {
            std::cerr << "[B-spline] LiDAR timestamps are not strictly increasing." << std::endl;
            return false;
        }
        times.push_back(time);
        previous_time = time;
        positions.row(i) = Lidar_state_group[i].pos_end.transpose();

        const M3D relative_rotation = initial_rotation.transpose() * Lidar_state_group[i].rot_end;
        const V3D principal = Log(relative_rotation);
        V3D phi = principal;
        // Select the equivalent rotation vector closest to the preceding one.
        // This avoids a +/-pi jump during a yaw sweep while retaining the
        // inexpensive SO(3) logarithm used elsewhere in GRIL-Calib.
        const double angle = principal.norm();
        if (angle > 1.0e-8) {
            const V3D axis = principal / angle;
            double best_distance = std::numeric_limits<double>::infinity();
            for (int winding = -4; winding <= 4; ++winding) {
                const V3D candidate = principal + (2.0 * M_PI * winding) * axis;
                const double distance = (candidate - previous_phi).squaredNorm();
                if (distance < best_distance) {
                    best_distance = distance;
                    phi = candidate;
                }
            }
        }
        rotation_vectors.row(i) = phi.transpose();
        previous_phi = phi;
    }

    BSplineFit position_fit;
    BSplineFit rotation_fit;
    if (!position_fit.fit(times, positions, bspline_ctrl_points, bspline_degree, bspline_smoothing) ||
        !rotation_fit.fit(times, rotation_vectors, bspline_ctrl_points, bspline_degree, bspline_smoothing)) {
        std::cerr << "[B-spline] Failed to solve the LiDAR trajectory fit." << std::endl;
        return false;
    }

    double min_dt = std::numeric_limits<double>::infinity();
    for (int i = 1; i < n; ++i) min_dt = std::min(min_dt, times[i] - times[i - 1]);
    const double derivative_step = std::max(1.0e-5, std::min(0.25 * min_dt,
                                                               0.01 * (times.back() - times.front())));

    for (int i = 0; i < n; ++i) {
        const double time = times[i];
        const V3D phi = rotation_fit.evaluate(time, 0);
        Lidar_state_group[i].rot_end = initial_rotation * Exp(phi);
        Lidar_state_group[i].pos_end = position_fit.evaluate(time, 0);
        Lidar_state_group[i].linear_vel = position_fit.evaluate(time, 1);
        Lidar_state_group[i].linear_acc = position_fit.evaluate(time, 2);
        Lidar_state_group[i].ang_vel = spline_body_angular_velocity(rotation_fit, time);

        // Differentiate the body angular velocity with a small symmetric
        // stencil.  The position and rotation splines themselves are
        // differentiated analytically through the B-spline basis; this last
        // derivative only accounts for the SO(3) Jacobian's time variation.
        const double h = std::min(derivative_step, 0.5 * (times.back() - times.front()));
        if (time <= times.front() + h) {
            Lidar_state_group[i].ang_acc =
                    (spline_body_angular_velocity(rotation_fit, time + h) -
                     spline_body_angular_velocity(rotation_fit, time)) / h;
        } else if (time >= times.back() - h) {
            Lidar_state_group[i].ang_acc =
                    (spline_body_angular_velocity(rotation_fit, time) -
                     spline_body_angular_velocity(rotation_fit, time - h)) / h;
        } else {
            Lidar_state_group[i].ang_acc =
                    (spline_body_angular_velocity(rotation_fit, time + h) -
                     spline_body_angular_velocity(rotation_fit, time - h)) /
                    (2.0 * h);
        }

        fout_LiDAR_meas << setprecision(12)
                        << Lidar_state_group[i].ang_vel.transpose() << " "
                        << Lidar_state_group[i].ang_vel.norm() << " "
                        << (Lidar_state_group[i].linear_acc - STD_GRAV).transpose() << " "
                        << Lidar_state_group[i].ang_acc.transpose() << " "
                        << Lidar_state_group[i].timeStamp << endl;
    }

    std::cout << "[B-spline] Fitted " << n << " LiDAR poses with degree "
              << bspline_degree << ", control points "
              << (bspline_ctrl_points > 0 ? bspline_ctrl_points : std::max(bspline_degree + 1,
                  std::min(50, std::max(8, n / 4))))
              << ", smoothing " << bspline_smoothing << std::endl;
    return true;
}

// Calculates IMU angular acceleration and, when requested, the legacy LiDAR
// derivatives.  The normal calibration path now passes false for the LiDAR
// part because those quantities come from bspline_fit_lidar_kinematics().
void Gril_Calib::central_diff(bool compute_lidar_kinematics) {
    if (IMU_state_group.size() >= 3) for (size_t i = 1; i + 1 < IMU_state_group.size(); ++i) {
        const double dt = IMU_state_group[i + 1].timeStamp - IMU_state_group[i - 1].timeStamp;
        if (dt <= 0.0) continue;
        IMU_state_group[i].ang_acc =
                (IMU_state_group[i + 1].ang_vel - IMU_state_group[i - 1].ang_vel) / dt;
        fout_IMU_meas << setprecision(12) << IMU_state_group[i].ang_vel.transpose() << " "
                      << IMU_state_group[i].ang_vel.norm() << " "
                      << IMU_state_group[i].linear_acc.transpose() << " "
                      << IMU_state_group[i].ang_acc.transpose() << " "
                      << IMU_state_group[i].timeStamp << endl;
    }

    if (!compute_lidar_kinematics) return;
    if (Lidar_state_group.size() >= 3) for (size_t i = 1; i + 1 < Lidar_state_group.size(); ++i) {
        const double dt = Lidar_state_group[i + 1].timeStamp - Lidar_state_group[i - 1].timeStamp;
        if (dt <= 0.0) continue;
        Lidar_state_group[i].ang_acc =
                (Lidar_state_group[i + 1].ang_vel - Lidar_state_group[i - 1].ang_vel) / dt;
        Lidar_state_group[i].linear_acc =
                (Lidar_state_group[i + 1].linear_vel - Lidar_state_group[i - 1].linear_vel) / dt;

        fout_LiDAR_meas << setprecision(12) << Lidar_state_group[i].ang_vel.transpose() << " "
                        << Lidar_state_group[i].ang_vel.norm() << " "
                        << (Lidar_state_group[i].linear_acc - STD_GRAV).transpose() << " "
                        << Lidar_state_group[i].ang_acc.transpose() << " "
                        << Lidar_state_group[i].timeStamp << endl;
    }
}

// Temporal calibration by Cross-Correlation : calculate time_lag_1
void Gril_Calib::xcorr_temporal_init(const double &odom_freq) {
    int N = IMU_state_group.size();
    //Calculate mean value of IMU and LiDAR angular velocity
    double mean_IMU_ang_vel = 0, mean_LiDAR_ang_vel = 0;
    for (int i = 0; i < N; i++) {
        mean_IMU_ang_vel += (IMU_state_group[i].ang_vel.norm() - mean_IMU_ang_vel) / (i + 1);
        mean_LiDAR_ang_vel += (Lidar_state_group[i].ang_vel.norm() - mean_LiDAR_ang_vel) / (i + 1);
    }

    //Calculate zero-centered cross correlation
    double max_corr = -DBL_MAX;
    for (int lag = -N + 1; lag < N; lag++) {
        double corr = 0;
        int cnt = 0;
        for (int i = 0; i < N; i++) {
            int j = i + lag;
            if (j < 0 || j > N - 1)
                continue;
            else {
                cnt++;
                corr += (IMU_state_group[i].ang_vel.norm() - mean_IMU_ang_vel) *
                        (Lidar_state_group[j].ang_vel.norm() - mean_LiDAR_ang_vel);  // Zero-centered cross correlation
            }
        }

        if (corr > max_corr) {
            max_corr = corr;
            lag_IMU_wtr_Lidar = -lag;
        }
    }

    time_lag_1 = lag_IMU_wtr_Lidar / odom_freq;
    cout << "Max Cross-correlation: IMU lag wtr Lidar : " << -lag_IMU_wtr_Lidar << endl;
    cout << "Time lag 1: IMU lag wtr Lidar : " << time_lag_1 << endl;
}

void Gril_Calib::IMU_time_compensate(const double &lag_time, const bool &is_discard) {
    if (is_discard) {
        // Discard first 10 Lidar estimations and corresponding IMU measurements due to long time interval
        int i = 0;
        while (i < 10) {
            Lidar_state_group.pop_front();
            IMU_state_group.pop_front();
            i++;
        }
    }

    auto it_IMU_state = IMU_state_group.begin();
    for (; it_IMU_state != IMU_state_group.end() - 1; it_IMU_state++) {
        it_IMU_state->timeStamp = it_IMU_state->timeStamp - lag_time;
    }

    while (Lidar_state_group.front().timeStamp < IMU_state_group.front().timeStamp)
        Lidar_state_group.pop_front();

    while (Lidar_state_group.front().timeStamp > IMU_state_group[1].timeStamp)
        IMU_state_group.pop_front();

    // align the size of two sequences
    while (IMU_state_group.size() > Lidar_state_group.size())
        IMU_state_group.pop_back();
    while (IMU_state_group.size() < Lidar_state_group.size())
        Lidar_state_group.pop_back();
}


void Gril_Calib::cut_sequence_tail() {
    // The original offline implementation always removed the last 20 samples.
    // For live calibration that can discard nearly all data when Ctrl+C arrives
    // early, so only retain that conservative trimming for sufficiently long runs.
    if (Lidar_state_group.size() > 80 && IMU_state_group.size() > 80) {
        for (int i = 0; i < 20; ++i) {
            Lidar_state_group.pop_back();
            IMU_state_group.pop_back();
        }
    }
    if (Lidar_state_group.size() < 2 || IMU_state_group.size() < 2) return;
    while (Lidar_state_group.size() > 1 && IMU_state_group.size() > 1 &&
           Lidar_state_group.front().timeStamp < IMU_state_group.front().timeStamp)
        Lidar_state_group.pop_front();
    while (Lidar_state_group.size() > 1 && IMU_state_group.size() > 1 &&
           Lidar_state_group.front().timeStamp > IMU_state_group[1].timeStamp)
        IMU_state_group.pop_front();
    while (IMU_state_group.size() > Lidar_state_group.size()) IMU_state_group.pop_back();
    while (IMU_state_group.size() < Lidar_state_group.size()) Lidar_state_group.pop_back();
}

void Gril_Calib::acc_interpolate() {
    //Interpolation to get acc_I(t_L)
    for (int i = 1; i < Lidar_state_group.size() - 1; i++) {
        double deltaT = Lidar_state_group[i].timeStamp - IMU_state_group[i].timeStamp;
        if (deltaT > 0) {
            double DeltaT = IMU_state_group[i + 1].timeStamp - IMU_state_group[i].timeStamp;
            double s = deltaT / DeltaT;
            IMU_state_group[i].linear_acc = s * IMU_state_group[i + 1].linear_acc +
                                            (1 - s) * IMU_state_group[i].linear_acc;
            IMU_state_group[i].timeStamp += deltaT;
        } else {
            double DeltaT = IMU_state_group[i].timeStamp - IMU_state_group[i - 1].timeStamp;
            double s = -deltaT / DeltaT;
            IMU_state_group[i].linear_acc = s * IMU_state_group[i - 1].linear_acc +
                                            (1 - s) * IMU_state_group[i].linear_acc;
            IMU_state_group[i].timeStamp += deltaT;
        }
    }
}

// Butterworth filter (Low-pass filter)
void Gril_Calib::Butter_filt(const deque<CalibState> &signal_in, deque<CalibState> &signal_out) {
    Gril_Calib::Butterworth butter;
    butter.extend_num = 10 * (butter.Coeff_size - 1);
    auto it_front = signal_in.begin() + butter.extend_num;
    auto it_back = signal_in.end() - 1 - butter.extend_num;

    deque<CalibState> extend_front;
    deque<CalibState> extend_back;

    for (int idx = 0; idx < butter.extend_num; idx++) {
        extend_front.push_back(*it_front);
        extend_back.push_front(*it_back);
        it_front--;
        it_back++;
    }

    deque<CalibState> sig_extended(signal_in);
    while (!extend_front.empty()) {
        sig_extended.push_front(extend_front.back());
        extend_front.pop_back();
    }
    while (!extend_back.empty()) {
        sig_extended.push_back(extend_back.front());
        extend_back.pop_front();
    }

    deque<CalibState> sig_out(sig_extended);
    // One-direction Butterworth filter Starts (all states)
    for (int i = butter.Coeff_size; i < sig_extended.size() - butter.extend_num; i++) {
        CalibState temp_state;
        for (int j = 0; j < butter.Coeff_size; j++) {
            auto it_sig_ext = *(sig_extended.begin() + i - j);
            temp_state += it_sig_ext * butter.Coeff_b[j];
        }
        for (int jj = 1; jj < butter.Coeff_size; jj++) {
            auto it_sig_out = *(sig_out.begin() + i - jj);
            temp_state -= it_sig_out * butter.Coeff_a[jj];
        }
        sig_out[i] = temp_state;
    }

    for (auto it = sig_out.begin() + butter.extend_num; it != sig_out.end() - butter.extend_num; it++) {
        signal_out.push_back(*it);
    }
}

// zero phase low-pass filter //
void Gril_Calib::zero_phase_filt(const deque<CalibState> &signal_in, deque<CalibState> &signal_out) {
    // A 6th-order forward/backward filter needs a substantial boundary region.
    // Do not manufacture boundary data for a short live session; preserve the
    // measurements so Ctrl+C can still produce a provisional calibration.
    if (signal_in.size() < 80) {
        signal_out = signal_in;
        return;
    }
    deque<CalibState> sig_out1;
    Butter_filt(signal_in, sig_out1);   // signal_in에 대해 Butterworth filter를 적용한 결과를 sig_out1에 저장

    deque<CalibState> sig_rev(sig_out1);
    reverse(sig_rev.begin(), sig_rev.end()); //Reverse the elements

    Butter_filt(sig_rev, signal_out);
    reverse(signal_out.begin(), signal_out.end()); //Reverse the elements
}


// To obtain a rough initial value of rotation matrix //
void Gril_Calib::solve_Rotation_only() {
    double R_LI_quat[4];

    R_LI_quat[0] = 1;
    R_LI_quat[1] = 0;
    R_LI_quat[2] = 0;
    R_LI_quat[3] = 0;

    ceres::Manifold *quatParam = new ceres::QuaternionManifold();
    ceres::Problem problem_rot;
    problem_rot.AddParameterBlock(R_LI_quat, 4, quatParam);

    for (int i = 0; i < IMU_state_group.size(); i++) {
        M3D Lidar_angvel_skew;
        Lidar_angvel_skew << SKEW_SYM_MATRX(Lidar_state_group[i].ang_vel);
        problem_rot.AddResidualBlock(Angular_Vel_Cost_only_Rot::Create(IMU_state_group[i].ang_vel,
                                                                       Lidar_state_group[i].ang_vel),
                                     nullptr,
                                     R_LI_quat);
    }

    ceres::Solver::Options options_quat;
    ceres::Solver::Summary summary_quat;
    ceres::Solve(options_quat, &problem_rot, &summary_quat);
    Eigen::Quaterniond q_LI(R_LI_quat[0], R_LI_quat[1], R_LI_quat[2], R_LI_quat[3]);
    Rot_Lidar_wrt_IMU = q_LI.matrix();  // LiDAR angulr velocity in IMU frame (from LiDAR to IMU)

}



// Proposed algorithm (Rotation + Translation)
void Gril_Calib::solve_Rot_Trans_calib(double &timediff_imu_wrt_lidar, const double &imu_sensor_height) {

    M3D R_IL_init = Rot_Lidar_wrt_IMU.transpose(); // Initial value of Rotation of IL (from IMU frame to LiDAR frame)
    Eigen::Quaterniond quat(R_IL_init);
    double R_IL_quat[4];
    R_IL_quat[0] = quat.w();
    R_IL_quat[1] = quat.x();
    R_IL_quat[2] = quat.y();
    R_IL_quat[3] = quat.z();

    double Trans_IL[3];             // Initial value of Translation of IL (IMU with respect to Lidar) - ceres solver input
    Trans_IL[0] = trans_IL_x;
    Trans_IL[1] = trans_IL_y;
    Trans_IL[2] = trans_IL_z;

    double bias_g[3];               // Initial value of gyro bias
    bias_g[0] = 0;
    bias_g[1] = 0;
    bias_g[2] = 0;

    double bias_aL[3];              // Initial value of acc bias
    bias_aL[0] = 0;
    bias_aL[1] = 0;
    bias_aL[2] = 0;

    double time_lag2 = 0;           // Second time lag (IMU wtr Lidar)

    // Define problem
    ceres::Problem problem_Ex_calib;
    ceres::Manifold *quatParam = new ceres::QuaternionManifold();

    // Define Loss function
    ceres::LossFunction *loss_function_angular = new ceres::CauchyLoss(0.5);
    ceres::ScaledLoss *loss_function_angular_scaled = new ceres::ScaledLoss(loss_function_angular, 0.5, ceres::TAKE_OWNERSHIP);

    ceres::LossFunction *loss_function_acc = new ceres::CauchyLoss(0.5);
    ceres::ScaledLoss *scaled_loss_acc = new ceres::ScaledLoss(loss_function_acc, 0.2, ceres::TAKE_OWNERSHIP);

    ceres::LossFunction *loss_function_plain_motion = new ceres::HuberLoss(0.5);
    ceres::ScaledLoss *loss_function_plain_motion_scaled = new ceres::ScaledLoss(loss_function_plain_motion, 0.3, ceres::TAKE_OWNERSHIP);

    // Add Parameter Block
    problem_Ex_calib.AddParameterBlock(R_IL_quat, 4, quatParam);
    problem_Ex_calib.AddParameterBlock(Trans_IL, 3);
    problem_Ex_calib.AddParameterBlock(bias_g, 3);
    problem_Ex_calib.AddParameterBlock(bias_aL, 3);

    // Use only samples available in every residual stream.
    const size_t sample_count = std::min(
            std::min(IMU_state_group.size(), Lidar_state_group.size()),
            std::min(Lidar_wrt_ground_group.size(),
                     std::min(IMU_wrt_ground_group.size(),
                              distance_Lidar_wrt_ground_group.size())));

    //Jacobian of acc_bias, gravity, Translation
    int Jaco_size = 3 * static_cast<int>(sample_count);
    MatrixXd Jacobian(Jaco_size, 9);
    Jacobian.setZero();

    // Jacobian of Translation
    MatrixXd Jaco_Trans(Jaco_size, 3);
    Jaco_Trans.setZero();

    // Add Residual Block
    for (size_t i = 0; i < sample_count; i++) {
        double deltaT = Lidar_state_group[i].timeStamp - IMU_state_group[i].timeStamp;

        problem_Ex_calib.AddResidualBlock(Ground_Plane_Cost_IL::Create(Lidar_wrt_ground_group[i],
                                                                      IMU_wrt_ground_group[i],
                                                                      distance_Lidar_wrt_ground_group[i],
                                                                      imu_sensor_height),
                                                            loss_function_plain_motion_scaled,
                                                        R_IL_quat,
                                                        Trans_IL);

        problem_Ex_calib.AddResidualBlock(Angular_Vel_IL_Cost::Create(IMU_state_group[i].ang_vel,
                                                                  IMU_state_group[i].ang_acc,
                                                                  Lidar_state_group[i].ang_vel,
                                                                  deltaT),
                                            loss_function_angular_scaled,
                                         R_IL_quat,
                                         bias_g,
                                         &time_lag2);

        problem_Ex_calib.AddResidualBlock(Linear_acc_Rot_Cost_without_Gravity::Create(Lidar_state_group[i],
                                                                                      IMU_state_group[i].linear_acc,
                                                                                      Lidar_wrt_ground_group[i]),
                                                                                scaled_loss_acc,
                                                                            R_IL_quat,
                                                                            bias_aL,
                                                                            Trans_IL);

        Jacobian.block<3, 3>(3 * i, 0) = -Lidar_state_group[i].rot_end;
        Jacobian.block<3, 3>(3 * i, 3) << SKEW_SYM_MATRX(STD_GRAV);
        M3D omg_skew, angacc_skew;
        omg_skew << SKEW_SYM_MATRX(Lidar_state_group[i].ang_vel);
        angacc_skew << SKEW_SYM_MATRX(Lidar_state_group[i].ang_acc);
        M3D Jaco_trans_i = -omg_skew * omg_skew - angacc_skew;
        Jaco_Trans.block<3, 3>(3 * i, 0) = Jaco_trans_i;
        Jacobian.block<3, 3>(3 * i, 6) = Jaco_trans_i;
    }

    // Set boundary
    for (int index = 0; index < 3; ++index) {
        problem_Ex_calib.SetParameterUpperBound(bias_aL, index, 0.01);
        problem_Ex_calib.SetParameterLowerBound(bias_aL, index, -0.01);

        problem_Ex_calib.SetParameterUpperBound(bias_g, index, 0.01);
        problem_Ex_calib.SetParameterLowerBound(bias_g, index, -0.01);
    }

    if(set_boundary) {
        for (int index = 0; index < 3; ++index) {
            problem_Ex_calib.SetParameterUpperBound(Trans_IL, index, Trans_IL[index] + bound_th);
            problem_Ex_calib.SetParameterLowerBound(Trans_IL, index, Trans_IL[index] - bound_th);
        }
    }

    // Solver options
    ceres::Solver::Options options_Ex_calib;
    options_Ex_calib.num_threads = 1;
    options_Ex_calib.use_explicit_schur_complement = true;
    options_Ex_calib.linear_solver_type = ceres::ITERATIVE_SCHUR;
    options_Ex_calib.preconditioner_type = ceres::SCHUR_JACOBI;
    options_Ex_calib.minimizer_progress_to_stdout = false;

    // Solve
    ceres::Solver::Summary summary_Ex_calib;
    ceres::Solve(options_Ex_calib, &problem_Ex_calib, &summary_Ex_calib);

    // std::cout << summary_Ex_calib.FullReport() << "\n";

    //** Update the result **//

    // Rotation matrix
    Eigen::Quaterniond q_IL_final(R_IL_quat[0], R_IL_quat[1], R_IL_quat[2], R_IL_quat[3]);  // quaternion from IMU frame to Lidar frame
    Rot_Lidar_wrt_IMU = q_IL_final.matrix().transpose();
    V3D euler_angle = RotMtoEuler(Rot_Lidar_wrt_IMU);

    // Translation vector
    V3D Trans_IL_vec(Trans_IL[0], Trans_IL[1], Trans_IL[2]);
    Trans_Lidar_wrt_IMU = -1.0 * Rot_Lidar_wrt_IMU * Trans_IL_vec;

    // gravity vector - not used
    M3D R_WLO = Lidar_wrt_ground_group[0].matrix();
    Grav_L0 = R_WLO * STD_GRAV;   // gravity in first lidar frame

    // bias acc
    V3D bias_a_Lidar(bias_aL[0], bias_aL[1], bias_aL[2]);
    acc_bias = Rot_Lidar_wrt_IMU * bias_a_Lidar;

    // bias gyro
    gyro_bias = V3D(bias_g[0], bias_g[1], bias_g[2]);

    // time offset
    time_lag_2 = time_lag2;
    time_delay_IMU_wtr_Lidar = time_lag_1 + time_lag_2;

    time_offset_result = time_delay_IMU_wtr_Lidar;

    //The second temporal compensation
    IMU_time_compensate(get_lag_time_2(), false);

    // For debug
    for (size_t i = 0; i < sample_count; i++) {
        M3D R_GL = Lidar_wrt_ground_group[i].matrix();
        V3D Grav_L = R_GL * STD_GRAV;

        V3D acc_I = Lidar_state_group[i].rot_end * Rot_Lidar_wrt_IMU.transpose() * IMU_state_group[i].linear_acc -
                    Lidar_state_group[i].rot_end * bias_a_Lidar;
        V3D acc_L = Lidar_state_group[i].linear_acc +
                    Lidar_state_group[i].rot_end * Jaco_Trans.block<3, 3>(3 * i, 0) * Trans_IL_vec - Grav_L;
        fout_acc_cost << setprecision(10) << acc_I.transpose() << " " << acc_L.transpose() << " "
                      << IMU_state_group[i].timeStamp << " " << Lidar_state_group[i].timeStamp << endl;
    }
}

void Gril_Calib::normalize_acc(deque<CalibState> &signal_in) {
    V3D mean_acc(0, 0, 0);

    for (int i = 1; i < 10; i++) {
        mean_acc += (signal_in[i].linear_acc - mean_acc) / i;
    }

    for (int i = 0; i < signal_in.size(); i++) {
        signal_in[i].linear_acc = signal_in[i].linear_acc / mean_acc.norm() * G_m_s2;
    }
}

// Align the size of various states
void Gril_Calib::align_Group(const deque<CalibState> &IMU_states, deque<Eigen::Quaterniond> &Lidar_wrt_ground_states,
                     deque<Eigen::Quaterniond> &IMU_wrt_ground_states, deque<V3D> &normal_vector_wrt_lidar_group, deque<double> &distance_Lidar_ground_states) {

    // Align the size of two sequences
    while(IMU_states.size() < Lidar_wrt_ground_states.size()) {
        Lidar_wrt_ground_states.pop_back();
        IMU_wrt_ground_states.pop_back();
        normal_vector_wrt_lidar_group.pop_back();
        distance_Lidar_ground_states.pop_back();
    }

}

bool Gril_Calib::data_sufficiency_assess(MatrixXd &Jacobian_rot, int &frame_num, V3D &lidar_omg, int &orig_odom_freq,
                                      int &cut_frame_num, QD &lidar_q, QD &imu_q, double &lidar_estimate_height) {
    //Calculation of Rotation Jacobian
    M3D lidar_omg_skew;
    lidar_omg_skew << SKEW_SYM_MATRX(lidar_omg);
    Jacobian_rot.block<3, 3>(3 * frame_num, 0) = lidar_omg_skew;
    bool data_sufficient = false;

    //Give a Data Appraisal every second
    if (frame_num % (orig_odom_freq * cut_frame_num) == 0) {
        M3D Hessian_rot = Jacobian_rot.transpose() * Jacobian_rot;
        EigenSolver<M3D> es(Hessian_rot);
        V3D EigenValue = es.eigenvalues().real();
        M3D EigenVec_mat = es.eigenvectors().real();

        M3D EigenMatCwise = EigenVec_mat.cwiseProduct(EigenVec_mat);
        std::vector<double> EigenMat_1_col{EigenMatCwise(0, 0), EigenMatCwise(1, 0), EigenMatCwise(2, 0)};
        std::vector<double> EigenMat_2_col{EigenMatCwise(0, 1), EigenMatCwise(1, 1), EigenMatCwise(2, 1)};
        std::vector<double> EigenMat_3_col{EigenMatCwise(0, 2), EigenMatCwise(1, 2), EigenMatCwise(2, 2)};

        // Find the maximum value of each column
        int maxPos[3] = {0};
        maxPos[0] = max_element(EigenMat_1_col.begin(), EigenMat_1_col.end()) - EigenMat_1_col.begin();
        maxPos[1] = max_element(EigenMat_2_col.begin(), EigenMat_2_col.end()) - EigenMat_2_col.begin();
        maxPos[2] = max_element(EigenMat_3_col.begin(), EigenMat_3_col.end()) - EigenMat_3_col.begin();

        V3D Scaled_Eigen = EigenValue / data_accum_length;   // the larger data_accum_length is, the more data is needed
        V3D Rot_percent(Scaled_Eigen[1] * Scaled_Eigen[2],
                        Scaled_Eigen[0] * Scaled_Eigen[2],
                        Scaled_Eigen[0] * Scaled_Eigen[1]);

        int axis[3];
        axis[2] = max_element(maxPos, maxPos + 3) - maxPos;
        axis[0] = min_element(maxPos, maxPos + 3) - maxPos;
        axis[1] = 3 - (axis[0] + axis[2]);

        double percentage_x = Rot_percent[axis[0]] < x_accumulate ? Rot_percent[axis[0]] : 1;
        double percentage_y = Rot_percent[axis[1]] < y_accumulate ? Rot_percent[axis[1]] : 1;
        double percentage_z = Rot_percent[axis[2]] < z_accumulate ? Rot_percent[axis[2]] : 1;

        clear(); //clear the screen
        printf("\033[3A\r");

        printProgress(percentage_x, 88);
        printProgress(percentage_y, 89);
        printProgress(percentage_z, 90);

        if(verbose){
            M3D R_GL = lidar_q.toRotationMatrix();
            M3D R_GI = imu_q.toRotationMatrix();

            printf(BOLDREDPURPLE "[Rotation matrix Ground to LiDAR (euler)] " RESET);
            cout << setprecision(4) << RotMtoEuler(R_GL).transpose() * 57.3 << " deg" << '\n';

            printf(BOLDREDPURPLE "[Rotation matrix Ground to IMU (euler)] " RESET);
            cout << setprecision(4) << RotMtoEuler(R_GI).transpose() * 57.3 << " deg" << '\n';

            printf(BOLDREDPURPLE "[Estimated LiDAR sensor height] : " RESET);
            cout << setprecision(4) << lidar_estimate_height << " m" << '\n';
        }


        fflush(stdout);
        if (Rot_percent[axis[0]] > x_accumulate && Rot_percent[axis[1]] > y_accumulate && Rot_percent[axis[2]] > z_accumulate) {
            printf(BOLDCYAN "[calibration] Data accumulation finished, Lidar IMU calibration begins.\n\n" RESET);
            printf(BOLDBLUE"============================================================ \n\n" RESET);
            data_sufficient = true;
        }
    }

    if (data_sufficient)
        return true;
    else
        return false;
}


void Gril_Calib::printProgress(double percentage, int axis_ascii) {
    int val = (int) (percentage * 100);
    int lpad = (int) (percentage * PBWIDTH);
    int rpad = PBWIDTH - lpad;
    printf(BOLDCYAN "[Data accumulation] ");
    if (percentage < 1) {
        printf(BOLDYELLOW "Rotation around Lidar %c Axis: ", char(axis_ascii));
        printf(YELLOW "%3d%% [%.*s%*s]\n", val, lpad, PBSTR, rpad, "");
        cout << RESET;
    } else {
        printf(BOLDGREEN "Rotation around Lidar %c Axis complete! ", char(axis_ascii));
        cout << RESET << "\n";
    }
}

void Gril_Calib::clear() {
    // CSI[2J clears screen, CSI[H moves the cursor to top-left corner
    cout << "\x1B[2J\x1B[H";
}

//** main function in LiDAR IMU calibration **//
void Gril_Calib::LI_Calibration(int &orig_odom_freq, int &cut_frame_num, double &timediff_imu_wrt_lidar,
                                const double &move_start_time) {

    TimeConsuming time("Batch optimization");

    if (Lidar_state_group.size() < 8 || IMU_state_group_ALL.size() < 8) {
        throw std::runtime_error("need at least 8 LiDAR poses and 8 IMU samples for calibration");
    }
    downsample_interpolate_IMU(move_start_time);
    if (IMU_state_group.size() < 3 || Lidar_state_group.size() < 3) {
        throw std::runtime_error("insufficient temporally overlapping LiDAR/IMU data");
    }
    fout_before_filter();
    IMU_time_compensate(0.0, true);

    deque<CalibState> IMU_after_zero_phase;
    zero_phase_filt(get_IMU_state(), IMU_after_zero_phase); // zero phase low-pass filter
    normalize_acc(IMU_after_zero_phase);
    set_IMU_state(IMU_after_zero_phase);
    cut_sequence_tail();

    // Fit the actual LiDAR pose sequence before temporal initialization.
    // state.bias_g/state.vel_end are CV-model states and are not differentiated.
    const bool lidar_fit_ok = bspline_enable && bspline_fit_lidar_kinematics();
    if (!lidar_fit_ok) {
        std::cerr << "[B-spline] Falling back to legacy LiDAR central differences." << std::endl;
    }

    xcorr_temporal_init(orig_odom_freq * cut_frame_num);
    IMU_time_compensate(get_lag_time_1(), false);

    // IMU angular acceleration remains needed by the unified time-offset
    // residual. Preserve B-spline LiDAR kinematics when the fit succeeded.
    central_diff(!lidar_fit_ok);

    deque<CalibState> IMU_after_2nd_zero_phase;
    zero_phase_filt(get_IMU_state(), IMU_after_2nd_zero_phase);
    set_IMU_state(IMU_after_2nd_zero_phase);
    fout_check_lidar(); // file output for visualizing fitted LiDAR kinematics


    solve_Rotation_only();
    acc_interpolate();
    align_Group(IMU_state_group, Lidar_wrt_ground_group, IMU_wrt_ground_group,
                normal_vector_wrt_lidar_group, distance_Lidar_wrt_ground_group);

    // Calibration at once
    solve_Rot_Trans_calib(timediff_imu_wrt_lidar, imu_sensor_height);

    printf(BOLDBLUE"============================================================ \n\n" RESET);
    double time_L_I = timediff_imu_wrt_lidar + time_delay_IMU_wtr_Lidar;
    print_calibration_result(time_L_I, Rot_Lidar_wrt_IMU, Trans_Lidar_wrt_IMU, gyro_bias, acc_bias, Grav_L0);

    printf(BOLDBLUE"============================================================ \n\n" RESET);
    printf(BOLDCYAN "Gril-Calib : Ground Robot IMU-LiDAR calibration done.\n");
    printf("" RESET);

    // For debug
    // plot_result();
}

void Gril_Calib::print_calibration_result(double &time_L_I, M3D &R_L_I, V3D &p_L_I, V3D &bias_g, V3D &bias_a, V3D gravity) {
    cout.setf(ios::fixed);
    printf(BOLDCYAN "[Calibration Result] " RESET);
    cout << setprecision(6)
         << "Rotation matrix from LiDAR frame to IMU frame    = " << RotMtoEuler(R_L_I).transpose() * 57.3 << " deg" << endl;
    printf(BOLDCYAN "[Calibration Result] " RESET);
    cout << "Translation vector from LiDAR frame to IMU frame = " << p_L_I.transpose() << " m" << endl;
    printf(BOLDCYAN "[Calibration Result] " RESET);
    printf("Time Lag IMU to LiDAR    = %.8lf s \n", time_L_I);
    printf(BOLDCYAN "[Calibration Result] " RESET);
    cout << "Bias of Gyroscope        = " << bias_g.transpose() << " rad/s" << endl;
    printf(BOLDCYAN "[Calibration Result] " RESET);
    cout << "Bias of Accelerometer    = " << bias_a.transpose() << " m/s^2" << endl;
}

void Gril_Calib::plot_result() {}
