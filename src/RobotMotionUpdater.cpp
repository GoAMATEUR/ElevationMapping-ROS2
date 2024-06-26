#include <elevation_mapping_ros2/RobotMotionUpdater.hpp>

namespace elevation_mapping
{
RobotMotionUpdater::RobotMotionUpdater()
{
    clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    pre_reduced_covariance_.setZero();
    pre_update_time_ = clock_->now();
}

RobotMotionUpdater::~RobotMotionUpdater()
{}


// 
bool RobotMotionUpdater::update(ElevationMap& _map, const PoseTransform& _pose_transform, const PoseCovariance& _pose_covariance, const rclcpp::Time& _time_stamp)
{
    if (pre_update_time_ == _time_stamp) return false;

    // initialize update data
    grid_map::Size size = _map.getRawMap().getSize();
    grid_map::Matrix variance_update(size(0), size(1));
    grid_map::Matrix horizontal_variance_update_x(size(0), size(1));
    grid_map::Matrix horizontal_variance_update_y(size(0), size(1));
    grid_map::Matrix horizontal_variance_update_xy(size(0), size(1));

    // Relative covariance matrix between two robot poses
    ReducedCovariance reduced_covariance; // sigma
    computeReducedCovariance(_pose_transform, _pose_covariance, reduced_covariance);
    ReducedCovariance relative_convariance; // sigma B_tildeB
    computeRelativeCovariance(_pose_transform, reduced_covariance, relative_convariance);

    Eigen::Matrix3d position_covariance = relative_convariance.topLeftCorner(3,3);
    Eigen::Matrix3d rotation_covariance = Eigen::Matrix3d::Zero();
    rotation_covariance(2,2) = relative_convariance(3,3);

    // Eigen::Matrix3d map2robot_rotation = _pose_transform.rotation().transpose()*_map.getPose().rotation(); // -C(\phi_tildeBM)
    Eigen::Matrix3d map2robot_rotation = _pose_transform.rotation().transpose(); // -C(\phi_tildeBM)
    Eigen::Matrix3d map2pre_robot_rotation_inverse = (map2robot_rotation).transpose();

    Eigen::Matrix3d translation_jac = -map2robot_rotation.transpose(); // J_r
    Eigen::Vector3f translation_variance_update = (translation_jac*position_covariance*translation_jac.transpose()).diagonal().cast<float>();

    // map-robot relative position
    const Eigen::Vector3d position_robot2map = -pre_robot_pose_.translation();

    auto& height_layer = _map.getRawMap()["elevation"];

    for (size_t i=0; i<static_cast<size_t>(size(0)); ++i)
    {
        for (size_t j=0; j<static_cast<size_t>(size(1)); ++j)
        {
            Eigen::Vector3d cell_position;

            const auto height = height_layer(i, j);
            if (std::isfinite(height))
            {
                grid_map::Position position;
                _map.getRawMap().getPosition({i, j}, position);
                cell_position.x() = position.x();
                cell_position.y() = position.y();
                cell_position.z() = height;

                const Eigen::Matrix3d rotation_jac = -computeSkewMatrixfromVector(position_robot2map + cell_position)*map2pre_robot_rotation_inverse;
                
                // rotation variance update
                const Eigen::Matrix2f rotation_variance_update = (rotation_jac*rotation_covariance*rotation_jac.transpose()).topLeftCorner<2,2>().cast<float>();

                // variance update
                variance_update(i, j) = translation_variance_update.z();
                horizontal_variance_update_x(i, j) = translation_variance_update.x() + rotation_variance_update(0, 0);
                horizontal_variance_update_y(i, j) = translation_variance_update.y() + rotation_variance_update(1, 1);
                horizontal_variance_update_xy(i, j) = rotation_variance_update(0, 1);
            }
            else 
            {
                variance_update(i, j) = std::numeric_limits<float>::infinity();
                horizontal_variance_update_x(i, j) = std::numeric_limits<float>::infinity();
                horizontal_variance_update_y(i, j) = std::numeric_limits<float>::infinity();
                horizontal_variance_update_xy(i, j) = std::numeric_limits<float>::infinity();
            }
        }
    }

    _map.update(variance_update, horizontal_variance_update_x, horizontal_variance_update_y, horizontal_variance_update_xy, _time_stamp);
    pre_reduced_covariance_ = reduced_covariance;
    pre_robot_pose_ = _pose_transform;

    return true;
}

bool RobotMotionUpdater::computeReducedCovariance(const PoseTransform& _pose_transform, const PoseCovariance& _pose_covariance, ReducedCovariance& _reduced_covariance)
{
    Eigen::Vector3d euler_zyx = _pose_transform.rotation().eulerAngles(2, 1, 0);
    double tan_pitch = std::tan(euler_zyx(1));

    Eigen::Matrix<double, 1, 3> yaw_jac(std::cos(euler_zyx(0))*tan_pitch, std::sin(euler_zyx(0))*tan_pitch, 1.0);
    Eigen::Matrix<double, 4, 6> jac;
    jac.setZero();
    jac.topLeftCorner(3,3).setIdentity();
    jac.bottomRightCorner(1,3) = yaw_jac;

    _reduced_covariance = jac*_pose_covariance*jac.transpose();
    return true;
}

bool RobotMotionUpdater::computeRelativeCovariance(const PoseTransform& _pose_transform, const ReducedCovariance& _reduced_covariance, ReducedCovariance& _relative_covariance)
{
    Eigen::Vector3d euler_zyx = _pose_transform.rotation().eulerAngles(2,1,0);
    // euler_zyx(1) = 0.0;
    // euler_zyx(2) = 0.0;
    Eigen::Matrix3d R_I2tildeB = Eigen::AngleAxisd(euler_zyx(0), Eigen::Vector3d::UnitZ()).matrix();

    // velocity*delta_t
    Eigen::Vector3d v_delta_t = pre_robot_pose_.rotation().transpose()*(_pose_transform.translation() - pre_robot_pose_.translation());    

    Eigen::Matrix<double, 4, 4> f; // F
    f.setIdentity();
    f.topRightCorner(3, 1) = computeSkewMatrixfromVector(Eigen::Vector3d(0.0, 0.0, 1.0)) * R_I2tildeB*v_delta_t;

    // jacobina inv(G)*delta_t
    Eigen::Matrix<double, 4, 4> inv_g_delta_t;
    inv_g_delta_t.setZero();
    inv_g_delta_t(3,3) = 0.0;
    Eigen::Matrix<double, 4, 4> inv_g_transpose_delta_t(inv_g_delta_t);
    inv_g_delta_t.topLeftCorner(3, 3) = R_I2tildeB.transpose();
    inv_g_transpose_delta_t.topLeftCorner(3,3) = R_I2tildeB;

    _relative_covariance = inv_g_delta_t*(_reduced_covariance - f*pre_reduced_covariance_*f.transpose())*inv_g_transpose_delta_t;

    return true;
}

Eigen::Matrix3d RobotMotionUpdater::computeSkewMatrixfromVector(const Eigen::Vector3d& _vec)
{
    Eigen::Matrix3d mat;
    mat.setZero();
    mat(0, 1) =-_vec.z();
    mat(0, 2) = _vec.y();
    mat(1, 0) = _vec.z();
    mat(1, 2) =-_vec.x();
    mat(2, 0) =-_vec.y();
    mat(2, 1) = _vec.z();

    return mat;
}

void RobotMotionUpdater::readParameters(rclcpp::Node* _node)
{
    logger_name_ = _node->declare_parameter("robot_motion_updater.logger_name", "RobotMotionUpdater");
}


}