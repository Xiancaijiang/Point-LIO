#ifndef COMMON_LIB_H  
#define COMMON_LIB_H  

#include <pcl/point_cloud.h>                   // for pcl::PointCloud
#include <pcl/point_types.h>  
#include <so3_math.h>  
#include <tf2_ros/transform_broadcaster.h>     // ROS TF2变换广播器

#include <../include/IKFoM/IKFoM_toolkit/esekfom/esekfom.hpp>  
#include <Eigen/Eigen>                         // Eigen库
#include <nav_msgs/msg/odometry.hpp>  
#include <queue>  
#include <sensor_msgs/msg/imu.hpp>  

using namespace std;  
using namespace Eigen;  

// 定义三维向量类型  
typedef MTK::vect<3, double> vect3;  
// 定义SO(3)旋转群类型  
typedef MTK::SO3<double> SO3;  
// 定义S2球面类型  
typedef MTK::S2<double, 98090, 10000, 1> S2;  
// 定义一维向量类型  
typedef MTK::vect<1, double> vect1;  
// 定义二维向量类型  
typedef MTK::vect<2, double> vect2;  

// 定义状态输入的流形  
MTK_BUILD_MANIFOLD(  
  state_input,   
  ((vect3, pos)) // 位置  
  ((SO3, rot)) // 旋转  
  ((SO3, offset_R_L_I)) // 左右相机之间的旋转偏移  
  ((vect3, offset_T_L_I)) // 左右相机之间的平移偏移  
  ((vect3, vel)) // 速度  
  ((vect3, bg)) // 陀螺仪偏置  
  ((vect3, ba)) // 加速度计偏置  
  ((vect3, gravity))); // 重力  

// 定义状态输出的流形  
MTK_BUILD_MANIFOLD(  
  state_output,  
  ((vect3, pos)) // 位置  
  ((SO3, rot)) // 旋转  
  ((SO3, offset_R_L_I)) // 左右相机之间的旋转偏移  
  ((vect3, offset_T_L_I)) // 左右相机之间的平移偏移  
  ((vect3, vel)) // 速度  
  ((vect3, omg)) // 角速度  
  ((vect3, acc)) // 加速度  
  ((vect3, gravity)) // 重力  
  ((vect3, bg)) // 陀螺仪偏置  
  ((vect3, ba))); // 加速度计偏置  

// 定义输入流形  
MTK_BUILD_MANIFOLD(input_ikfom, ((vect3, acc))((vect3, gyro)));  

// 定义过程噪声输入的流形  
MTK_BUILD_MANIFOLD(process_noise_input, ((vect3, ng))((vect3, na))((vect3, nbg))((vect3, nba)));  

// 定义过程噪声输出的流形  
MTK_BUILD_MANIFOLD(  
  process_noise_output,   
  ((vect3, vel)) // 速度噪声  
  ((vect3, ng)) // 陀螺仪噪声  
  ((vect3, na)) // 加速度计噪声  
  ((vect3, nbg)) // 陀螺仪偏置噪声  
  ((vect3, nba))); // 加速度计偏置噪声  

// 声明扩展卡尔曼滤波器实例  
extern esekfom::esekf<state_input, 24, input_ikfom> kf_input;  
extern esekfom::esekf<state_output, 30, input_ikfom> kf_output;  

// 进度条宽度和字符定义  
#define PBWIDTH 30  
#define PBSTR "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"  

// 定义常数  
#define PI_M (3.14159265358) // 圆周率  
// #define G_m_s2 (9.81)         // 广东/中国的重力常数  
#define DIM_STATE (24)   // 状态维度（SO(3)维度为3）  
#define DIM_PROC_N (12)  // 过程噪声维度（SO(3)维度为3）  
#define CUBE_LEN (6.0) // 立方体长度  
#define LIDAR_SP_LEN (2) // 激光雷达点间隔长度  
#define INIT_COV (0.0001) // 初始协方差  
#define NUM_MATCH_POINTS (5) // 匹配点数量  
#define MAX_MEAS_DIM (10000) // 最大测量维度  

// 将数组转换为向量的宏定义  
#define VEC_FROM_ARRAY(v) v[0], v[1], v[2]  
#define VEC_FROM_ARRAY_SIX(v) v[0], v[1], v[2], v[3], v[4], v[5]  
#define MAT_FROM_ARRAY(v) v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]  
// 限制值范围的宏定义  
#define CONSTRAIN(v, min, max) ((v > min) ? ((v < max) ? v : max) : min)  
// 将Eigen矩阵转换为数组的宏定义  
#define ARRAY_FROM_EIGEN(mat) mat.data(), mat.data() + mat.rows() * mat.cols()  
// 将Eigen矩阵转换为std::vector的宏定义  
#define STD_VEC_FROM_EIGEN(mat) \
  vector<decltype(mat)::Scalar>(mat.data(), mat.data() + mat.rows() * mat.cols())  
// 日志文件路径宏定义  
#define DEBUG_FILE_DIR(name) (string(string(ROOT_DIR) + "Log/" + name))  

// 点云类型定义  
typedef pcl::PointXYZINormal PointType; // 带法线的点类型  
typedef pcl::PointXYZRGB PointTypeRGB;   // 带颜色的点类型  
typedef pcl::PointCloud<PointType> PointCloudXYZI; // 点云类型，包含法线  
typedef pcl::PointCloud<PointTypeRGB> PointCloudXYZRGB; // 点云类型，包含颜色  
typedef vector<PointType, Eigen::aligned_allocator<PointType>> PointVector; // 点向量类型  
typedef Vector3d V3D; // 三维double向量  
typedef Matrix3d M3D; // 三维double矩阵  
typedef Vector3f V3F; // 三维float向量  
typedef Matrix3f M3F; // 三维float矩阵  

// 宏定义，用于简化矩阵和向量的声明  
#define MD(a, b) Matrix<double, (a), (b)> // double矩阵  
#define VD(a) Matrix<double, (a), 1> // double列向量  
#define MF(a, b) Matrix<float, (a), (b)> // float矩阵  
#define VF(a) Matrix<float, (a), 1> // float列向量  

// 常量定义  
const M3D Eye3d(M3D::Identity()); // 3D单位矩阵  
const M3F Eye3f(M3F::Identity()); // 3D单位矩阵（float）  
const V3D Zero3d(0, 0, 0); // 3D零向量（double）  
const V3F Zero3f(0, 0, 0); // 3D零向量（float）  

// 测量组结构体，包含Lidar数据和IMU数据  
struct MeasureGroup  
{  
  MeasureGroup()  
  {  
    lidar_beg_time = 0.0; // Lidar开始时间初始化为0  
    lidar_last_time = 0.0; // Lidar最后时间初始化为0  
    this->lidar.reset(new PointCloudXYZI()); // 创建新的点云对象  
  };  

  double lidar_beg_time; // Lidar开始时间  
  double lidar_last_time; // Lidar最后时间  
  PointCloudXYZI::Ptr lidar; // Lidar点云指针  
  deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu; // 存储IMU数据的队列  
};  

// 计算两点之间距离的模板函数  
template <typename T>  
T calc_dist(PointType p1, PointType p2)  
{  
  // 计算并返回平方距离  
  T d =  
    (p1.x - p2.x) * (p1.x - p2.x) +   
    (p1.y - p2.y) * (p1.y - p2.y) +   
    (p1.z - p2.z) * (p1.z - p2.z);  
  return d;  
}  

// 计算点与三维Eigen向量之间的距离  
template <typename T>  
T calc_dist(Eigen::Vector3d p1, PointType p2)  
{  
  T d = (p1(0) - p2.x) * (p1(0) - p2.x) +   
        (p1(1) - p2.y) * (p1(1) - p2.y) +  
        (p1(2) - p2.z) * (p1(2) - p2.z);  
  return d;  
}  

// 压缩点云时间序列的模板函数  
template <typename T>  
std::vector<int> time_compressing(const PointCloudXYZI::Ptr & point_cloud)  
{  
  int points_size = point_cloud->points.size(); // 点云的点数  
  int j = 0; // 索引计数器  
  std::vector<int> time_seq; // 用于存储时间序列的向量  
  time_seq.reserve(points_size); // 预留空间以提高效率  
  
  // 遍历点云中的点  
  for (int i = 0; i < points_size - 1; i++) {  
    j++;  
    // 若下一个点的曲率大于当前点，则记录指数  
    if (point_cloud->points[i + 1].curvature > point_cloud->points[i].curvature) {  
      time_seq.emplace_back(j);  
      j = 0; // 重置计数器  
    }  
  }  
  //   if (j == 0)
  //   {
  //     time_seq.emplace_back(1);
  //   }
  //   else
  // 将最后一个计数值添加到时间序列中  
  time_seq.emplace_back(j + 1);  
  return time_seq; // 返回时间序列  
}  

/* comment
plane equation: Ax + By + Cz + D = 0
convert to: A/D*x + B/D*y + C/D*z = -1
solve: A0*x0 = b0
where A0_i = [x_i, y_i, z_i], x0 = [A/D, B/D, C/D]^T, b0 = [-1, ..., -1]^T
normvec:  normalized x0
*/
// 估计法向量的模板函数  
template <typename T>  
bool esti_normvector(  
  Matrix<T, 3, 1> & normvec, // 输出法向量  
  const PointVector & point, // 输入的点云  
  const T & threshold, // 误差阈值  
  const int & point_num) // 点的数量  
{  
  MatrixXf A(point_num, 3); // 存储点坐标的矩阵  
  MatrixXf b(point_num, 1); // 存储常数向量  
  b.setOnes(); // 设置常数向量为-1  
  b *= -1.0f;  

  // 将点坐标填入矩阵A  
  for (int j = 0; j < point_num; j++) {  
    A(j, 0) = point[j].x;  
    A(j, 1) = point[j].y;  
    A(j, 2) = point[j].z;  
  }  
  
  // 通过最小二乘法求法向量  
  normvec = A.colPivHouseholderQr().solve(b);  

  // 检查所有点到平面的距离是否在阈值内  
  for (int j = 0; j < point_num; j++) {  
    if (fabs(normvec(0) * point[j].x + normvec(1) * point[j].y + normvec(2) * point[j].z + 1.0f) > threshold) {  
      return false; // 如果超出阈值，则返回false  
    }  
  }  

  normvec.normalize(); // 归一化法向量  
  return true; // 返回true表示成功  
}  

// 估计平面的模板函数  
template <typename T>  
bool esti_plane(Matrix<T, 4, 1> & pca_result, const PointVector & point, const T & threshold)  
{  
  Matrix<T, NUM_MATCH_POINTS, 3> A; // 存储匹配点的坐标  
  Matrix<T, NUM_MATCH_POINTS, 1> b; // 常数向量  
  A.setZero(); // 初始化为零  
  b.setOnes();  
  b *= -1.0f; // 设置常数向量为-1  

  // 将匹配点的坐标填入矩阵A  
  for (int j = 0; j < NUM_MATCH_POINTS; j++) {  
    A(j, 0) = point[j].x;  
    A(j, 1) = point[j].y;  
    A(j, 2) = point[j].z;  
  }  

  // 通过最小二乘法估计法向量  
  Matrix<T, 3, 1> normvec = A.colPivHouseholderQr().solve(b);  

  T n = normvec.norm(); // 计算法向量的范数  
  pca_result(0) = normvec(0) / n; // 归一化法向量的x分量  
  pca_result(1) = normvec(1) / n; // 归一化法向量的y分量  
  pca_result(2) = normvec(2) / n; // 归一化法向量的z分量  
  pca_result(3) = 1.0 / n; // 标定平面方程的D项  

  // 检查所有点到平面的距离是否在阈值内  
  for (int j = 0; j < NUM_MATCH_POINTS; j++) {  
    if (fabs(pca_result(0) * point[j].x + pca_result(1) * point[j].y + pca_result(2) * point[j].z + pca_result(3)) > threshold) {  
      return false; // 如果超出阈值，则返回false  
    }  
  }  
  return true; // 返回true表示估计成功  
}  

// 获取ROS时间戳的简单转换函数  
inline double get_time_sec(const builtin_interfaces::msg::Time & time)  
{  
  return rclcpp::Time(time).seconds(); // 转换为秒  
}  

// 将时间戳转换为ROS时间的简单映射函数  
inline rclcpp::Time get_ros_time(double timestamp)  
{  
  int32_t sec = std::floor(timestamp); // 获取秒数  
  auto nanosec_d = (timestamp - std::floor(timestamp)) * 1e9; // 将小数部分转换为纳秒  
  uint32_t nanosec = nanosec_d; // 纳秒部分  
  return rclcpp::Time(sec, nanosec); // 返回ROS时间  
}  

#endif // COMMON_LIB_H
