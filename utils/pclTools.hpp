#ifndef PCL_TOOLS_HPP
#define PCL_TOOLS_HPP

#define PCL_NO_PRECOMPILE

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/uniform_sampling.h>

#include <pcl/common/transforms.h>

#include "point_type.h"

#include <iostream>
#include <vector>
#include <cstdlib>
#include <vector>

typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointXYZRGB PointXYZRGB;
typedef pcl::PointNormal PointNormal;
typedef pcl::PointXYZI PointXYZI;
typedef PointXYZIR PointXYZIR;
typedef PointXYZIRL PointXYZIRL;

template <typename PointT>
class pclTools
{
public:
  // 편의를 위한 typedef (스마트 포인터)
  using PointCloudPtr = typename pcl::PointCloud<PointT>::Ptr;

  inline static void voxelGridSubsample(
      const typename pcl::PointCloud<PointT>::Ptr &input, typename pcl::PointCloud<PointT>::Ptr &output, float leaf_size)
  {
    if (!output)
    output.reset(new pcl::PointCloud<PointT>());
    output->clear();
    // reserve 는 한 번만(예: 생성자에서) 해 두시면 됩니다.
    // output->points.reserve(max_expected_size);

    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(input);
    vg.setLeafSize(leaf_size, leaf_size, leaf_size);
    vg.filter(*output);
  }

  // 지정한 반경 내에서 균일하게 샘플링
  inline static typename pcl::PointCloud<PointT>::Ptr uniformSubsample(
      const typename pcl::PointCloud<PointT>::Ptr &input, float radius)
  {
    pcl::UniformSampling<PointT> uniform;
    uniform.setInputCloud(input);
    uniform.setRadiusSearch(radius);

    // UniformSampling은 선택된 점들의 인덱스 벡터를 반환합니다.
    std::vector<int> indices;
    uniform.filter(indices);

    typename pcl::PointCloud<PointT>::Ptr output(new pcl::PointCloud<PointT>());
    output->points.reserve(indices.size());
    for (const int idx : indices)
    {
      output->points.push_back(input->points[idx]);
    }
    output->width = static_cast<uint32_t>(output->points.size());
    output->height = 1;
    output->is_dense = input->is_dense;
    return output;
  }

  // 입력 클라우드에서 무작위로 sample_num 개의 포인트 선택
  inline static typename pcl::PointCloud<PointT>::Ptr randomSubsample(
      const typename pcl::PointCloud<PointT>::Ptr &input, int sample_num)
  {
    typename pcl::PointCloud<PointT>::Ptr output(new pcl::PointCloud<PointT>());
    pcl::RandomSample<PointT> random_filter;
    random_filter.setInputCloud(input);
    random_filter.setSample(sample_num);
    random_filter.filter(*output);
    return output;
  }

  inline static double calcDist3d(
      const typename pcl::PointCloud<PointT>::Ptr p1,
      const typename pcl::PointCloud<PointT>::Ptr p2)
  {
    double dist = sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
    return dist;
  }

  inline static void rotatePointCloud(
      const typename pcl::PointCloud<PointT>::Ptr inCld,
      const typename pcl::PointCloud<PointT>::Ptr outCld,
      const Eigen::Matrix3d &R)
  {
    Eigen::Matrix4d tf_r(Eigen::Matrix4d::Identity());
    tf_r.block(0, 0, 3, 3) = R;
    pcl::transformPointCloud(*inCld, *outCld, tf_r);
    return;
  }

  inline static void removeClosePoints(
      const typename pcl::PointCloud<PointT>::Ptr inCld,
      const typename pcl::PointCloud<PointT>::Ptr outCld,
      const float thres)
  {
    outCld->header = inCld->header;
    outCld->points.resize(inCld->points.size());

    size_t j = 0;

    for (size_t i = 0; i < inCld->points.size(); ++i)
    {
      if (isnanl(inCld->points[i].x) || isnanl(inCld->points[i].y) || isnanl(inCld->points[i].z))
        continue;

      if (inCld->points[i].x * inCld->points[i].x + inCld->points[i].y * inCld->points[i].y + inCld->points[i].z * inCld->points[i].z < thres * thres)
        continue;

      outCld->points[j] = inCld->points[i];
      j++;
    }

    if (j != inCld->points.size())
    {
      outCld->points.resize(j);
    }

    outCld->height = 1;
    outCld->width = static_cast<uint32_t>(j);
    outCld->is_dense = true;

    return;
  }

  inline static void removeNaNCloud(
    const typename pcl::PointCloud<PointT>::Ptr &cloud){
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
  }
};

#endif

// inline double normalizeAngle(double angle)
// {
//     while (angle >= M_PI / 2.0)
//         angle -= M_PI;
//     while (angle <= -M_PI / 2.0)
//         angle += M_PI;
//     return angle;
// }
// inline double calcDist3d(PointXYZIRL p1, PointXYZIRL p2)
// {
//     double dist = sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
//     return dist;
// }
// inline double calcDist3d(pcl::PointXYZINormal p1, pcl::PointXYZINormal p2)
// {
//     double dist = sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
//     return dist;
// }
// inline double calcDist3d(pcl::PointXYZ p1, pcl::PointXYZ p2)
// {
//     double dist = sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
//     return dist;
// }
;
// }

// inline void filterXYZ(pcl::PointCloud<PointXYZIRL>::Ptr cld,
//                       pcl::PointXYZ min_p, pcl::PointXYZ max_p, std::string field, bool keep)
// {
//   pcl::ConditionalRemoval<PointXYZIRL> condrem;

//   if(keep)
//   {
//     pcl::ConditionAnd<PointXYZIRL>::Ptr rangeCondAnd(new pcl::ConditionAnd<PointXYZIRL>());

//     if(isContain(field,"x"))
//     {
//       rangeCondAnd->addComparison(pcl::FieldComparison<PointXYZIRL>::ConstPtr(new pcl::FieldComparison<PointXYZIRL>("x", pcl::ComparisonOps::GT, min_p.x)));
//       rangeCondAnd->addComparison(pcl::FieldComparison<PointXYZIRL>::ConstPtr(new pcl::FieldComparison<PointXYZIRL>("x", pcl::ComparisonOps::LT, max_p.x)));
//     }
//     if(isContain(field,"y"))
//     {
//       rangeCondAnd->addComparison(pcl::FieldComparison<PointXYZIRL>::ConstPtr(new pcl::FieldComparison<PointXYZIRL>("y", pcl::ComparisonOps::GT, min_p.y)));
//       rangeCondAnd->addComparison(pcl::FieldComparison<PointXYZIRL>::ConstPtr(new pcl::FieldComparison<PointXYZIRL>("y", pcl::ComparisonOps::LT, max_p.y)));
//     }
//     if(isContain(field,"z"))
//     {
//       rangeCondAnd->addComparison(pcl::FieldComparison<PointXYZIRL>::ConstPtr(new pcl::FieldComparison<PointXYZIRL>("z", pcl::ComparisonOps::GT, min_p.z)));
//       rangeCondAnd->addComparison(pcl::FieldComparison<PointXYZIRL>::ConstPtr(new pcl::FieldComparison<PointXYZIRL>("z", pcl::ComparisonOps::LT, max_p.z)));
//     }
//     condrem.setCondition(rangeCondAnd);
//   }
//   else
//   {
//     pcl::ConditionOr<PointXYZIRL>::Ptr rangeCondOr(new pcl::ConditionOr<PointXYZIRL>());
//     if(isContain(field,"x"))
//     {
//       rangeCondOr->addComparison(pcl::FieldComparison<PointXYZIRL>::ConstPtr(new pcl::FieldComparison<PointXYZIRL>("x", pcl::ComparisonOps::LT, min_p.x)));
//       rangeCondOr->addComparison(pcl::FieldComparison<PointXYZIRL>::ConstPtr(new pcl::FieldComparison<PointXYZIRL>("x", pcl::ComparisonOps::GT, max_p.x)));
//     }
//     if (isContain(field, "y"))
//     {
//       rangeCondOr->addComparison(pcl::FieldComparison<PointXYZIRL>::ConstPtr(new pcl::FieldComparison<PointXYZIRL>("y", pcl::ComparisonOps::LT, min_p.y)));
//       rangeCondOr->addComparison(pcl::FieldComparison<PointXYZIRL>::ConstPtr(new pcl::FieldComparison<PointXYZIRL>("y", pcl::ComparisonOps::GT, max_p.y)));
//     }
//     if (isContain(field, "z"))
//     {
//       rangeCondOr->addComparison(pcl::FieldComparison<PointXYZIRL>::ConstPtr(new pcl::FieldComparison<PointXYZIRL>("z", pcl::ComparisonOps::LT, min_p.z)));
//       rangeCondOr->addComparison(pcl::FieldComparison<PointXYZIRL>::ConstPtr(new pcl::FieldComparison<PointXYZIRL>("z", pcl::ComparisonOps::GT, max_p.z)));
//     }
//     condrem.setCondition(rangeCondOr);
//   }
//   condrem.setInputCloud(cld);
//   condrem.setKeepOrganized(keep);
//   condrem.filter(*cld);
//   std::vector<int> indiceLet;
//   pcl::removeNaNFromPointCloud(*cld, *cld, indiceLet);

//   return;
// }
// inline void setROI(
//     const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &inCld,
//     const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &outCld,
//     double x_min, double x_max, double y_min, double y_max, double z_min, double z_max)
// {

//     pcl::PassThrough<pcl::PointXYZINormal> pass;
//     pass.setInputCloud(inCld);
//     pass.setFilterFieldName("x");
//     pass.setFilterLimits(x_min, x_max);
//     pass.filter(*outCld);

//     pass.setInputCloud(outCld);
//     pass.setFilterFieldName("y");
//     pass.setFilterLimits(y_min, y_max);
//     pass.filter(*outCld);

//     pass.setInputCloud(outCld);
//     pass.setFilterFieldName("z");
//     pass.setFilterLimits(z_min, z_max);
//     pass.filter(*outCld);
//     return;
// }
// inline Eigen::Matrix4d getTFMat(Eigen::Quaterniond q, Eigen::Vector3d t)
// {
//   Eigen::Matrix4d transform_Mat;
//   transform_Mat << q.toRotationMatrix(), -q.toRotationMatrix() * t, 0, 0, 0, 1;
//   return transform_Mat;
// }
// inline Eigen::Matrix4d getTFMat(double x, double y, double z, double roll, double pitch, double yaw)
// {
//   Eigen::AngleAxisd rollAngle(roll * M_PI / 180.0, Eigen::Vector3d::UnitX());
//   Eigen::AngleAxisd pitchAngle(pitch * M_PI / 180.0, Eigen::Vector3d::UnitY());
//   Eigen::AngleAxisd yawAngle(yaw * M_PI / 180.0, Eigen::Vector3d::UnitZ());

//   Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
//   Eigen::Matrix3d R = q.normalized().toRotationMatrix();
//   Eigen::Matrix4d tf_r(Eigen::Matrix4d::Identity());
//   tf_r.block(0, 0, 3, 3) = R;
//   tf_r(0, 3) = x;
//   tf_r(1, 3) = y;
//   tf_r(2, 3) = z;

//   return tf_r;
// }
// inline void tfPointCloud(
//     const pcl::PointCloud<pcl::PointXYZINormal>::Ptr inCld,
//     const pcl::PointCloud<pcl::PointXYZINormal>::Ptr outCld,
//     double x, double y, double z, Eigen::Matrix3d R)
// {
//   // tf2::Quaternion euler2quat;
//   // euler2quat.setRPY(roll, pitch, yaw);
//   // Eigen::Quaternionf rotation_quat;
//   // rotation_quat.x() = euler2quat.getX();
//   // rotation_quat.y() = euler2quat.getY();
//   // rotation_quat.z() = euler2quat.getZ();
//   // rotation_quat.w() = euler2quat.getW();

//   // Eigen::Matrix3f R = rotation_quat.normalized().toRotationMatrix();
//   Eigen::Matrix4d tf_r(Eigen::Matrix4d::Identity());
//   tf_r.block(0, 0, 3, 3) = R;
//   tf_r(0, 3) = x;
//   tf_r(1, 3) = y;
//   tf_r(2, 3) = z;
//   pcl::transformPointCloud(*inCld, *outCld, tf_r);

//   return;
// }
// inline void tfPointCloud(
//     const pcl::PointCloud<pcl::PointXYZI>::Ptr inCld,
//     const pcl::PointCloud<pcl::PointXYZI>::Ptr outCld,
//     double x, double y, double z, double roll, double pitch, double yaw)
// {
//   Eigen::AngleAxisd rollAngle(roll * M_PI / 180.0, Eigen::Vector3d::UnitX());
//   Eigen::AngleAxisd pitchAngle(pitch * M_PI / 180.0, Eigen::Vector3d::UnitY());
//   Eigen::AngleAxisd yawAngle(yaw * M_PI / 180.0, Eigen::Vector3d::UnitZ());

//   Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
//   // Eigen::Matrix3d rotationMatrix = q.matrix();
//   // tf2::Quaternion euler2quat;
//   // euler2quat.setRPY(roll, pitch, yaw);
//   // Eigen::Quaterniond rotation_quat;
//   // rotation_quat.x() = euler2quat.getX();
//   // rotation_quat.y() = euler2quat.getY();
//   // rotation_quat.z() = euler2quat.getZ();
//   // rotation_quat.w() = euler2quat.getW();

//   Eigen::Matrix3d R = q.normalized().toRotationMatrix();
//   Eigen::Matrix4d tf_r(Eigen::Matrix4d::Identity());
//   tf_r.block(0, 0, 3, 3) = R;
//   tf_r(0, 3) = x;
//   tf_r(1, 3) = y;
//   tf_r(2, 3) = z;
//   if(inCld->points.size())
//     pcl::transformPointCloud(*inCld, *outCld, tf_r);
//   return;
// }
// inline void tfPointCloud(
//     const pcl::PointCloud<pcl::PointXYZINormal>::Ptr inCld,
//     const pcl::PointCloud<pcl::PointXYZINormal>::Ptr outCld,
//     double x, double y, double z, double roll, double pitch, double yaw)
// {
//   Eigen::AngleAxisd rollAngle(roll * M_PI / 180.0, Eigen::Vector3d::UnitX());
//   Eigen::AngleAxisd pitchAngle(pitch * M_PI / 180.0, Eigen::Vector3d::UnitY());
//   Eigen::AngleAxisd yawAngle(yaw * M_PI / 180.0, Eigen::Vector3d::UnitZ());

//   Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
//   // Eigen::Matrix3d rotationMatrix = q.matrix();
//   // tf2::Quaternion euler2quat;
//   // euler2quat.setRPY(roll, pitch, yaw);
//   // Eigen::Quaterniond rotation_quat;
//   // rotation_quat.x() = euler2quat.getX();
//   // rotation_quat.y() = euler2quat.getY();
//   // rotation_quat.z() = euler2quat.getZ();
//   // rotation_quat.w() = euler2quat.getW();

//   Eigen::Matrix3d R = q.normalized().toRotationMatrix();
//   Eigen::Matrix4d tf_r(Eigen::Matrix4d::Identity());
//   tf_r.block(0, 0, 3, 3) = R;
//   tf_r(0, 3) = x;
//   tf_r(1, 3) = y;
//   tf_r(2, 3) = z;
//   pcl::transformPointCloud(*inCld, *outCld, tf_r);

//   return;
// }
// inline void tfPointCloud(
//     const pcl::PointCloud<pcl::PointXYZINormal>::Ptr inCld,
//     const pcl::PointCloud<pcl::PointXYZINormal>::Ptr outCld,
//     const Eigen::Matrix3d &tf)
// {
//   Eigen::Matrix4d tf_r(Eigen::Matrix4d::Identity());
//   tf_r.block(0, 0, 3, 3) = tf;
//   pcl::transformPointCloud(*inCld, *outCld, tf_r);
//   return;
// }

// inline void ransacLine(
//     const pcl::PointCloud<pcl::PointXYZ>::Ptr inCld,
//     const pcl::ModelCoefficients::Ptr &coeff,
//     const pcl::PointIndices::Ptr &inliers, Eigen::Vector3f axis,
//     double eps_angle, double distance_threshold, int max_interations)
// {
//   pcl::SACSegmentation<pcl::PointXYZ> seg;
//   seg.setOptimizeCoefficients(true);
//   seg.setModelType(pcl::SACMODEL_PARALLEL_LINE);
//   seg.setInputCloud(inCld);
//   seg.setAxis(axis);
//   seg.setEpsAngle(eps_angle * (M_PI / 180.0f));
//   seg.setDistanceThreshold(distance_threshold);
//   seg.setMaxIterations(max_interations);
//   seg.segment(*inliers, *coeff);
//   return;
// }
// inline void clustering(
//     const pcl::PointCloud<pcl::PointXYZINormal>::Ptr inCld,
//     const pcl::PointCloud<pcl::PointXYZINormal>::Ptr outCld,
//     double tolerance, int cluster_min, int cluster_max, int threshold, double h_threshold)
// {
//   std::vector<pcl::PointIndices> cluster_indices;
//   pcl::EuclideanClusterExtraction<pcl::PointXYZINormal> ec;
//   ec.setClusterTolerance(tolerance);
//   ec.setMinClusterSize(cluster_min);
//   ec.setMaxClusterSize(cluster_max);
//   ec.setInputCloud(inCld);
//   ec.extract(cluster_indices);

//   if (cluster_indices.empty())
//   {
//     return;
//   }
//   for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
//        it != cluster_indices.end(); ++it)
//   {
//     if(it->indices.size() >= threshold)
//     {
//       pcl::PointCloud<pcl::PointXYZINormal>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZINormal>);
//       double height = 0.0;
//       double min_z = 10.0;
//       double max_z = -10.0;
//       for (std::vector<int>::const_iterator pit = it->indices.begin();
//            pit != it->indices.end(); ++pit)
//       {
//         if(pit == it->indices.begin())
//         {
//           tmp->push_back(inCld->points[*pit]);
//           min_z = inCld->points[*pit].z;
//           max_z = inCld->points[*pit].z;
//           continue;
//         }

//         if(max_z - min_z >= h_threshold)
//         {
//           if(tmp->points.size() > 0)
//           {
//             *outCld += *tmp;
//             tmp->clear();
//           }
//           outCld->push_back(inCld->points[*pit]);
//           continue;
//         }

//         if(inCld->points[*pit].z < min_z)
//           min_z = inCld->points[*pit].z;
//         else if(inCld->points[*pit].z > max_z)
//           max_z = inCld->points[*pit].z;

//       }
//     }
//   }
//   return;
// }

// inline void clustering(
//     const pcl::PointCloud<pcl::PointXYZINormal>::Ptr inCld,
//     const pcl::PointCloud<pcl::PointXYZINormal>::Ptr outCld,
//     double tolerance, int cluster_min, int cluster_max)
// {
//   std::vector<pcl::PointIndices> cluster_indices;
//   pcl::EuclideanClusterExtraction<pcl::PointXYZINormal> ec;
//   ec.setClusterTolerance(tolerance);
//   ec.setMinClusterSize(cluster_min);
//   ec.setMaxClusterSize(cluster_max);
//   ec.setInputCloud(inCld);
//   ec.extract(cluster_indices);

//   if (cluster_indices.empty())
//   {
//     return;
//   }
//   for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
//        it != cluster_indices.end(); ++it)
//   {
//       for (std::vector<int>::const_iterator pit = it->indices.begin();
//            pit != it->indices.end(); ++pit)
//       {
//         outCld->points.push_back(inCld->points[*pit]);
//       }
//   }
//   return;
// }