#include "ros/ros.h"
#include "iis_panda_controls/SurfaceFeatures.h"
#include "iis_panda_controls/Feature.h"
#include "iis_panda_controls/FeatureVector.h"

#include <thread>
#include <pcl/point_types.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <math.h>
#include <vector>

typedef iis_panda_controls::Feature Feature;
typedef iis_panda_controls::FeatureVector FeatureVector;

int N_NORMAL_HISTOGRAM_BINS = 20;
int N_CURVATURE_HISTOGRAM_BINS = 20;
int N_SHAPE_ID_HISTOGRAM_BINS = 20;

void Init(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_rgb, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    // Reduce XYZRGB to XYZ
    cloud->width = cloud_rgb->width;
    cloud->height = cloud_rgb->height;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);

    for (size_t i = 0; i < cloud->points.size(); i++)
    {
        cloud->points[i].x = cloud_rgb->points[i].x;
        cloud->points[i].y = cloud_rgb->points[i].y;
        cloud->points[i].z = cloud_rgb->points[i].z;
    }
}

void calcFeature(Feature &feature, cv::Mat feature_data)
{
    cv::MatND feature_hist = cv::MatND(1, feature.n_hist_bins, CV_16U);
    float range[] = {feature.range_min, feature.range_max};
    const float *ranges[] = {range};
    const int channel = 0;
    const int n_bins = feature.n_hist_bins;
    cv::calcHist(&feature_data, 1, &channel, cv::Mat(), feature_hist, 1, &n_bins, ranges, true, false);
    double min = 0;
    double max = 0;
    cv::Scalar avg = 0;
    cv::Scalar std_dev = 0;
    cv::minMaxLoc(feature_data, &min, &max, 0, 0);
    cv::meanStdDev(feature_data, avg, std_dev);
    feature.min = (float)min;
    feature.max = (float)max;
    //      feature.avg = (avg.val[0] - feature.range_min) / (feature.range_max - feature.range_min);
    //      feature.dev = (std_dev.val[0] - feature.range_min) / (feature.range_max - feature.range_min);
    feature.avg = avg.val[0];
    feature.dev = std_dev.val[0];
    feature.var = feature.dev * feature.dev;

    feature.his.resize(feature.n_hist_bins);
    for (uint16_t ci = 0; ci < feature.n_hist_bins; ci++)
        feature.his[ci] = ((float *)feature_hist.data)[ci];
}

bool srv_calc_surface_features(iis_panda_controls::SurfaceFeatures::Request  &req,
         iis_panda_controls::SurfaceFeatures::Response &res)
{
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne_;
    pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> pce_;
    pcl::PointCloud<pcl::PrincipalCurvatures> princip_curves_;
    FeatureVector entity;

    bool enable_normals_ = true;
    bool enable_curvatures_ = true;

    // do nothing, probably wrong configuration
    if (!enable_curvatures_ && !enable_normals_)
        return (-1);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(req.cloud, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*cloud_rgb);

    Init(cloud_rgb, cloud);

    // in any case surface normals should be calculated
    pcl::PointCloud<pcl::Normal>::Ptr pointcloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne_.setSearchMethod(tree);
    ne_.setViewPoint(0, 0, 1.2); // this could be done more informed by getting this information from a topic

    ne_.setInputCloud(cloud);
    ne_.setRadiusSearch(0.03);
    ne_.compute(*pointcloud_normals);
    
    // pcl::io::savePCDFileASCII ("/home/marko/0_normals/test.pcd", *pointcloud_normals);

    if (enable_normals_)
    {
        cv::Mat normals_zen = cv::Mat(1, pointcloud_normals->points.size(), CV_32F);
        cv::Mat normals_azi = cv::Mat(1, pointcloud_normals->points.size(), CV_32F);

        for (uint16_t j = 0; j < pointcloud_normals->points.size(); j++)
        {
            double n_x = pointcloud_normals->points[j].normal_x;
            double n_y = pointcloud_normals->points[j].normal_y;
            double n_z = pointcloud_normals->points[j].normal_z;

            // always returns [0, PI]
            normals_zen.at<float>(0, j) = (atan2(sqrt(n_x * n_x + n_y * n_y), n_z)) * 180 / M_PI;

            // may return [-PI, PI]
            normals_azi.at<float>(0, j) = (atan2(n_y, n_x)) * 180 / M_PI;
            if (normals_azi.at<float>(0, j) < 0)
                normals_azi.at<float>(0, j) += 360;
        }

        Feature feature_zen;
        feature_zen.val_type = iis_panda_controls::Feature::DISPERSIVE_VALUED;
        feature_zen.range_min = 0;
        feature_zen.range_max = 360;
        feature_zen.n_hist_bins = N_NORMAL_HISTOGRAM_BINS;
        feature_zen.type = iis_panda_controls::Feature::NORMAL_ZEN;

        calcFeature(feature_zen, normals_zen);

        Feature feature_azi;
        feature_azi.val_type = iis_panda_controls::Feature::DISPERSIVE_VALUED;
        feature_azi.range_min = 0;
        feature_azi.range_max = 360;
        feature_azi.n_hist_bins = N_NORMAL_HISTOGRAM_BINS;
        feature_azi.type = iis_panda_controls::Feature::NORMAL_AZI;

        calcFeature(feature_azi, normals_azi);

        FeatureVector features_normal;
        features_normal.features.push_back(feature_azi);
        features_normal.features.push_back(feature_zen);

        for (Feature feat : features_normal.features)
        {
            std::cout << "New Hist" << std::endl;
            for (float value : feat.his)
            {
                std::cout << value << ' ';
            }
            std::cout << std::endl;
        }
        entity.features.insert(entity.features.end(), features_normal.features.begin(), features_normal.features.end());
        // entity.appendFeatures(features_normal);
    }
    if (enable_curvatures_)
    {
        // TODO: calculate surface curvatures here
        pce_.setKSearch(15);
        pce_.setSearchMethod(tree);
        pce_.setInputCloud(cloud);
        pce_.setInputNormals(pointcloud_normals);
        pce_.compute(princip_curves_);

        Feature feature_curvs_min;
        feature_curvs_min.val_type = iis_panda_controls::Feature::DISPERSIVE_VALUED;
        Feature feature_curvs_max;
        feature_curvs_max.val_type = iis_panda_controls::Feature::DISPERSIVE_VALUED;
        feature_curvs_min.range_min = 0;
        feature_curvs_min.range_max = 1;
        feature_curvs_min.n_hist_bins = N_CURVATURE_HISTOGRAM_BINS;
        feature_curvs_min.type = iis_panda_controls::Feature::CURV_MIN;

        feature_curvs_max = feature_curvs_min;
        feature_curvs_max.type = iis_panda_controls::Feature::CURV_MAX;

        cv::Mat curvs_min = cv::Mat(1, princip_curves_.points.size(), CV_32F);
        cv::Mat curvs_max = cv::Mat(1, princip_curves_.points.size(), CV_32F);

        for (uint16_t j = 0; j < princip_curves_.points.size(); j++)
        {

            curvs_min.at<float>(0, j) = princip_curves_.points[j].pc2;
            curvs_max.at<float>(0, j) = princip_curves_.points[j].pc1;
        }
        calcFeature(feature_curvs_min, curvs_min);
        calcFeature(feature_curvs_max, curvs_max);

        FeatureVector features_curvatures;
        features_curvatures.features.push_back(feature_curvs_min);
        features_curvatures.features.push_back(feature_curvs_max);
        entity.features.insert(entity.features.end(), features_curvatures.features.begin(), features_curvatures.features.end());
        // entity.appendFeatures(features_curvatures);
    }

    std::vector<float> shape_indices(princip_curves_.points.size());

    // extract shape index
    if (enable_curvatures_ && enable_normals_)
    {
        cv::Mat shape_indices = cv::Mat(1, princip_curves_.points.size(), CV_32F);
        for (uint i = 0; i < princip_curves_.points.size(); i++)
        {
            pcl::PrincipalCurvatures pcurves = princip_curves_.points[i];

            shape_indices.at<float>(0, i) = atan2(pcurves.pc1 + pcurves.pc2, pcurves.pc1 - pcurves.pc2) / M_PI;
        }

        FeatureVector feature_shape_indices;
        Feature feature_sid; // shape index
        feature_sid.val_type = iis_panda_controls::Feature::DISPERSIVE_VALUED;
        feature_sid.range_min = -1.0;
        feature_sid.range_max = 1.0;
        feature_sid.type = iis_panda_controls::Feature::SHAPE_INDEX;
        feature_sid.n_hist_bins = N_SHAPE_ID_HISTOGRAM_BINS;

        calcFeature(feature_sid, shape_indices);
        feature_shape_indices.features.push_back(feature_sid);
        entity.features.insert(entity.features.end(), feature_shape_indices.features.begin(), feature_shape_indices.features.end());
        // entity.appendFeatures(feature_shape_indices);
    }
    res.surface_features = entity;  
    ROS_INFO("Sending back response!");
    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "calculate_surface_features_service");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("calculate_surface_features", srv_calc_surface_features);
  ROS_INFO("Ready to calculate Surface Features of 3D point cloud.");
  ros::spin();

  return 0;
}