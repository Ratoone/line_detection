/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author(s): Ferenc Balint-Benczedi <balintbe@cs.uni-bremen.de>
 *         Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
 *         Jan-Hendrik Worch <jworch@cs.uni-bremen.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// UIMA
#include <uima/api.hpp>

#include <opencv2/opencv.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// RS
#include <rs/scene_cas.h>
#include <rs/DrawingAnnotator.h>
#include <rs/utils/time.h>
#include <rs/utils/output.h>
#include <rs/utils/common.h>
#include <rs/utils/exception.h>

//ROS
#include <ros/package.h>

#define DEBUG_OUTPUT 0

using namespace uima;

class PlaneExtraction : public DrawingAnnotator
{
private:
  std::string mode;
  // PCL
  pcl::PointIndices::Ptr plane_inliers;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr display;
  std::vector<int> mapping_indices;
  pcl::ExtractIndices<pcl::PointXYZRGBA> ei;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output;

  //Drawing
  bool foundPlane, saveToFile;
  cv::Mat image;
  double pointSize;

  //params
  int min_plane_inliers,
      max_iterations;
  float distance_threshold,
        max_curvature,
        angular_threshold_deg;

  //std::string pathToModelFile;
  const std::string cloudname;

public:
  PlaneExtraction() : DrawingAnnotator(__func__),mode("PCL"), display(new pcl::PointCloud<pcl::PointXYZRGBA>()),
    saveToFile(false), pointSize(1),cloudname("TestCloud")
  {

  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");


    if(ctx.isParameterDefined("min_plane_inliers"))
    {
      ctx.extractValue("min_plane_inliers", min_plane_inliers);
    }
    if(ctx.isParameterDefined("max_iterations"))
    {
      ctx.extractValue("max_iterations", max_iterations);
    }
    if(ctx.isParameterDefined("distance_threshold"))
    {
      ctx.extractValue("distance_threshold", distance_threshold);
    }
    if(ctx.isParameterDefined("max_curvature"))
    {
      ctx.extractValue("max_curvature", max_curvature);
    }
    if(ctx.isParameterDefined("angular_threshold_deg"))
    {
      ctx.extractValue("angular_threshold_deg", angular_threshold_deg);
    }

    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }


  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    MEASURE_TIME;
    outInfo("process begins");

    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();

    cas.get(VIEW_COLOR_IMAGE_HD, image);

    foundPlane = false;

    outInfo("Estimating form Point Cloud");
    estimateFromPCL(tcas, scene);


    if(!foundPlane)
    {
      outWarn("no plane found, no further processing!");
      throw rs::FrameFilterException();
    }


    return UIMA_ERR_NONE;
  }



  void getMask(const pcl::PointIndices &inliers, const cv::Size &size, cv::Mat &mask, cv::Rect &roi)
  {
    cv::Mat tmp = cv::Mat::zeros(size.height, size.width, CV_8U);
    int minX = size.width, maxX = 0;
    int minY = size.height, maxY = 0;

    //#pragma omp parallel for
    for(size_t i = 0; i < inliers.indices.size(); ++i)
    {
      const int index = inliers.indices[i];
      const int x = index % size.width;
      const int y = index / size.width;
      tmp.at<uint8_t>(y, x) = 255;

      minX = std::min(minX, x);
      maxX = std::max(maxX, x);
      minY = std::min(minY, y);
      maxY = std::max(maxY, y);
    }

    roi = cv::Rect(minX, minY, (maxX - minX) + 1, (maxY - minY) + 1);
    tmp(roi).copyTo(mask);
  }


  void estimateFromPCL(CAS &tcas, rs::Scene &scene)
  {
    outInfo("Estimating plane form Point Cloud data");
    rs::SceneCas cas(tcas);

    cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients);

    cas.get(VIEW_CLOUD, *cloud);

    std::vector<float> planeModel(4);
    if(process_cloud(plane_coefficients))
    {
      foundPlane = true;

      if(plane_coefficients->values[3] < 0)
      {
        planeModel[0] = plane_coefficients->values[0];
        planeModel[1] = plane_coefficients->values[1];
        planeModel[2] = plane_coefficients->values[2];
        planeModel[3] = plane_coefficients->values[3];
      }
      else
      {
          planeModel[0] = -plane_coefficients->values[0];
          planeModel[1] = -plane_coefficients->values[1];
          planeModel[2] = -plane_coefficients->values[2];
          planeModel[3] = -plane_coefficients->values[3];
      }

      pcl::PointIndices::Ptr temp(new pcl::PointIndices());
      temp->indices.resize(plane_inliers->indices.size());
      for(size_t i = 0; i < plane_inliers->indices.size(); ++i)
      {
        temp->indices[i] = mapping_indices[plane_inliers->indices[i]];
      }
      plane_inliers.swap(temp);

      cv::Mat mask;
      cv::Rect roi;
      getMask(*plane_inliers, cv::Size(cloud->width, cloud->height), mask, roi);

      rs::Plane plane = rs::create<rs::Plane>(tcas);
      plane.model(planeModel);
      plane.inliers(plane_inliers->indices);
      plane.roi(rs::conversion::to(tcas, roi));
      plane.mask(rs::conversion::to(tcas, mask));
      plane.source("RANSAC");
      scene.annotations.append(plane);

      output.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
      ei.setInputCloud(cloud);
      ei.setIndices(plane_inliers);
      //      ei.setKeepOrganized(true);
      ei.filter(*output);
      cas.set(VIEW_CLOUD,*output);
    }
    else
    {
      outInfo("No plane found in the cloud");
    }
  }


  bool process_cloud(pcl::ModelCoefficients::Ptr &plane_coefficients)
  {
    plane_inliers = pcl::PointIndices::Ptr(new pcl::PointIndices);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(cloud);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered_no_nan(new pcl::PointCloud<pcl::PointXYZRGBA>);

    mapping_indices.clear();

    pcl::removeNaNFromPointCloud(*cloud_filtered, *cloud_filtered_no_nan, mapping_indices);

    // find the major plane
    pcl::SACSegmentation<pcl::PointXYZRGBA> plane_segmentation;

    plane_segmentation.setOptimizeCoefficients(true);
    plane_segmentation.setModelType(pcl::SACMODEL_PLANE);
    plane_segmentation.setMethodType(pcl::SAC_RANSAC);
    plane_segmentation.setDistanceThreshold(distance_threshold);
    plane_segmentation.setMaxIterations(max_iterations);
    plane_segmentation.setInputCloud(cloud_filtered_no_nan);
    plane_segmentation.segment(*plane_inliers, *plane_coefficients);

    if(plane_inliers->indices.size() < min_plane_inliers)
    {
      outWarn("not enough inliers!");
      return false;
    }

    std::sort(plane_inliers->indices.begin(), plane_inliers->indices.end());
    if(plane_inliers->indices.size() == 0)
    {
      return false;
    }
    outDebug("Number of inliers in plane:" << plane_inliers->indices.size());

#if DEBUG_OUTPUT
    pcl::PCDWriter writer;
    outInfo("Size of input cloud: " << cloud->points.size());
    outInfo("Filtered cloud size: " << cloud_filtered_no_nan->points.size());
    outInfo("Downsampled cloud size: " << cloud_downsampled->points.size());
    if(cloud_filtered_no_nan->points.size() > 0)
    {
      writer.writeASCII("original.pcd", *cloud_filtered_no_nan);
    }
    if(plane_inliers->indices.size() > 0)
    {
      writer.writeASCII("plane.pcd", *cloud_filtered_no_nan, plane_inliers->indices);
    }

#endif
    return true;
  }



  void drawImageWithLock(cv::Mat &disp)
  {
    if(!foundPlane)
    {
      disp = cv::Mat::zeros(image.rows, image.cols, CV_8UC3);
      return;
    }


  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
  {

    if(firstRun)
    {
      visualizer.addPointCloud(output, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
    else
    {
      visualizer.updatePointCloud(output, cloudname);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }



 }
};

MAKE_AE(PlaneExtraction)
