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

#include <uima/api.hpp>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/impl/common.hpp>

//RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/DrawingAnnotator.h>

using namespace uima;


class OnlinePointCloudFilter : public DrawingAnnotator
{

private:
  typedef pcl::PointXYZRGBA PointT;
  pcl::PointCloud<PointT>::Ptr cloud_filtered;

  double pointSize;
  float minX, maxX, minY, maxY, minZ, maxZ, depthThreshold;
  Type cloud_type;

public:

  OnlinePointCloudFilter(): DrawingAnnotator(__func__), pointSize(1)
  {
    cloud_filtered = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    ctx.extractValue("minX", minX);
    ctx.extractValue("maxX", maxX);

    ctx.extractValue("minY", minY);
    ctx.extractValue("maxY", maxY);

    ctx.extractValue("minZ", minZ);
    ctx.extractValue("maxZ", maxZ);

    ctx.extractValue("depthThreshold",depthThreshold);

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
    outInfo("process start");
    rs::SceneCas cas(tcas);
    pcl::PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT>);
    (new pcl::PointCloud<PointT>);

    cas.get(VIEW_CLOUD, *cloud_ptr);

    PointT closest, furthest;
    pcl::getMinMax3D(*cloud_ptr,closest,furthest);
    maxZ = closest.z + depthThreshold;

    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud_ptr);
    pass.setKeepOrganized(true);
    pass.setFilterLimits(minX, maxX);
    pass.setFilterFieldName("x");
    pass.filter(*cloud_filtered);

    pass.setFilterLimits(minY, maxY);
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("y");
    pass.filter(*cloud_filtered);

    pass.setFilterLimits(minZ, maxZ);
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("z");
    pass.filter(*cloud_filtered);

    cas.set(VIEW_CLOUD, *cloud_filtered);

    return UIMA_ERR_NONE;
  }

  void drawImageWithLock(cv::Mat &disp)
  {

  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
  {
    const std::string &cloudname = this->name;

    if(firstRun)
    {
      visualizer.addPointCloud(cloud_filtered, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
    else
    {
      visualizer.updatePointCloud(cloud_filtered, cloudname);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
  }

};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(OnlinePointCloudFilter)
