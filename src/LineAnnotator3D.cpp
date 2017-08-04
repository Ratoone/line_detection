#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/impl/common.hpp>
#include <pcl/filters/extract_indices.h>

// RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/DrawingAnnotator.h>
#include <rs/types/all_types.h>

// ros
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32.h>

//
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#define SHELF_SLOPE_THRESHOLD 0.2f

using namespace uima;
using namespace cv;
using namespace std;

class LineAnnotator3D : public DrawingAnnotator
{
private:
  float cameraAngle;
  double pointSize;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output;

public:
  LineAnnotator3D()
      : DrawingAnnotator(__func__),
        pointSize(1.0)
  {
    cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(
        new pcl::PointCloud<pcl::PointXYZRGBA>);
    output = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(
        new pcl::PointCloud<pcl::PointXYZRGBA>);
  }

  TyErrorId initialize(AnnotatorContext& ctx)
  {
    outInfo("initialize");
    if (ctx.isParameterDefined("cameraAngle"))
    {
      ctx.extractValue("cameraAngle", cameraAngle);
    }
    else
    {
      cameraAngle = 0;
    }
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  void line_det_3d(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
  {
    pcl::SACSegmentation<pcl::PointXYZRGBA> segmentation;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr shelfWannabe (new pcl::PointCloud<pcl::PointXYZRGBA>());

    segmentation.setModelType(pcl::SACMODEL_PARALLEL_LINE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setDistanceThreshold(0.02f);

    segmentation.setInputCloud(cloud);
    segmentation.segment(*inliers, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZRGBA> filter (true);
    filter.setInputCloud(cloud);
    filter.setIndices(inliers);
    filter.setNegative(false);
    filter.filter(*shelfWannabe);

    outInfo("Obtained Shelf Wannabe");

    pcl::PointXYZRGBA min, max;
    pcl::getMinMax3D(*shelfWannabe,min,max);
    float shelfWannabeSlope = (max.y-min.y)/(max.x-min.y);
    if (abs(shelfWannabeSlope -tan(cameraAngle))<SHELF_SLOPE_THRESHOLD)
    {
      filter.setNegative(true);
      filter.filter(*output);
      //return true;
      //send coordinates as shelfX
    }
    //return false;
  }

  TyErrorId processWithLock(CAS& tcas, ResultSpecification const& res_spec)
  {
    rs::SceneCas cas(tcas);

    outInfo("process start");

    cas.get(VIEW_CLOUD, *output);

    for (int i=0;i<3;i++)
    {
      line_det_3d(output);
    }
    return UIMA_ERR_NONE;
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer& visualizer,
                              const bool firstRun)
  {
    const std::string& cloudname = "Line Cloud Stuff";

    if (firstRun)
    {
      visualizer.addPointCloud(output, cloudname);
      visualizer.setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
    else
    {
      visualizer.updatePointCloud(output, cloudname);
      visualizer.getPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
  }
  void drawImageWithLock(cv::Mat& disp) {}
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(LineAnnotator3D)
