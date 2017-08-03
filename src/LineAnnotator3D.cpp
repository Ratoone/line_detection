#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/impl/common.hpp>
#include <rs/types/all_types.h>
// RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/DrawingAnnotator.h>

// ros
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32.h>

//
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#define LENGTH_THRESHOLD 120.0f
#define SLOPE_THRESHOLD 0.4f

using namespace uima;
using namespace cv;
using namespace std;

class LineAnnotator3D : public DrawingAnnotator
{
private:
  float test_param;
  // pcl::PointIndices::Ptr inliers;
  double pointSize;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;

public:
  LineAnnotator3D()
      : DrawingAnnotator(__func__),
        pointSize(1.0) // inliers (new pcl::PointIndices)
  {
    cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(
        new pcl::PointCloud<pcl::PointXYZRGBA>);
  }

  TyErrorId initialize(AnnotatorContext& ctx)
  {
    outInfo("initialize");
    ctx.extractValue("test_param", test_param);
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  void line_det_3d(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
  {
    pcl::PointXYZRGBA closest, furthest;
    pcl::getMinMax3D(*cloud,closest,furthest);
    this->getAnnotatorContext().assignValue("maxZ",0);

    pcl::SACSegmentation<pcl::PointXYZRGBA> segmentation;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    segmentation.setModelType(pcl::SACMODEL_LINE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setDistanceThreshold(0.01f);

    segmentation.setInputCloud(cloud);
    segmentation.segment(*inliers, *coefficients);
    // pcl::copyPointCloud<pcl::PointXYZRGBA>(*cloud,
    // inliers->indices,*filtered_cloud);
  }

  TyErrorId processWithLock(CAS& tcas, ResultSpecification const& res_spec)
  {
    rs::SceneCas cas(tcas);

    outInfo("process start");

    cas.get(VIEW_CLOUD, *cloud);

    outInfo("finished 2D");
    line_det_3d(cloud);

    return UIMA_ERR_NONE;
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer& visualizer,
                              const bool firstRun)
  {
    const std::string& cloudname = "Line Cloud Stuff";

    if (firstRun)
    {
      visualizer.addPointCloud(cloud, cloudname);
      visualizer.setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
    else
    {
      visualizer.updatePointCloud(cloud, cloudname);
      visualizer.getPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
  }
  void drawImageWithLock(cv::Mat& disp) {}
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(LineAnnotator3D)
