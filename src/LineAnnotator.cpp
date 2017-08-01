#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
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

using namespace uima;
using namespace cv;
using namespace std;

class LineAnnotator : public DrawingAnnotator
{
private:
  float test_param;
  //pcl::PointIndices::Ptr inliers;
  double pointSize;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filtered_cloud;
  Mat final_line;

public:
  LineAnnotator(): DrawingAnnotator(__func__), pointSize(1.0)//inliers (new pcl::PointIndices)
  {
    cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr (new pcl::PointCloud<pcl::PointXYZRGBA>);
    filtered_cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
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

  void line_det_2d(Mat image)
  {
    Mat image_gray, edges;
    cvtColor(image, image_gray, CV_BGR2GRAY);
    GaussianBlur(image_gray, image_gray, Size(3, 3), 0, 0);
    Canny(image_gray, edges, 30, 90, 3);

    cvtColor(edges, final_line, CV_GRAY2BGR);

    vector<Vec4i> lines;
    HoughLinesP(edges, lines, 1, CV_PI / 180, 50, 100, 5);
    for (size_t i = 0; i < lines.size(); i++)
    {
      Vec4i l = lines[i];
      line(final_line, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 255, 0),
           2, CV_AA);
    }
    imshow("img",final_line);
    waitKey(10);
  }

  void line_det_3d(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
  {
    outInfo("filtered"<<cloud->points.size());

    pcl::SACSegmentation<pcl::PointXYZRGBA> segmentation;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    segmentation.setModelType(pcl::SACMODEL_LINE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setDistanceThreshold(0.02f);

    segmentation.setInputCloud(cloud);
    segmentation.segment(*inliers,*coefficients);
   // pcl::copyPointCloud<pcl::PointXYZRGBA>(*cloud,inliers->indices,*filtered_cloud);
  }

  TyErrorId processWithLock(CAS& tcas, ResultSpecification const& res_spec)
  {
    rs::SceneCas cas(tcas);
    Mat image;

    outInfo("process start");

    cas.get(VIEW_COLOR_IMAGE,image);
    //cas.get(VIEW_CLOUD_DOWNSAMPLED, *cloud);

    line_det_2d(image);
    outInfo("finished 2D");
    //line_det_3d(cloud);


    return UIMA_ERR_NONE;
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
  {
    const std::string &cloudname = "Line Cloud Stuff";

    if(firstRun)
    {
      visualizer.addPointCloud(cloud, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
    else
    {
      visualizer.updatePointCloud(cloud, cloudname);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }
  }
  void drawImageWithLock(cv::Mat &disp)
  {
    disp = final_line;
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(LineAnnotator)
