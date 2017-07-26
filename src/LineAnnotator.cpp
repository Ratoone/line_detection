#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <rs/types/all_types.h>
//RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>

//ros
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32.h>

//
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <cv_bridge/cv_bridge.h>

using namespace uima;
using namespace cv;


class LineAnnotator : public Annotator
{
private:
  float test_param;

public:
  static ros::Publisher test;

  TyErrorId initialize(AnnotatorContext &ctx)
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

  static void line_det_2d(const sensor_msgs::Image &msg)
  {
    //std_msgs::UInt8 x;
    //x.data = 5;
    //test.publish(x);
    //gray image
    //outInfo("callback 2D");
    cv_bridge::CvImagePtr cvImage = cv_bridge::toCvCopy(msg);
    Mat image = cvImage->image;
    Mat image_gray;
    cvtColor(image,image_gray,CV_BGR2GRAY);
    imshow("Shit",image_gray);
    GaussianBlur(image_gray, image_gray, Size(3, 3),0,0);
    //imshow("Bluuurrr",image_gray);
    Canny(image_gray,image_gray,30,90,3);
    imshow("Cannyyy",image_gray);
    //
    waitKey(10);
  }

  static void line_det_3d(pcl::PointCloud<pcl::PointXYZ>::Ptr msg)
  {

  }

  TyErrorId process(CAS &tcas, ResultSpecification const &res_spec)
  {
    int argc = 0;
    char **argv;
    outInfo("process start");
    ros::init(argc,argv,"line_detection");
    ros::NodeHandle handler;
    ros::Subscriber sub2d = handler.subscribe("/camera/rgb/image_raw",100,line_det_2d);
    //test = handler.advertise<std_msgs::Float32>("/random_topic_name",100);
    //ros::Subscriber sub3d = handler.subscribe("/camera/depth/points",100,line_det_3d);
    ros::spin();
    //pub = handler.advertise<std_msgs::Float32>("avg",100);
    /*rs::StopWatch clock;
    rs::SceneCas cas(tcas);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    outInfo("Test param =  " << test_param);
    cas.get(VIEW_CLOUD,*cloud_ptr);

    outInfo("Cloud size: " << cloud_ptr->points.size());
    outInfo("took: " << clock.getTime() << " ms.");*/
    return UIMA_ERR_NONE;
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(LineAnnotator)