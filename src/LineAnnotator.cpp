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
#include <math.h>
#include <geometry_msgs/Polygon.h>

#define LENGTH_THRESHOLD 180.0f

using namespace uima;
using namespace cv;
using namespace std;

class LineAnnotator : public DrawingAnnotator
{
private:
  int croppedDistance = 30;
  Mat final_line;
  ros::NodeHandle nh;
  ros::Publisher pub;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;

public:
  LineAnnotator()
      : DrawingAnnotator(__func__), nh("~")
  {
    pub = nh.advertise<geometry_msgs::Polygon>("lines2D",100);
    cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(
        new pcl::PointCloud<pcl::PointXYZRGBA>);
  }

  TyErrorId initialize(AnnotatorContext& ctx)
  {
    outInfo("initialize");
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

//  static float slope(Vec4i line)
//  {
//    if (line[2]==line[0])
//    {
//        return std::numeric_limits<float>::max();
//    }
//    return (line[3] - line[1]) / (line[2] - line[0]);
//  }

  float lineDistance(Vec4i line)
  {
    Point p1 (line[0],line[1]);
    Point p2 (line[2],line[3]);
    return abs(p1.x*p2.y-p1.y*p2.x)/norm(p1-p2);
  }

  static float lineLength(Vec4i line)
  {
    Point p1 (line[0],line[1]);
    Point p2 (line[2],line[3]);
    return norm(p1-p2);
  }

  void processImage (Mat& image)
  {
    Mat image_gray, edges;
    vector<vector<Point>> contours;
    Rect regionOfInterest (Point(croppedDistance,croppedDistance),Point(image.cols-croppedDistance,image.rows-croppedDistance));
    image_gray = image(regionOfInterest);

    //cvtColor(image_gray,image_gray,COLOR_BGRA2BGR);
    //adaptiveBilateralFilter(image_gray,image_filtered,Size(3,3),3.0);
    cvtColor(image_gray, image_gray, CV_BGR2GRAY);
    GaussianBlur(image_gray, image_gray, Size(5, 5), 0, 0);

    int lowThreshold = 70;
    Canny(image_gray, edges, 120, 200, 3);

    //cv::morphologyEx(edges,edges,cv::MORPH_CLOSE,Mat());

    dilate(edges,image,Mat(),Point(-1,-1),2);

    findContours(image,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);

    cvtColor(image, final_line, CV_GRAY2BGR);
    drawContours(final_line,contours,-1,Scalar(255,0,255),3);
    cvtColor(final_line,image,CV_BGR2GRAY);
  }

//  void detectEdges(Mat& image)
//  {
//    Mat image_gray;
//    int croppedDistance = 20;
//    Rect regionOfInterest (Point(croppedDistance,croppedDistance),Point(image.cols-croppedDistance,image.rows-croppedDistance));
//    image_gray = image(regionOfInterest);
//    cvtColor(image_gray, image_gray, CV_BGR2GRAY);
//    Mat newImage = Mat::zeros(image_gray.rows,image_gray.cols,CV_8UC1);
//    for(int i = 0; i<image.rows; i++)
//    {
//      for (int j = 0; j<image.cols;j++)
//      {
//      }
//    }
//  }

  void lineDetect2D(Mat image)
  {
    processImage(image);
    vector<Vec4i> lines;
    HoughLinesP(image, lines, 1, CV_PI / 180, 50, 100, 5);

    std::sort(lines.begin(), lines.end(), [](Vec4i l1, Vec4i l2)
                  {
                    return lineLength(l1) > lineLength(l2);
                  });

    for (size_t i = 0; i < lines.size(); i++)
    {
      // remove short lines
      if (lineLength(lines[i]) < lineLength(lines[0])*0.75)
      {
//        lines[i--] = lines[lines.size() - 1];
        lines.resize(i);
        break;
      }
      line(final_line, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]),
           Scalar(0, 255, 0), 2, CV_AA);
    }

    if (lines.size()==0)
    {
      return;
    }

    line(final_line, Point(lines[0][0], lines[0][1]), Point(lines[0][2], lines[0][3]),
         Scalar(0, 0, 255), 2, CV_AA);


    geometry_msgs::Polygon poly;
    geometry_msgs::Point32 point1,point2;
    outInfo("starting line: "<<lines[0][0]<<" and "<<lines[0][1]);
    point1.x = cloud->at(lines[0][0]+croppedDistance,lines[0][1]+croppedDistance).x;
    point1.y = cloud->at(lines[0][0]+croppedDistance,lines[0][1]+croppedDistance).y;
    point1.z = cloud->at(lines[0][0]+croppedDistance,lines[0][1]+croppedDistance).z;

    point2.x = cloud->at(lines[0][2]+croppedDistance,lines[0][3]+croppedDistance).x;
    point2.y = cloud->at(lines[0][2]+croppedDistance,lines[0][3]+croppedDistance).y;
    point2.z = cloud->at(lines[0][2]+croppedDistance,lines[0][3]+croppedDistance).z;

    poly.points.push_back(point1);
    poly.points.push_back(point2);
    pub.publish(poly);

  }

  TyErrorId processWithLock(CAS& tcas, ResultSpecification const& res_spec)
  {
    rs::SceneCas cas(tcas);
    Mat image;

    outInfo("process start");

    cas.get(VIEW_COLOR_IMAGE, image);
    cas.get(VIEW_CLOUD,*cloud);

    lineDetect2D(image);
    //processImage(image);
    return UIMA_ERR_NONE;
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer& visualizer,
                              const bool firstRun)
  {
    return;
  }
  void drawImageWithLock(cv::Mat& disp) { disp = final_line; }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(LineAnnotator)
