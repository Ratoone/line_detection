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

#define LENGTH_THRESHOLD 120.0f
#define SLOPE_THRESHOLD tan(CV_PI * 15 / 180)

using namespace uima;
using namespace cv;
using namespace std;

class LineAnnotator : public DrawingAnnotator
{
private:
  float cameraAngle = 0;
  Mat final_line;
  ros::NodeHandle nh;
  ros::Publisher pub;

public:
  LineAnnotator()
      : DrawingAnnotator(__func__), nh("~")
  {
    pub = nh.advertise<geometry_msgs::Polygon>("lines2D",100);
  }

  TyErrorId initialize(AnnotatorContext& ctx)
  {
    outInfo("initialize");
    ctx.extractValue("test_param", cameraAngle);
    cameraAngle *= CV_PI / 180;
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  static float slope(Vec4i line)
  {
    if (line[2]==line[0])
    {
        return std::numeric_limits<float>::max();
    }
    return (line[3] - line[1]) / (line[2] - line[0]);
  }

  float lineDistance(Vec4i line)
  {
    Point p1 (line[0],line[1]);
    Point p2 (line[2],line[3]);
    return abs(p1.x*p2.y-p1.y*p2.x)/norm(p1-p2);
  }

  float lineLength(Vec4i line)
  {
    Point p1 (line[0],line[1]);
    Point p2 (line[2],line[3]);
    return norm(p1-p2);
  }

  void lineDetect2D(Mat image)
  {
    Mat image_gray, edges;
    cvtColor(image, image_gray, CV_BGR2GRAY);
    GaussianBlur(image_gray, image_gray, Size(5, 5), 0, 0);

    Canny(image_gray, edges, 50, 120, 3);

    dilate(edges,edges,Mat());

    cvtColor(edges, final_line, CV_GRAY2BGR);

    vector<Vec4i> lines;
    HoughLinesP(edges, lines, 1, CV_PI / 180, 50, 100, 5);
    for (size_t i = 0; i < lines.size(); i++)
    {
      // remove short lines
      if (lineLength(lines[i]) < LENGTH_THRESHOLD || abs(slope(lines[i])-tan(cameraAngle)) > SLOPE_THRESHOLD)
      {
        lines[i--] = lines[lines.size() - 1];
        lines.resize(lines.size() - 1);
      }
    }
    outInfo("short lines removed");
    std::sort(lines.begin(), lines.end(), [](Vec4i l1, Vec4i l2)
              {
                return slope(l1) < slope(l2);
              });

    if (lines.size()==0)
    {
      return;
    }

    float  maxLength = 0, maxIndex = 0;

    for (size_t i = 0; i < lines.size() - 1; i++)
    {
      outInfo("Iterating: "<<i<<" out of "<<lines.size());
      Vec4i l1 = lines[i];
      Vec4i l2 = lines[i+1];

      // if parallel and "close", but not very close
      if (slope(l2) - slope(l1) < SLOPE_THRESHOLD &&
          abs(lineDistance(l1)-lineDistance(l2)) < LENGTH_THRESHOLD / 2 &&
          abs(lineDistance(l1)-lineDistance(l2)) > LENGTH_THRESHOLD / 10)
      {
        if (lineLength(l1) > maxLength)
        {
          maxLength = lineLength(l1);
          maxIndex = i;
        }
      }
    }
    line(final_line, Point(lines[maxIndex][0], lines[maxIndex][1]), Point(lines[maxIndex][2], lines[maxIndex][3]),
         Scalar(0, 0, 255), 2, CV_AA);
    line(final_line, Point(lines[maxIndex+1][0], lines[maxIndex+1][1]), Point(lines[maxIndex+1][2], lines[maxIndex+1][3]),
         Scalar(0, 0, 255), 2, CV_AA);

    geometry_msgs::Polygon poly;
    geometry_msgs::Point32 point1,point2;
    point1.x = lines[maxIndex][0];
    point1.y = lines[maxIndex][1];

    point2.x = lines[maxIndex][2];
    point2.y = lines[maxIndex][3];

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

    lineDetect2D(image);

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
