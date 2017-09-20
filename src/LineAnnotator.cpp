#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/norms.h>
#include <rs/types/all_types.h>
// RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/DrawingAnnotator.h>

// ros
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

//
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <math.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>

#define LENGTH_THRESHOLD 0.8f
#define DISTANCE_THRESHOLD 30

using namespace uima;
using namespace cv;
using namespace std;

class LineAnnotator : public DrawingAnnotator
{
private:
  int croppedDistance = 20;
  int lastMessageIndex = 0;
  Mat final_line;
  ros::NodeHandle nh;
  ros::Publisher pubLines, pubNormal;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
  pcl::PointCloud<pcl::Normal>::Ptr cloudNormals;
  geometry_msgs::Point32 point1,point2;
  sensor_msgs::CameraInfo cameraInfo;

public:
  LineAnnotator()
      : DrawingAnnotator(__func__), nh("~")
  {
    pubLines = nh.advertise<geometry_msgs::PolygonStamped>("ShelfSegment",5);
    pubNormal = nh.advertise<geometry_msgs::PointStamped>("ShelfNormal",5);
    cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(
        new pcl::PointCloud<pcl::PointXYZRGBA>);
    cloudNormals = pcl::PointCloud<pcl::Normal>::Ptr(
        new pcl::PointCloud<pcl::Normal>);
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

  static float lineDistance(Vec4i line)
  {
    Point p1 (line[0],line[1]);
    Point p2 (line[2],line[3]);
    return abs(p1.x*p2.y-p1.y*p2.x)/norm(p1-p2);
  }

  static float lineDistance(geometry_msgs::PolygonStamped line)
  {
     geometry_msgs::Point32 p1 = line.polygon.points[0];
     geometry_msgs::Point32 p2 = line.polygon.points[1];
     return lineDistance(p1,p2);
  }

  static float lineDistance(geometry_msgs::Point32 p1, geometry_msgs::Point32 p2)
  {
    return abs(p1.x*p2.y-p1.y*p2.x)/sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y));
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

    cvtColor(image_gray, image_gray, CV_BGR2GRAY);
    GaussianBlur(image_gray, image_gray, Size(5, 5), 0, 0);
    Canny(image_gray, edges, 120, 200, 3);
    dilate(edges,image,Mat(),Point(-1,-1),2);

    findContours(image,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);

    cvtColor(image, final_line, CV_GRAY2BGR);
    drawContours(final_line,contours,-1,Scalar(255,0,255),3);
    cvtColor(final_line,image,CV_BGR2GRAY);
  }

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
      if (lineLength(lines[i]) < lineLength(lines[0])*LENGTH_THRESHOLD)
      {
        lines.resize(i);
        break;
      }
      line(final_line, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]),
           Scalar(0, 255, 0), 2, CV_AA);
    }

    std::sort(lines.begin(), lines.end(), [](Vec4i l1, Vec4i l2)
                  {
                    return lineDistance(l1) > lineDistance(l2);
                  });

    vector<geometry_msgs::PolygonStamped> lastShelves;
    vector<geometry_msgs::PointStamped> lastNormals;
//    if (lastMessageIndex > 0)
//    {
//      rosbag::Bag bag ("messages.bag",rosbag::bagmode::Read);
//      vector<string> topics;
//      topics.push_back("Shelf"+(lastMessageIndex-1));
//      topics.push_back("Normal"+(lastMessageIndex-1));
//      rosbag::View view (bag,rosbag::TopicQuery(topics));

//      BOOST_FOREACH(rosbag::MessageInstance const m, view)
//      {
//        geometry_msgs::PolygonStampedConstPtr p = m.instantiate<geometry_msgs::PolygonStamped>();
//        if (p!= NULL)
//        {
//          lastShelves.push_back(*p);
//        }

//        geometry_msgs::PointStampedConstPtr point = m.instantiate<geometry_msgs::PointStamped>();
//        if (point!= NULL)
//        {
//          lastNormals.push_back(*point);
//        }
//      }
//    }
//    rosbag::Bag bag ("messages.bag",rosbag::bagmode::Write);

    for (size_t i =  0; i < lines.size();i++)
    {
      if (i != 0 && abs(lineDistance(lines[i])-lineDistance(lines[i-1])) < DISTANCE_THRESHOLD)
      {
        continue;
      }

      geometry_msgs::PolygonStamped lineMessage;

      line(final_line, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]),
           Scalar(0, 0, 255), 2, CV_AA);

      int dx=-2,dy=-4;
      do
      {
        dy++;
        if (dy == 3)
        {
          dy = -3;
          dx++;
        }
        point1.x = cloud->at(lines[i][0]+croppedDistance+dx,lines[i][1]+croppedDistance+dy).x;
        point1.y = cloud->at(lines[i][0]+croppedDistance+dx,lines[i][1]+croppedDistance+dy).y;
        point1.z = cloud->at(lines[i][0]+croppedDistance+dx,lines[i][1]+croppedDistance+dy).z;
      }while (std::isnan(point1.x));
      int point1CoordinateX = lines[i][0]+croppedDistance+dx;
      int point1CoordinateY = lines[i][1]+croppedDistance+dy;

      dx=2,dy=4;
      do
      {
        dy--;
        if (dy == -3)
        {
          dy = 3;
          dx--;
        }
        point2.x = cloud->at(lines[i][2]+croppedDistance+dx,lines[i][3]+croppedDistance+dy).x;
        point2.y = cloud->at(lines[i][2]+croppedDistance+dx,lines[i][3]+croppedDistance+dy).y;
        point2.z = cloud->at(lines[i][2]+croppedDistance+dx,lines[i][3]+croppedDistance+dy).z;
      }while (isnan(point2.x));
      int point2CoordinateX = lines[i][2]+croppedDistance+dx;
      int point2CoordinateY = lines[i][3]+croppedDistance+dy;

//      BOOST_FOREACH(geometry_msgs::PolygonStamped const poly, lastShelves)
//      {
//        if (abs(lineDistance(poly)-lineDistance(point1,point2))<0.2)//same line
//        {
//          if (poly.polygon.points[0].x<point1.x)
//          {
//            point1 = poly.polygon.points[0];
//          }
//          if (poly.polygon.points[1].x>point2.x)
//          {
//            point2 = poly.polygon.points[1];
//          }
//        }
//      }

      lineMessage.polygon.points.push_back(point1);
      lineMessage.polygon.points.push_back(point2);
      lineMessage.header.stamp = cameraInfo.header.stamp;
      pubLines.publish(lineMessage);

//      bag.write("Shelf"+lastMessageIndex,lineMessage.header.stamp,lineMessage);

      float normalX = 0, normalY = 0, normalZ = 0, nbOfPoints = 0;
      //normal averaging
      for(int x = std::min(point1CoordinateX,point2CoordinateX); x <= std::max(point1CoordinateX,point2CoordinateX); x++)
      {
        for(int y = std::min(point1CoordinateY,point2CoordinateY); y <= std::max(point1CoordinateY,point2CoordinateY); y++)
        {
          if(isnan(cloudNormals->at(x,y).normal_x))
          {
            continue;
          }
          normalX += cloudNormals->at(x,y).normal_x;
          normalY += cloudNormals->at(x,y).normal_y;
          normalZ += cloudNormals->at(x,y).normal_z;
          nbOfPoints++;
        }
      }

      geometry_msgs::PointStamped normal;
      normal.point.x = normalX/nbOfPoints;
      normal.point.y = normalY/nbOfPoints;
      normal.point.z = normalZ/nbOfPoints;
      normal.header.stamp = cameraInfo.header.stamp;
      pubNormal.publish(normal);
//      bag.write("Normal"+lastMessageIndex,normal.header.stamp,normal);
    }
//    bag.close();
    lastMessageIndex++;
  }

  TyErrorId processWithLock(CAS& tcas, ResultSpecification const& res_spec)
  {
    rs::SceneCas cas(tcas);
    Mat image;

    outInfo("process start");

    cas.get(VIEW_COLOR_IMAGE, image);
    cas.get(VIEW_CLOUD,*cloud);
    cas.get(VIEW_NORMALS,*cloudNormals);
    cas.get(VIEW_CAMERA_INFO,cameraInfo);

    lineDetect2D(image);
    return UIMA_ERR_NONE;
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer& visualizer,
                              const bool firstRun)
  {
    const std::string cloudname = "LineAnnotatorCloud";
    double pointSize = 1;

    if (firstRun)
    {
      visualizer.addPointCloud(cloud, cloudname);
      visualizer.setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
      visualizer.addSphere<pcl::PointXYZ>(pcl::PointXYZ(point1.x,point1.y,point1.z),0.01,"start");
      visualizer.addSphere<pcl::PointXYZ>(pcl::PointXYZ(point2.x,point2.y,point2.z),0.01,"end");
    }
    else
    {
      visualizer.updatePointCloud(cloud, cloudname);
      visualizer.getPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
      visualizer.updateSphere<pcl::PointXYZ>(pcl::PointXYZ(point1.x,point1.y,point1.z),0.01,0,0,255,"start");
      visualizer.updateSphere<pcl::PointXYZ>(pcl::PointXYZ(point2.x,point2.y,point2.z),0.01,255,0,0,"end");
    }
  }
  void drawImageWithLock(cv::Mat& disp) { disp = final_line; }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(LineAnnotator)
