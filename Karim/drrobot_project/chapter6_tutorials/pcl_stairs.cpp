#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_painter2D.h>

class cloudHandler
{
public:
    cloudHandler()
    {
        pcl_sub = nh.subscribe("pcl_downsampled", 10, &cloudHandler::frontCloud, this);
        pcl_front_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_front", 10);
        pcl_stairs_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_stairs", 1);
        polygon_pub = nh.advertise<jsk_recognition_msgs::PolygonArray>("polygon", 1);
    }

    void frontCloud(const sensor_msgs::PointCloud2 &input)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud_in;
        pcl::PointCloud<pcl::PointXYZRGB> cloud_front;

        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        sensor_msgs::PointCloud2 front;
        // pack r/g/b into rgb
        uint8_t r = 255, g = 0, b = 255;    // Example: Red color
        uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);

        pcl::fromROSMsg(input, cloud_in);

        for (int t=0; t<9;++t) {

            copyPointCloud(cloud_in, cloud_front);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
            //ROS_INFO("t: %d",t);
            for (size_t i = 0; i < cloud_in.points.size (); ++i)
            {
                if((-(M_PI/4) + t*(M_PI/4)) < atan(cloud_in.points[i].y/cloud_in.points[i].x) && atan(cloud_in.points[i].y/cloud_in.points[i].x) < ((M_PI/4) + t*(M_PI/4)) && -0.3f < cloud_in.points[i].z){
                    //cloud_front.points[i].x = cloud_in.points[i].x;
                    cloud_front.points[i].rgb = *reinterpret_cast<float*>(&rgb);
                }
                else {
                    inliers->indices.push_back(i);
                }
            }

            extract.setInputCloud(cloud_front.makeShared());
            extract.setIndices(inliers);
            extract.setNegative(true);
            extract.filter(cloud_front);

            //stairsDetection(cloud_front);

            pcl::toROSMsg(cloud_front, front);
            front.header.frame_id = "velodyne";
            front.header.stamp = ros::Time::now();
            pcl_front_pub.publish(front);
        }
    }

    bool stairsDetection(const pcl::PointCloud<pcl::PointXYZRGB> &cloud_front)
    {
        pcl::PointCloud<pcl::PointXYZRGB> cloud_stairs;
        sensor_msgs::PointCloud2 stairs;
        stairs.header.frame_id = "velodyne";
        cloud_stairs = cloud_front;
        jsk_recognition_msgs::PolygonArray PolygonArray;
        PolygonArray.header.frame_id = "velodyne";

        float xi(0.0f), yi(0.0f), zi(0.0f), xs(0.0f), ys(0.0f), zs(0.0f), ye(0.0f), ze(0.0f), zp(0.0f), h(0.0f), w(0.0f), l(0.0f), htotal(0.0f), wtotal(0.0f);
        int np(0), ns(0);
        uint8_t r = 255, g = 0, b = 255;
        uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);

        for (size_t i = 0; i < cloud_front.points.size (); ++i)
        {
            xi = cloud_stairs.points[i].x;
            yi = cloud_stairs.points[i].y;
            zi = cloud_stairs.points[i].z;
            cloud_stairs.points[i].rgb = *reinterpret_cast<float*>(&rgb);
            if(xs == 0){
                xs = xi;
                zs = zi;
                zp = zi;
                ys = yi;
            }
            else if(fabs(xi-xs) <= 0.1f) {
                ye = yi;
                ze = zi;
                ++np;
            }
            else if(np > 30){
                ROS_INFO("np: %d",np);
                h = ze - zp;
                htotal = htotal + h;
                l = ye - ys;
                //ROS_INFO("l: %f",l);
                if(0.07f < h && h < 0.3f){
                    w = xi - xs;
                    wtotal = wtotal + w;
                    ROS_INFO("w: %f",w);
                    if(0.1f < w && w < 0.4f){
                        addVerticalRect(xs,ys,zp,h,l, PolygonArray);
                        addHorizontalRect(xi,yi,ze,w,l,PolygonArray);
                        int inclination = (int)((atan(htotal/wtotal)*180)/M_PI);
                        ++ns;
                        r = 255*(((ns+1)%2)%2), g = 255*((ns%2)%2);
                        rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
                        cloud_stairs.points[i].rgb = *reinterpret_cast<float*>(&rgb);
                        if(ns > 3)
                            ROS_INFO("Stairs detected with %d steps and %d degrees of inclination", ns, inclination);
                            polygon_pub.publish(PolygonArray);
                            return true;
                    }
                    else {
                        xs = xi;
                    }
                }
                xs = xi, zs = zi, ys = yi, np = 0, zp = ze;
            }
        }
        pcl::toROSMsg(cloud_stairs, stairs);
        pcl_stairs_pub.publish(stairs);
        return false;
    }

    void addPointPolygon(float x, float y, float z, geometry_msgs::PolygonStamped& Polygon){
        geometry_msgs::Point32 PolygonPoint;
        PolygonPoint.x = x, PolygonPoint.y = y, PolygonPoint.z = z;
        Polygon.polygon.points.push_back(PolygonPoint);
    }

    void addVerticalRect(float x, float y, float z, float h, float l, jsk_recognition_msgs::PolygonArray& PolygonArray){
        geometry_msgs::PolygonStamped Polygon;
        Polygon.header.frame_id = "velodyne";
        addPointPolygon(x,y,z,Polygon);
        addPointPolygon(x,y+l,z,Polygon);
        addPointPolygon(x,y+l,z+h,Polygon);
        addPointPolygon(x,y,z+h,Polygon);
        PolygonArray.polygons.push_back(Polygon);
    }

    void addHorizontalRect(float x, float y, float z, float w, float l, jsk_recognition_msgs::PolygonArray& PolygonArray){
        geometry_msgs::PolygonStamped Polygon;
        Polygon.header.frame_id = "velodyne";
        addPointPolygon(x,y,z,Polygon);
        addPointPolygon(x-w,y,z,Polygon);
        addPointPolygon(x-w,y+l,z,Polygon);
        addPointPolygon(x,y+l,z,Polygon);
        PolygonArray.polygons.push_back(Polygon);
    }

protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    ros::Publisher pcl_front_pub;
    ros::Publisher pcl_stairs_pub;
    ros::Publisher polygon_pub;
};

main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_stairs");

    cloudHandler handler;
    ros::Rate loop_rate(0.01);
    while (ros::ok()){
        ros::spin();
        loop_rate.sleep();
    }


    return 0;
}
