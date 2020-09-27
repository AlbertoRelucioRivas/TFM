#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_painter2D.h>
#include <std_msgs/Int32.h>
#include <list>

class cloudHandler
{
public:
    cloudHandler()
    {
        pcl_sub = nh.subscribe("pcl_downsampled", 10, &cloudHandler::frontCloud, this);
        pcl_front_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_front", 1);
        pcl_stairs_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_stairs", 1);
        inclination_pub = nh.advertise<std_msgs::Int32>("stairs_inclination", 1);
        polygon_pub = nh.advertise<jsk_recognition_msgs::PolygonArray>("polygon", 1);
        stairs_distance = nh.advertise<std_msgs::Int32>("stairs_distance", 1);
    }

    void frontCloud(const sensor_msgs::PointCloud2 &input)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud_in;
        pcl::PointCloud<pcl::PointXYZRGB> cloud_front;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        sensor_msgs::PointCloud2 front;
        // pack r/g/b into rgb
        uint8_t r = 255, g = 0, b = 255;    // Example: Red color
        uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);

        pcl::fromROSMsg(input, cloud_in);

        copyPointCloud(cloud_in, cloud_front);

        for (size_t i = 0; i < cloud_in.points.size (); ++i)
        {
            if(-0.2f < cloud_in.points[i].y && cloud_in.points[i].y < 0.2f && 0.0f < cloud_in.points[i].x && -0.3f < cloud_in.points[i].z){
                cloud_front.points[i].x = cloud_in.points[i].x;
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

        bool stairsDetected = stairsDetection(cloud_front);

        if(stairsDetected != stairsDetectedOld && stairsDetected)
        {
            ROS_INFO("Stairs detected with %d steps and %d degrees of inclination", n_steps, inclination);
        }
        stairsDetectedOld = stairsDetected;

        pcl::toROSMsg(cloud_front, front);
        front.header.frame_id = "top";
        pcl_front_pub.publish(front);
    }

    bool stairsDetection(const pcl::PointCloud<pcl::PointXYZRGB> &cloud_front)
    {
        pcl::PointCloud<pcl::PointXYZRGB> cloud_stairs;
        sensor_msgs::PointCloud2 stairs;
        std_msgs::Int32 distance;
        std_msgs::Int32 inclination_msg;
        stairs.header.frame_id = "top";
        cloud_stairs = cloud_front;
        jsk_recognition_msgs::PolygonArray PolygonArray;
        PolygonArray.header.frame_id = "top";

        float xi(0.0f), yi(0.0f), zi(0.0f), x_init(0.0f), y_init(0.0f), z_init(0.0f),
                ye(0.0f), ze(0.0f), z_prev(0.0f), h(0.0f), w(0.0f), l(0.0f), htotal(0.0f), wtotal(0.0f);
        int np(0);
        uint8_t r = 255, g = 0, b = 255;
        uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);

        for (size_t i = 0; i < cloud_front.points.size (); ++i)
        {
            xi = cloud_stairs.points[i].x;
            yi = cloud_stairs.points[i].y;
            zi = cloud_stairs.points[i].z;
            cloud_stairs.points[i].rgb = *reinterpret_cast<float*>(&rgb);
            if(x_init == 0){
                x_init = xi;
                z_init = zi;
                z_prev = zi;
                y_init = yi;
                stairsDistance((int)(x_init*100));
                distance.data = mean;
                stairs_distance.publish(distance);
            }
            else if(fabs(xi-x_init) <= 0.1f) {
                ye = yi;
                ze = zi;
                ++np;
            }
            else if(np > 30){
                //ROS_INFO("np: %d",np);
                h = ze - z_prev;
                htotal = htotal + h;
                l = ye - y_init;
                //ROS_INFO("l: %f",l);
                //ROS_INFO("h: %f",h);
                //ROS_INFO("n_steps: %d",n_steps);
                if(0.07f < h && h < 0.2f){
                    w = xi - x_init;
                    wtotal = wtotal + w;
                    //ROS_INFO("w: %f",w);
                    if(0.07f < w && w < 0.7f && l > 0.0f){
                        addVerticalRect(x_init,y_init,z_prev,h,l, PolygonArray);
                        addHorizontalRect(xi,yi,ze,w,l,PolygonArray);
                        inclination = (int)((atan(htotal/wtotal)*180)/M_PI);
                        inclination_msg.data = inclination;
                        inclination_pub.publish(inclination_msg);
                        ++n_steps;
                        r = 255*(((n_steps+1)%2)%2), g = 255*((n_steps%2)%2);
                        rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
                        cloud_stairs.points[i].rgb = *reinterpret_cast<float*>(&rgb);
                        if(n_steps > 2){
                            polygon_pub.publish(PolygonArray);
                            return true;
                        }
                    }
                    else {
                        x_init = xi;
                        n_steps = 0;
                        polygon_pub.publish(PolygonArray);
                    }
                }
                x_init = xi, z_init = zi, y_init = yi, np = 0, z_prev = ze;
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
        Polygon.header.frame_id = "top";
        addPointPolygon(x,y,z,Polygon);
        addPointPolygon(x,y+l,z,Polygon);
        addPointPolygon(x,y+l,z+h,Polygon);
        addPointPolygon(x,y,z+h,Polygon);
        PolygonArray.polygons.push_back(Polygon);
    }

    void addHorizontalRect(float x, float y, float z, float w, float l, jsk_recognition_msgs::PolygonArray& PolygonArray){
        geometry_msgs::PolygonStamped Polygon;
        Polygon.header.frame_id = "top";
        addPointPolygon(x,y,z,Polygon);
        addPointPolygon(x-w,y,z,Polygon);
        addPointPolygon(x-w,y+l,z,Polygon);
        addPointPolygon(x,y+l,z,Polygon);
        PolygonArray.polygons.push_back(Polygon);
    }

    void stairsDistance(int x){

        int sum = 0;
        if(list.size() < 10){
            list.push_front(x);
        }
        else {
            list.push_front(x);
            list.pop_back();
        }
        //std::cout << "Size of list " << list.size() << std::endl;
        for (std::list<int>::iterator it = list.begin(); it != list.end(); ++it) {
            //std::cout << "list " << sum << std::endl;
            sum += *it;
        }
        mean = sum/list.size();
    }

protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    ros::Publisher pcl_front_pub;
    ros::Publisher pcl_stairs_pub;
    ros::Publisher polygon_pub;
    ros::Publisher stairs_distance;
    ros::Publisher inclination_pub;

private:
    std::list<int> list;
    int mean;
    int inclination;
    int n_steps;
    bool stairsDetectedOld;
};

main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_stairs");

    cloudHandler handler;

    ros::spin();

    return 0;
}
