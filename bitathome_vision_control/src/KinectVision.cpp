#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <map>
#include <string>
#include <iostream>

#include "gesture.hpp"

using namespace std;

class KinectVision
{

    ros::NodeHandle _nh;
    image_transport::ImageTransport _it;
    image_transport::Publisher _pub_skeleton, _pub_image;
    image_transport::CameraSubscriber _sub;
    image_geometry::PinholeCameraModel _CamMod;
    tf::TransformListener _listener;
    int body_status[10];
    static const int PointRadio = 7;

public:
    // __init__()
    KinectVision( )
        : _it(_nh)
    {
        memset(body_status,0,sizeof(body_status));
        bool flag = true;
        while ( flag ) try
        {
            //init _pub
            ROS_INFO("initializing publisher ...");
            _pub_skeleton = _it.advertise("KinectVision/skeleton_image", 1);
            _pub_image = _it.advertise("KinectVision/image", 1);
            //init _sub
            ROS_INFO("initializing subscriber ...");
            string NodeTopic = _nh.resolveName("camera/rgb/image_color");
            _sub = _it.subscribeCamera(NodeTopic, 1, &KinectVision::img_CB, this);
            flag = false;
        }
        catch ( exception & e )
        {
        }
    }

    // _sub::callback
    void img_CB(
        const sensor_msgs::ImageConstPtr & img_msg,
        const sensor_msgs::CameraInfoConstPtr & info_msg
    )
    {
        cv::Mat image;
        cv_bridge::CvImagePtr input_bridge;
        try
        {
            input_bridge = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
            image = input_bridge->image;
            _pub_image.publish(img_msg);
        }
        catch ( cv_bridge::Exception & e )
        {
            ROS_ERROR("[get]: convertion failed!");
            return ;
        }

        _CamMod.fromCameraInfo(info_msg);

        vector<string> frame_ids;
        _listener.getFrameStrings(frame_ids);
        GestureTest::points points;
        bool flagHasSkeleton = false;
        BOOST_FOREACH( const string & frame_id, frame_ids )
        {
            tf::StampedTransform _transform;
            int body_id = -1;
            string body_name;

            if ( useless(frame_id, body_id, body_name) )
            {
                continue;
            }
            flagHasSkeleton = true;
            try
            {
                _listener.lookupTransform("openni_depth_frame", frame_id, ros::Time(0), _transform);
            }
            catch ( tf::TransformException & e )
            {
                ROS_ERROR("[tf]: %s", e.what());
                break ;
            }

            ros::Time ReceivedTime = _transform.stamp_;
            ros::Time CurrentTime = ros::Time::now();
            if ( CurrentTime.sec - ReceivedTime.sec > 1 )
            {
                if ( -1 != body_status[body_id] )
                {
                    ROS_WARN("skeleton [id=%d] lost", body_id);
                    body_status[body_id] = -1;
                }
                break;
            }

            switch ( body_status[body_id] )
            {
            case 0:
                ROS_INFO("get new skeleton [id=%d]", body_id);
                body_status[body_id] = 1;
                break;
            case -1:
                ROS_INFO("found old skeleton [id=%d]", body_id);
                body_status[body_id] = 1;
                break;
            }

            tf::Point pt = _transform.getOrigin();
            cv::Point3d pt_cv(-pt.y(), -pt.z(), pt.x());
            cv::Point2d uv;
            uv = _CamMod.project3dToPixel(pt_cv);

            addPoint(image, uv, body_id);

            points.body_id = body_id;
            points.p2d[body_name] = uv;
            points.p3d[body_name] = pt_cv;
        }
        _pub_skeleton.publish(input_bridge->toImageMsg());
        cv::imshow("KinectVision", image);
        cv::waitKey(3);
        if ( flagHasSkeleton ) GestureTest::GetGestureStatus(points);

    }
    void addPoint( cv::Mat & image, cv::Point2d uv, int body_id )
    {
        const CvScalar SkeletonColor[] =
        {
            CV_RGB(255,255,255),
            CV_RGB(204,51,255),
            CV_RGB(0,255,0),
            CV_RGB(255,255,0),
            CV_RGB(0,255,255),
            CV_RGB(255,0,0),
            CV_RGB(0,0,255),
            CV_RGB(255,0,255),
            CV_RGB(255,153,0),
            CV_RGB(102,51,51),
            CV_RGB(51,153,51)
        };
        cv::circle(image, uv, PointRadio, SkeletonColor[body_id], -1);
    }
    // check frame is useless or not
    bool useless( string _buf, int & id, string & name )
    {
        for ( int i = 0; i < 15; i ++ )
        {
            if ( _buf.substr(0,_buf.size()-2) == GestureTest::body_FrameID[i] )
            {
                id = _buf[_buf.size()-1] - '0';
                name = GestureTest::body_FrameID[i];
                return false;
            }
        }
        id = -1;
        name = "";
        return true;
    }
};


int main( int argc, char ** argv )
{
    ros::init(argc, argv, "KinectVision");
    ROS_INFO("starting node /KinectVision...");
    KinectVision kv;
    ros::spin();
    return 0;
}
