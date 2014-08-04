#ifndef GESTURE_HPP_INCLUDED
#define GESTURE_HPP_INCLUDED

#define DEBUG_FLAG

#include <ros/ros.h>
#include <opencv/cv.h>
#include <iostream>
#include <cmath>
#include <cstdio>
#include <map>
#include <string>

namespace GestureTest
{

    const double pi = acos(-1.0);

    struct Vector
    {
        double x, y;
        double length;
        Vector()
        {
            x = y = length = 0;
        }
        Vector(double _x, double _y)
        {
            x = _x;
            y = _y;
            length = sqrt(_x*_x + _y*_y);
        }
        friend double operator * ( Vector v1, Vector v2 )// * 点乘 point production
        {
            return v1.x*v2.x + v1.y*v2.y;
        }
        friend double operator ^ ( Vector v1, Vector v2 )// ^ 叉积 fox production
        {
            return v1.x*v2.y - v1.y*v2.x;
        }
    };

    struct points
    {
        int body_id;
        std::map<std::string, cv::Point2d> p2d;
        std::map<std::string, cv::Point3d> p3d;
        points()
        {
            body_id = -1;
        }
        points ( const points & that )
        {
            body_id = that.body_id;
            p2d = that.p2d;
            p3d = that.p3d;
        }
        points( int _body_id, std::map<std::string, cv::Point2d> _p2d, std::map<std::string, cv::Point3d> _p3d )
        {
            body_id = _body_id;
            p2d = _p2d;
            p3d = _p3d;
        }
        ~points()
        {
            p2d.clear();
            p3d.clear();
        }
    };

    enum GestureStatus
    {
        NONE,//ARMS_DOWN
        L_ARM_UP,//R_ARM_DOWN
        R_ARM_UP,//L_ARM_DOWN
        ARMS_UP,
        L_HIGH,//R_ARM_DOWN
        R_HIGH,//L_ARM_DOWN
        ARMS_HIGH,
        L_ARM_LINE,//R_ARM_DOWN
        R_ARM_LINE,//L_ARM_DOWN
        ARMS_LINE,
        NYA//にゃ
    };
    const std::string body_gestureID[] =
    {
        "NONE",//ARMS_DOWN
        "L_ARM_UP",//R_ARM_DOWN
        "R_ARM_UP",//L_ARM_DOWN
        "ARMS_UP",
        "L_HIGH",//R_ARM_DOWN
        "R_HIGH",//L_ARM_DOWN
        "ARMS_HIGH",
        "L_ARM_LINE",//R_ARM_DOWN
        "R_ARM_LINE",//L_ARM_DOWN
        "ARMS_LINE",
        "NYA"//にゃ
    };
    const char body_picture[][12] =
    {
        //┌ └ ┐ ┘ ─ │ ├ ┤ ┬ ┴ ┼
        " O \nq+p\n A ",//NONE,//ARMS_DOWN
        " O \nd+p\n A ",//L_ARM_UP,//R_ARM_DOWN
        " O \nq+b\n A ",//R_ARM_UP,//L_ARM_DOWN
        " O \nd+b\n A ",//ARMS_UP,
        "|O \nd+p\n A ",//L_HIGH,//R_ARM_DOWN
        " O|\nq+b\n A ",//R_HIGH,//L_ARM_DOWN
        "|O|\nd+b\n A ",//ARMS_HIGH,
        " O \n-+p\n A ",//L_ARM_LINE,//R_ARM_DOWN
        " O \nq+-\n A ",//R_ARM_LINE,//L_ARM_DOWN
        " O \n-+-\n A ",//ARMS_LINE,
        " O \n + \n A "//NYA//にゃ
    };

    bool nya_flag = true;
    const std::string body_FrameID[] =
    {
        "head",//0
        "neck",//1
        "torso",//2
        "left_shoulder",//3
        "left_elbow",//4
        "left_hand",//5
        "right_shoulder",//6
        "right_elbow",//7
        "right_hand",//8
        "left_hip",//9
        "left_knee",//10
        "left_foot",//11
        "right_hip",//12
        "right_knee",//13
        "right_foot"//14
    };

    inline double degree( double _rad )
    {
        return _rad / pi * 180.0;
    }

    inline double rad( double _degree )
    {
        return _degree / 180.0 * pi;
    }

    double GetAngle( cv::Point2d left, cv::Point2d center, cv::Point2d right )
    {
        Vector
            v1(center.x-left.x, center.y-left.y),
            v2(center.x-right.x, center.y-right.y);
        double angle = acos( ( v1*v2 ) / ( v1.length * v2.length ) );  // * 点乘 point production
        double ret = degree( angle );
        if ( ( v1 ^ v2 ) > 0 )  // ^ 叉积 fox production
        {
            ret = 360.0 - ret;
        }
        std::cout<<ret<<std::endl;
        return ret;
    }

    inline bool check( double out, double ans, double fix )
    {
        if ( ans - fix <= out && out <= ans + fix )
        {
            return true;
        }
        else return false;
    }

    GestureStatus GetGestureStatus(points p)
    {
        /*
        *    "head",//0"neck",//1"torso",//2
        *    "left_shoulder",//3"left_elbow",//4"left_hand",//5
        *    "right_shoulder",//6"right_elbow",//7"right_hand",//8
        *    "left_hip",//9"left_knee",//10"left_foot",//11
        *    "right_hip",//12"right_knee",//13"right_foot"//14
        */
        double
            ang_L_shoulder = GetAngle(p.p2d[body_FrameID[6]], p.p2d[body_FrameID[3]], p.p2d[body_FrameID[4]]),
            ang_R_shoulder = GetAngle(p.p2d[body_FrameID[7]], p.p2d[body_FrameID[6]], p.p2d[body_FrameID[3]]),
            ang_L_elbow = GetAngle(p.p2d[body_FrameID[3]], p.p2d[body_FrameID[4]], p.p2d[body_FrameID[5]]),
            ang_R_elbow = GetAngle(p.p2d[body_FrameID[8]], p.p2d[body_FrameID[7]], p.p2d[body_FrameID[6]]);
        const double fix = 25.0;
        //L_ARM_UP,//R_ARM_DOWN
        if (
            check(ang_L_shoulder, 180, fix) && check(ang_L_elbow, 90, fix) &&
            check(ang_R_shoulder, 270, fix) && check(ang_R_elbow, 180, fix)
        )
        {
            GestureStatus ret = L_ARM_UP;
            ROS_INFO("[gesture] : %s\n%s", body_gestureID[ret].c_str(), body_picture[ret]);

            return ret;
        }
        //R_ARM_UP,//L_ARM_DOWN
        if (
            check(ang_L_shoulder, 270, fix) && check(ang_L_elbow, 180, fix) &&
            check(ang_R_shoulder, 180, fix) && check(ang_R_elbow, 90, fix)
        )
        {
            GestureStatus ret = R_ARM_UP;
            ROS_INFO("[gesture] : %s\n%s", body_gestureID[ret].c_str(), body_picture[ret]);
            return ret;
        }
        //ARMS_UP,
        if (
            check(ang_L_shoulder, 180, fix) && check(ang_L_elbow, 90, fix) &&
            check(ang_R_shoulder, 180, fix) && check(ang_R_elbow, 90, fix)
        )
        {
            GestureStatus ret = ARMS_UP;
            ROS_INFO("[gesture] : %s\n%s", body_gestureID[ret].c_str(), body_picture[ret]);
            return ret;
        }
        //L_HIGH,//R_ARM_DOWN
        if (
            check(ang_L_shoulder, 90, fix) && check(ang_L_elbow, 180, fix) &&
            check(ang_R_shoulder, 270, fix) && check(ang_R_elbow, 180, fix)
        )
        {
            GestureStatus ret = L_HIGH;
            ROS_INFO("[gesture] : %s\n%s", body_gestureID[ret].c_str(), body_picture[ret]);
            return ret;
        }
        //R_HIGH,//L_ARM_DOWN
        if (
            check(ang_L_shoulder, 270, fix) && check(ang_L_elbow, 180, fix) &&
            check(ang_R_shoulder, 90, fix) && check(ang_R_elbow, 180, fix)
        )
        {
            GestureStatus ret = R_HIGH;
            ROS_INFO("[gesture] : %s\n%s", body_gestureID[ret].c_str(), body_picture[ret]);
            return ret;
        }
        //ARMS_HIGH,
        if (
            check(ang_L_shoulder, 90, fix) && check(ang_L_elbow, 180, fix) &&
            check(ang_R_shoulder, 90, fix) && check(ang_R_elbow, 180, fix)
        )
        {
            GestureStatus ret = ARMS_HIGH;
            ROS_INFO("[gesture] : %s\n%s", body_gestureID[ret].c_str(), body_picture[ret]);
            return ret;
        }
        //L_ARM_LINE,//R_ARM_DOWN
        if (
            check(ang_L_shoulder, 180, fix) && check(ang_L_elbow, 180, fix) &&
            check(ang_R_shoulder, 270, fix) && check(ang_R_elbow, 180, fix)
        )
        {
            GestureStatus ret = L_ARM_LINE;
            ROS_INFO("[gesture] : %s\n%s", body_gestureID[ret].c_str(), body_picture[ret]);
            return ret;
        }
        //R_ARM_LINE,//L_ARM_DOWN
        if (
            check(ang_L_shoulder, 270, fix) && check(ang_L_elbow, 180, fix) &&
            check(ang_R_shoulder, 180, fix) && check(ang_R_elbow, 180, fix)
        )
        {
            GestureStatus ret = R_ARM_LINE;
            ROS_INFO("[gesture] : %s\n%s", body_gestureID[ret].c_str(), body_picture[ret]);
            return ret;
        }
        //ARMS_LINE,
        if (
            check(ang_L_shoulder, 180, fix) && check(ang_L_elbow, 180, fix) &&
            check(ang_R_shoulder, 180, fix) && check(ang_R_elbow, 180, fix)
        )
        {
            GestureStatus ret = ARMS_LINE;
            ROS_INFO("[gesture] : %s\n%s", body_gestureID[ret].c_str(), body_picture[ret]);
            return ret;
        }
        //NYA//にゃ
        //if (
        //    check(ang_L_shoulder, 90, fix) && check(ang_L_elbow, 180, fix) &&
        //    check(ang_R_shoulder, 270, fix) && check(ang_R_elbow, 180, fix)
        //)
        //{
            GestureStatus ret = NYA;
            ROS_INFO("[gesture] : %s\n%s", body_gestureID[ret].c_str(), body_picture[ret]);
            return ret;
        //}
    }

}

#endif // GESTURE_HPP_INCLUDED
