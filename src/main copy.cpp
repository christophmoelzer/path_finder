#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include "opencv2/opencv.hpp"

struct info{
    int x;
    int y;
    int dist;
};

// Check if Node is free
bool checkFree(cv::Point point, cv::Mat map){
    if(map.at<uchar>(point.x,point.y) > 0){
        //std::cout << (int)map.at<uchar>(point.x,point.y) << std::endl;
        return true;
    }
    else{
        //std::cout << "Point is not free" << std::endl;
        return false;
    }
}

// Check if Node is Destination
bool checkDestination(cv::Point point, cv::Point dest){
    if(point == dest){
        std::cout << "Point is destination" << std::endl;
        return true;
    }
    else{
        
        return false;
    }
}

// Calculate Distance
int calcDistance(cv::Point point, cv::Point dest){
    int dist = (abs(point.x-dest.x)+abs(point.y-dest.y));
    //std::cout << "dist = " << dist << std::endl;

    return dist;
}


// Check if Node is valid
bool checkValid(cv::Point point, cv::Point origin, cv::Mat map){
    if (point.x >= 0 && point.x < map.cols && point.y >= 0 && point.y < map.rows && point!=origin){
        return true;
    }
    else{
        //std::cout << "Point is not valid" << std::endl;
        return false;
    }

}

bool checkLastPoints(cv::Point point, std::vector<info> last_points){
    for(int i=0; i<last_points.size(); i++){
        if(point.x == last_points[i].x && point.y == last_points[i].y){
            return false;
        }
    }
    return true;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "talker");

    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

    ros:: Rate loop_rate(1);

    std::cout << "foo bar" << std::endl;

    int count = 0;

    cv::Point start(4,4);
    cv::Point destination(4,25);

    std::cout << start << std::endl;
    std::cout << destination << std::endl;
    
    cv::namedWindow("map",0);

    while(ros::ok() && count==0){
        std_msgs::String msg;

        std::stringstream ss;
        ss << "foo" << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

        chatter_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;

       
        cv::Mat map = cv::imread("/home/chris/Desktop/map.png");
        cv::cvtColor(map, map, CV_BGR2GRAY);
        map.convertTo(map, CV_8U);

        if(map.rows <= 0){
            std::cout << "Your picture seems to be not pretty enough." << std::endl;
            return -1;
        }
        map.at<uchar>(start.x,start.y)=50;
        map.at<uchar>(destination.x,destination.y)=50;
        
        ROS_INFO_STREAM("SIZE = " << map.rows << " / " << map.cols );

        cv::Point newPoint=start;
        cv::Point checkPoint;
        cv::Point offset;
        
        std::vector<info> foo;
        std::vector<info> last_points;
        info temp;
        info min_val;
        temp.x = 0;
        temp.y = 0;
        temp.dist = 0;
        min_val.x=start.x;
        min_val.y=start.y;
        min_val.dist=-1;
        int shade=100;

        //last_points.push_back(temp);
        bool searching = true;
        int search_counter = 0;
        int idx_new_start_point = 0;

        //for(int cycles=0; cycles<40; cycles++){
        while(searching && search_counter<2000){
            ++search_counter;
            int valid_points = 8;

            // Check neighbour points
            for(offset.x=-1; offset.x<=1; offset.x++){
                for(offset.y=-1; offset.y<=1; offset.y++){
                    checkPoint=newPoint+offset;
                    
                    if(checkValid(checkPoint, newPoint, map)){  // The point has to be inside the map
                        
                        if(!checkDestination(checkPoint,destination)){  // The point is not the destination
                            
                            if(checkFree(checkPoint,map)&&checkLastPoints(checkPoint,last_points)){ // The point is free and a new one
                                //std::cout << checkPoint << std::endl;
                                temp.x = checkPoint.x;
                                temp.y = checkPoint.y;
                                temp.dist = calcDistance(checkPoint,destination);
                                //std::cout << "x/y/dist " << temp.x << "/" << temp.y << "/" << temp.dist << std::endl;

                                foo.push_back(temp); // Push the all valid points to the list
                            }
                            else{
                                --valid_points;
                                if (valid_points <= 0){
                                    std::cout << "should not happen" << std::endl;
                                    std::cout << "last x/y/dist " << temp.x << "/" << temp.y << "/" << temp.dist << std::endl;
                                    // Set new start point
                                    if (last_points.size() > 0){
                                        min_val = last_points[idx_new_start_point];
                                    }
                                    else{
                                        std::cout << "hmmmmmmm" << std::endl;
                                    }
                                    std::cout << "new x/y/dist " << last_points[idx_new_start_point].x << "/" << last_points[idx_new_start_point].y << "/" << last_points[idx_new_start_point].dist << std::endl;
                                    ++idx_new_start_point;
                                    std::cout << "new idx = " << idx_new_start_point << std::endl;
                                }
                            }
                        }
                        else{   // The point is the destination
                            searching = false;
                            break;
                        }
                    }
                }
            }

            
            /*min_val.x=start.x;
            min_val.y=start.y;*/
            min_val.dist=-1;
            //std::cout << "size of foo = " << foo.size() << std::endl;

            // Find the point with the minimal distance to the destination
            if (foo.size()>0){
                for(int search=0; search<foo.size(); search++){
                    if(min_val.dist==-1){
                        min_val=foo[search];
                    }
                    if(foo[search].dist < min_val.dist){
                        min_val=foo[search];
                    }
                }
                foo.clear();
                last_points.push_back(min_val);     // Push the point with the minimum distance to the list of the used points
                std::cout << "show x/y/dist " << last_points[last_points.size()-1].x << "/" << last_points[last_points.size()-1].y << "/" << last_points[last_points.size()-1].dist << std::endl;
                                    
            }
            else{
                ;//searching = false;
            }
            
            newPoint.x = min_val.x;
            newPoint.y = min_val.y;
            if (map.at<uchar>(min_val.x,min_val.y) == 200){
                map.at<uchar>(min_val.x,min_val.y)=180;
            }
            else if(map.at<uchar>(min_val.x,min_val.y) == 180){
                map.at<uchar>(min_val.x,min_val.y)=160;
            }
            else if(map.at<uchar>(min_val.x,min_val.y) == 160){
                map.at<uchar>(min_val.x,min_val.y)=140;
            }
            else {
                map.at<uchar>(min_val.x,min_val.y)=200;
            }
            std::cout << "min --> x/y/dist " << min_val.x << "/" << min_val.y << "/" << min_val.dist << std::endl;
            cv::imshow("map", map);
            //cv::waitKey(0);
        }
        for(int i=0; i<last_points.size(); i++){
            std::cout << "last points " << i << " -> " << last_points[i].x << std::endl;
        }
    }
    cv::waitKey(0);
    return 0;
}