#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include "opencv2/opencv.hpp"

struct weighted_point{
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

bool checkLastPoints(cv::Point point, std::vector<std::vector<weighted_point>> last_points){
    if (last_points.size() > 0){
        for(int i=0; i<last_points.size(); i++){
            for(int j=0; j<last_points[i].size(); j++){
                if(point.x == last_points[i][j].x && point.y == last_points[i][j].y){
                    return false;
                }
            }
        }
    }
    return true;
}

void printVector(std::string text,std::vector<std::vector<weighted_point>> vec){
    for(int i=0; i<vec.size(); i++){
        for(int j=0; j<vec[i].size(); j++){
            std::cout << text << " " << i << "/" << j << " --> x/y/dist = " << vec[i][j].x << "|" << vec[i][j].y << "|" << vec[i][j].dist << std::endl;
        }
    }
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
        
        std::vector<weighted_point> foo;
        std::vector<weighted_point> last_points;
        std::vector<weighted_point> outer_boarder;
        std::vector<weighted_point> around_point;
        std::vector<std::vector<weighted_point>> point_search;
        std::vector<std::vector<weighted_point>> new_point_search;
        std::vector<std::vector<weighted_point>> last_point_search;

        weighted_point temp;
        weighted_point min_val;
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

        around_point.clear();
        temp.x = start.x;
        temp.y = start.y;
        around_point.push_back(temp);
        point_search.push_back(around_point);

        //for(int cycles=0; cycles<40; cycles++){
        while(searching && search_counter<2000){
            ++search_counter;
            

            // Breath search
            for(int i=0; i<point_search.size(); i++){
                
                for(int j=0; j<point_search[i].size(); j++){
                    newPoint.x=point_search[i][j].x;
                    newPoint.y=point_search[i][j].y;

                    std::cout << "new point " << i << " -> " << newPoint.x << "/" << newPoint.y << std::endl;
                    int valid_points = 8;
                    // Check neighbour points
                    for(offset.x=-1; offset.x<=1; offset.x++){
                        for(offset.y=-1; offset.y<=1; offset.y++){
                            checkPoint=newPoint+offset;
                            if(checkValid(checkPoint, newPoint, map)){  // The point has to be inside the map
                                if(!checkDestination(checkPoint,destination)){  // The point is not the destination
                                    if(checkFree(checkPoint,map)&&checkLastPoints(checkPoint,last_point_search)){ // The point is free and a new one
                                        temp.x = checkPoint.x;
                                        temp.y = checkPoint.y;
                                        temp.dist = calcDistance(checkPoint,destination);
                                        around_point.push_back(temp);
                                        map.at<uchar>(temp.x,temp.y)=100;
                                    }
                                    else{
                                        --valid_points;
                                        if (valid_points <= 0){
                                            std::cout << "should not happen" << std::endl;
                                            std::cout << "last x/y/dist " << temp.x << "/" << temp.y << "/" << temp.dist << std::endl;
                                            
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
                    new_point_search.push_back(around_point);
                }
            }
            //printVector("new",new_point_search);
            point_search.clear();
            point_search = new_point_search;
            last_point_search.insert(last_point_search.end(),new_point_search.begin(),new_point_search.end());
            //printVector("last",last_point_search);
            new_point_search.clear();
            
            cv::imshow("map", map);
            cv::waitKey(0);
        }
        
    }
    cv::waitKey(0);
    return 0;
}