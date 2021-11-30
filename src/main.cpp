#include "ros/ros.h"
#include "std_msgs/String.h"
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

bool checkLastPoints(cv::Point point, std::vector<cv::Point> last_points){
    if (last_points.size() > 0){
        for(int i=0; i<last_points.size(); i++){
            //std::cout << "x/y: " << point << std::endl;
            if(point.x == last_points[i].x && point.y == last_points[i].y){
                //std::cout << "already visited" << std::endl;
                return false;
            }
        }
    }
    return true;
}

void printVector(std::string text,std::vector<cv::Point> vec){
    for(int i=0; i<vec.size(); i++){
        //std::cout << text << " " << i << " --> x/y = " << vec[i] << std::endl;
        std::cout << i << "->" << vec[i] << "; ";
    }
}

void printVector(std::string text,std::vector<int> vec){
    for(int i=0; i<vec.size(); i++){
        std::cout << text << " " << i << " --> weigth = " << vec[i] << std::endl;
    }
}

int main(int argc, char **argv){

    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros:: Rate loop_rate(1);

    int cycles = 0;

    cv::Point start(4,4);
    cv::Point destination(4,24);
    cv::namedWindow("map",0);


    while(ros::ok() && cycles==0){
        
        /*std_msgs::String msg;
        std::stringstream ss;
        ss << "foo" << count;
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());
        chatter_pub.publish(msg);*/

        ros::spinOnce();
        loop_rate.sleep();
        

       
        cv::Mat map = cv::imread("/home/chris/Desktop/map.png");
        cv::cvtColor(map, map, CV_BGR2GRAY);
        map.convertTo(map, CV_8U);

        if(map.rows <= 0){
            std::cout << "Loading of map failed!" << std::endl;
            return -1;
        }

        cv::Mat weight_map = cv::Mat::zeros(map.rows,map.cols,CV_32S);
        for(int x=0; x<weight_map.rows; x++){
            for(int y=0; y<weight_map.cols; y++){
                weight_map.at<int>(x,y)=-1;
            }        
        }
        weight_map.at<int>(start.x,start.y)=0;

        // Mark start and end point on the map
        map.at<uchar>(start.x,start.y)=50;
        map.at<uchar>(destination.x,destination.y)=50;
        
        // Show size of map
        ROS_INFO_STREAM("SIZE = " << map.rows << " / " << map.cols );

        // Define two lists
        // Visited list
        std::vector<cv::Point> visited;
        std::vector<cv::Point> path;
        // Queue list
        std::vector<cv::Point> queue;
        std::vector<cv::Point> persistent_queue;
        // Weigth list
        std::vector<int> weigth;
        //
        std::vector<cv::Point> neighbours;
        neighbours.push_back({0,-1});
        neighbours.push_back({-1,0});
        neighbours.push_back({1,0});
        neighbours.push_back({0,1});

        bool is_destination = false;
        int temp_breath_cells=0;
        int breath_counter=0;
        int temp_cycle_counter=0;
        int old_queue_size=0;

        // The search starts at the start point
        visited.push_back(start);
        weigth.push_back(0);

        cv::Point offset;
        cv::Point temp_point;

        while((cycles < 1000) && not(is_destination)){
            
            
            // Search valid neighbour cells around the visited cell
            
            bool no_valid_point = true;
            int valid_counter = 0;
            
            //for(offset.x=-1; offset.x<=1; offset.x++){
            //    for(offset.y=-1; offset.y<=1; offset.y++){
            for(int idx=0; idx<neighbours.size(); idx++){
                temp_point=visited[visited.size()-1]+neighbours[idx];
                if(checkValid(temp_point, visited[visited.size()-1], map)){  // The point has to be inside the map
                    if(checkLastPoints(temp_point,visited)){
                        if(checkLastPoints(temp_point,queue)){
                            if(checkFree(temp_point,map)){
                                queue.push_back(temp_point);
                                persistent_queue.push_back(temp_point);
                                //std::cout << temp_point << std::endl;
                                valid_counter++;
                                map.at<uchar>(temp_point.x,temp_point.y)=100;
                                weight_map.at<int>(temp_point.x,temp_point.y)=breath_counter+1;
                                no_valid_point=false;
                                if(!checkDestination(temp_point,destination)){  // The point is not the destination
                                    
                                }
                                else{
                                    is_destination=true;
                                }
                            }
                        }
                    }
                }
            }
            //}

            // Breath counter
            //printVector("queue: ",queue);
            //std::cout << "new cells = " << valid_counter << std::endl;
            //std::cout << "temp cells = " << temp_cycle_counter << "/" << temp_breath_cells << std::endl;
            if(temp_cycle_counter>=temp_breath_cells){
                //std::cout << persistent_queue.size() << " - " << old_queue_size << " = ";
                temp_breath_cells = persistent_queue.size()-old_queue_size;
                //std::cout << temp_breath_cells << std::endl;
                //persistent_queue.erase(persistent_queue.begin(),persistent_queue.begin()+temp_breath_cells-1);
                old_queue_size = persistent_queue.size();
                temp_cycle_counter=0;
                breath_counter++;
                //std::cout << "new breath: " << breath_counter << std::endl;
                //std::cout << "breath queue: " << temp_breath_cells << "/" << persistent_queue.size() << std::endl;
                //printVector("", persistent_queue);
                //std::cout << std::endl;
                cv::waitKey(0);
                
            }
            temp_cycle_counter++;

            // Copy the first queue element to visited
            visited.push_back(queue[0]);
            weigth.push_back(breath_counter);
            map.at<uchar>(queue[0].x,queue[0].y)=30;
            // Remove the first queue element
            queue.erase(queue.begin());
            
            cv::imshow("map", map);
            
            ++cycles;
        }
        //printVector("queue: ",queue);
        //printVector("visited: ",visited);        
        //printVector("weigth: ",weigth);        
        std::cout << cycles << std::endl;

        //std::cout << weight_map;

        //Reconstruct path
        bool path_ok=false;
        path.push_back(destination);
        int temp_weigth=weight_map.at<int>(destination.x,destination.y);
        while((cycles < 1000) && not(path_ok)){
            cycles++;
            for(int idx=0; idx<neighbours.size(); idx++){
                //for(offset.x=-1; offset.x<=1; offset.x++){
                //for(offset.y=-1; offset.y<=1; offset.y++){
                temp_point=path[path.size()-1]+neighbours[idx];
                if((weight_map.at<int>(temp_point.x,temp_point.y) < temp_weigth) && (weight_map.at<int>(temp_point.x,temp_point.y) > -1)){
                    path.push_back(temp_point);
                    temp_weigth=weight_map.at<int>(temp_point.x,temp_point.y);
                    map.at<uchar>(temp_point.x,temp_point.y)=200;
                    if(weight_map.at<int>(temp_point.x,temp_point.y) == 0){
                        path_ok = true;
                    }
                }
                //}
            }
        }
        printVector("path", path);
        cv::imshow("map", map);
        
    }
        
    cv::waitKey(0);
    return 0;
}