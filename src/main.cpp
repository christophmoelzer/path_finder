#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include "opencv2/opencv.hpp"

struct weighted_point{
    int x;
    int y;
    int dist;
};

struct point_n_path{
    cv::Point point;
    std::vector<cv::Point> path;
};

// Check if Node is free
bool checkFree(cv::Point point, cv::Mat map){
    if(map.at<uchar>(point.x,point.y) > 0){
        std::cout << "free " << point << std::endl;
        return true;
    }
    else{
        std::cout << "not free " << point << std::endl;
        return false;
    }
}

bool checkCorner(cv::Point point, cv::Mat map){
    // Set offset neighbour cells
    std::vector<cv::Point> neighbours;
    cv::Point temp_point;
    int wall_counter=0;

    neighbours.push_back({0,-1});
    neighbours.push_back({-1,0});
    neighbours.push_back({1,0});
    neighbours.push_back({0,1});

    neighbours.push_back({-1,-1});
    neighbours.push_back({-1,1});
    neighbours.push_back({1,-1});
    neighbours.push_back({1,1});


    for(int idx=0; idx<neighbours.size(); idx++){
        temp_point=point+neighbours[idx];
        if (map.at<uchar>(temp_point.x,temp_point.y) == 0) {
            
            wall_counter++;
        }
    }

    if (wall_counter >= 5){
        return true;
    }
    else{
        return false;
    }

}

// Check if Node is Destination
bool checkDestination(cv::Point point, cv::Point dest){
    if(point == dest){
        ROS_INFO_STREAM("Point is destination");
        return true;
    }
    else{
        
        return false;
    }
}

// Calculate Distance
int calcDistance(cv::Point point, cv::Point dest){
    int dist = (abs(point.x-dest.x)+abs(point.y-dest.y));

    return dist;
}

// Check if Node is valid
bool checkValid(cv::Point point, cv::Point origin, cv::Mat map, bool free_flag){
    if (free_flag==true){
        if (point.x >= 0 && point.x < map.cols && point.y >= 0 && point.y < map.rows && point != origin){
            return true;
        }
        else{
            return false;
        }
    }
    else{
        if (point.x >= 0 && point.x < map.cols && point.y >= 0 && point.y < map.rows && point!=origin && map.at<uchar>(point.x,point.y) != 0 ){
            return true;
        }
        else{
            return false;
        }
    }
    

}

bool checkLastPoints(cv::Point point, std::vector<cv::Point> last_points){
    if (last_points.size() > 0){
        for(int i=0; i<last_points.size(); i++){
            if(point.x == last_points[i].x && point.y == last_points[i].y){
                return false;
            }
        }
    }
    return true;
}

void printVector(std::string text,std::vector<cv::Point> vec){
    for(int i=0; i<vec.size(); i++){
        ROS_INFO_STREAM(i << "->" << vec[i] << "; ");
    }
}

void printVector(std::string text,std::vector<int> vec){
    for(int i=0; i<vec.size(); i++){
        ROS_INFO_STREAM(text << " " << i << " --> weight = " << vec[i]);
    }
}

cv::Mat readMap(std::string path_to_map){
    
    // Import the map and convert it to to gray
    cv::Mat map = cv::imread(path_to_map);
    cv::cvtColor(map, map, CV_BGR2GRAY);
    map.convertTo(map, CV_8U);

    // Show size of map
    ROS_INFO_STREAM("SIZE = " << map.rows << " / " << map.cols );

    return map;
}

point_n_path searchLoop(cv::Mat map, cv::Point start, std::vector<cv::Point> corners, int max_cycles, int methode){
    /*
    methode:
    1 - corner
    2 - next obstacle / wall
    3 - dest point
    4 - 
    */

    // Set offset neighbour cells
    std::vector<cv::Point> neighbours;
    switch (methode)
    {
    case 1:
        neighbours.push_back({0,-1});
        neighbours.push_back({-1,0});
        neighbours.push_back({1,0});
        neighbours.push_back({0,1});

        break;
    case 2:
        neighbours.push_back({0,-1});

        break;
    case 3:
        neighbours.push_back({-1,0});

        break;
    case 4:
        neighbours.push_back({0,1});

        break;
    case 5:
        neighbours.push_back({1,0});

        break;
    case 6:
        neighbours.push_back({0,-1});
        neighbours.push_back({-1,0});
        neighbours.push_back({1,0});
        neighbours.push_back({0,1});

        break;
    
    default:
        neighbours.push_back({0,-1});
        neighbours.push_back({-1,0});
        neighbours.push_back({1,0});
        neighbours.push_back({0,1});
        break;
    }
    
    int cycles = 0;
    cv::Point destination(23,23);
    // Create weighted map for path planning
    cv::Mat weight_map = cv::Mat::zeros(map.rows,map.cols,CV_32S);
    for(int x=0; x<weight_map.rows; x++){
        for(int y=0; y<weight_map.cols; y++){
            weight_map.at<int>(x,y)=-1;
        }        
    }

    weight_map.at<int>(start.x,start.y)=0;          // Set the start point on the weighted map (value "0" equals the start point)
    map.at<uchar>(start.x,start.y)=50;              // Mark start and end point on the map
    map.at<uchar>(destination.x,destination.y)=50;
    
    
    std::vector<cv::Point> visited;               // Vector for visited cells
    std::vector<cv::Point> path;                  // Vector for path
    std::vector<cv::Point> temp_queue;            // Vector for temporary queue
    std::vector<cv::Point> persistent_queue;      // Vector for persistent queue
    std::vector<int> weight;                      // Vector for weights (assigned to visited vector) 

    bool free_flag = false;
    bool wall_flag = false;

    bool is_destination = false;
    int temp_breath_cells=0;
    int breath_counter=0;
    int temp_cycle_counter=0;
    int old_queue_size=0;

    
    visited.push_back(start);   // The search starts at the start point
    weight.push_back(0);        // The start point is weighted with the value "0"

    cv::Point offset;
    cv::Point temp_point;

    ROS_INFO_STREAM("Press any key to start");
    //cv::waitKey(0);


    // Search the destination point
    while((cycles < max_cycles) && not(is_destination)){
        // Search valid neighbour cells around the visited cell
                    
        // Check neighbour cells
        for(int idx=0; idx<neighbours.size(); idx++){
            if (methode==2) std::cout << "-----------------" << std::endl<< std::endl;
            temp_point=visited[visited.size()-1]+neighbours[idx];
            
            if(checkLastPoints(temp_point,visited) && checkLastPoints(temp_point,corners)){                    // The point must NOT be visited
                if (methode==2) std::cout << "not visited " << temp_point << std::endl;
                if(checkLastPoints(temp_point,temp_queue)){             // The point must NOT be in the temp_queue
                    if (methode==2) std::cout << "not in queue " << temp_point << std::endl;
                    if(checkValid(temp_point, visited[visited.size()-1], map, free_flag)){     // The point has to be inside the map
                        if (methode==2) std::cout << "valid " << temp_point << std::endl;
                        switch (methode)
                        {
                        case 1: // Corner
                            if(checkCorner(temp_point,map)){
                                is_destination=true;
                                destination = temp_point;
                                map.at<uchar>(destination.x,destination.y)=50;
                                ROS_INFO_STREAM("Corner found");
                            }

                            temp_queue.push_back(temp_point);                                   // Append the point to the temp_queue
                            persistent_queue.push_back(temp_point);
                            //map.at<uchar>(temp_point.x,temp_point.y)=100;                       // Mark the cell as next search cell
                            weight_map.at<int>(temp_point.x,temp_point.y)=breath_counter+1;     // Set the weight for the current breath
                            break;
                        case 2: case 3: case 4: case 5: // Next wall up or down
                            std::cout << "methode = " << methode << std::endl;
                            if(checkFree(temp_point,map)){                      // The point must be free
                            
                                temp_queue.push_back(temp_point);                                   // Append the point to the temp_queue
                                persistent_queue.push_back(temp_point);
                                //map.at<uchar>(temp_point.x,temp_point.y)=100;                       // Mark the cell as next search cell
                                weight_map.at<int>(temp_point.x,temp_point.y)=breath_counter+1;     // Set the weight for the current breath
                                
                                std::cout << "free " << temp_point << std::endl;
                            }
                            else if(!checkFree(temp_point,map) && free_flag){
                                std::cout << "wall found " << temp_point << std::endl;
                                wall_flag=true;
                                free_flag=false;
                                is_destination=true;
                                destination = temp_point;
                            }
                            else{
                                std::cout << "not free" << std::endl;
                                
                            }

                            break;
                        
                        case 6: // Point
                            if(checkFree(temp_point,map)){                      // The point must be free
                            
                                temp_queue.push_back(temp_point);                                   // Append the point to the temp_queue
                                persistent_queue.push_back(temp_point);
                                //map.at<uchar>(temp_point.x,temp_point.y)=100;                       // Mark the cell as next search cell
                                weight_map.at<int>(temp_point.x,temp_point.y)=breath_counter+1;     // Set the weight for the current breath
                                
                                if(checkDestination(temp_point,destination)){                       // Check for destionation point
                                    is_destination=true;
                                }
                                
                            }
                            break;
                        default:
                            break;
                        }
                    }
                    else{
                        if (methode==2){
                            if (!wall_flag) free_flag = true;
                            std::cout << "not valid " << temp_point << std::endl;
                        } 

                    }
                }
                else{
                    if (methode==2) std::cout << " in queue" << temp_point << std::endl;
                }
            }
            else{
                if (methode==2) std::cout << " visited " << temp_point << std::endl;
            }
        }

        // Increase the breath counter
        if(temp_cycle_counter>=temp_breath_cells){
            temp_breath_cells = persistent_queue.size()-old_queue_size;
            old_queue_size = persistent_queue.size();
            temp_cycle_counter=0;
            breath_counter++;
            
            //cv::waitKey(30);
        }
        temp_cycle_counter++;


        visited.push_back(temp_queue[0]);                   // Copy the first temp_queue element to visited
        //map.at<uchar>(temp_queue[0].x,temp_queue[0].y)=30;  // Mark the cell as visited (value = 30)
        weight.push_back(breath_counter);                   // Set the weight for the current cell
        temp_queue.erase(temp_queue.begin());               // Remove the first temp_queue element
        ++cycles;

       // cv::imshow("map", map);                             // Show map for visualization of the search algorithm
    }
    
    // Search has finished       
    ROS_INFO_STREAM("Iterations: " << cycles);
    ROS_INFO_STREAM("Breaths: " << breath_counter);

    // Reconstruct path
    bool path_ok=false;
    path.push_back(destination);
    map.at<uchar>(destination.x,destination.y)=200;
    int temp_weight=weight_map.at<int>(destination.x,destination.y);

    while((cycles < max_cycles) && not(path_ok)){
        cycles++;
        for(int idx=0; idx<neighbours.size(); idx++){
            temp_point=path[path.size()-1]+neighbours[idx];

            // Search neighbour with the lower weight
            if((weight_map.at<int>(temp_point.x,temp_point.y) < temp_weight) && (weight_map.at<int>(temp_point.x,temp_point.y) > -1)){

                path.push_back(temp_point);                                 // Attach the current point to the path
                temp_weight=weight_map.at<int>(temp_point.x,temp_point.y);  // Set new weight for the path search
                map.at<uchar>(temp_point.x,temp_point.y)=100;               // Mark point as path point (value = 200)

                if(weight_map.at<int>(temp_point.x,temp_point.y) == 0){     // Check if point is start point
                    path_ok = true;
                }
            }
        }
        //cv::waitKey(1);
        //cv::imshow("map", map);         // Visualize path reconstruction
    }

    point_n_path ret_val;

    ret_val.path = path;
    ret_val.point = destination;

    return ret_val;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros:: Rate loop_rate(1);

    bool run_once = false;
    int loop_counter = 0;

    cv::Point start(26,26);
    cv::Point destination(27,20);
    std::vector<cv::Point> path;
    std::vector<cv::Point> corners;
    point_n_path ret_val;
    //cv::namedWindow("map",0);

    
    
    while(ros::ok() && run_once==false){
        run_once=true;
        loop_counter++;

        ros::spinOnce();
        loop_rate.sleep();
        
        cv::Mat map = readMap("/home/chris/Desktop/map.png");   // Import the map and convert it to to gray
        if(map.rows <= 0){                                          // Check if map is valid
            ROS_INFO_STREAM("Loading of map failed!");
            return -1;
        }

        

        ROS_INFO_STREAM("start");
        ret_val=searchLoop(map, start, corners, 800, 1);    // search path to destination
        cv::imwrite("/home/chris/Desktop/a.pgm",map);
        ROS_INFO_STREAM("stop");
        corners.push_back(ret_val.point);
        start = ret_val.point;
        ret_val=searchLoop(map, start, corners, 800, 2);    // search path to destination
        cv::imwrite("/home/chris/Desktop/b.pgm",map);
        
        corners.push_back(ret_val.point);
        start = ret_val.point;
        ret_val=searchLoop(map, start, corners, 800, 3);    // search path to destination
        cv::imwrite("/home/chris/Desktop/c.pgm",map);

        std::string path_desktop = "/home/chris/Desktop/";
        std::string extension = ".pgm";
        std::stringstream ss;
        ss << loop_counter;
        std::string filename = ss.str();
        path_desktop.append(filename);
        path_desktop.append(extension);
        cv::imwrite(path_desktop,map);
        //cv::waitKey(10);
    }
    
    ROS_INFO_STREAM("Press any key to exit");
    //cv::waitKey(10);
    
    return 0;
}