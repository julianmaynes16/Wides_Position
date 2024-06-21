
#include <pigpio.h>
#include <vector>
#include <chrono>
//#include <rs_pipeline.h>
#include "realsenselib/rs.hpp"
#include <system_error>
#include <iomanip>
#include <fstream>
#include <iostream>
#include <cstring>
#include <ctype.h>
//Shouldn't be passed ever -- dependent on how long we want loop to run for  
#define LOOP_LIM 1000000
//0 is default, 1 is time, 2 is auto, 3 is auto + time
int argument;
//default time limit in seconds
int time_limit = 10;
//array iterator
 int n = 0;

int main(int argc, char *argv[]){
    if(argc == 1){ // default, no input arguments, 
        argument = 0;
    }else if(argc == 2){ // 1 argument
        if(isdigit(*(argv[1]))){
            argument = 1; // time only
        }
        else {
            if((strcmp(argv[1], "Auto") == 0) || (strcmp(argv[1], "auto") == 0))
            argument = 2; // auto only
            else{
                std::cout << "Error: Invalid argument" << std::endl;
            }
        }
    }else if(argc == 3){ // 2 arguments
        argument = 3;
    }else{
        std::cout << "Error: Too many arguments." << std::endl;
        return 0;
    }
    gpioInitialise();
    //establishes pin 17 as input
    gpioSetMode(17, PI_INPUT);
    std::cout << "T265 Pose - Matrix ver." << std::endl;

    if(argument == 1){ // only time given
        time_limit = atoi(argv[1]);
    }
    else if(argument == 3){ // auto and time
        time_limit = atoi(argv[2]);
    }else

    if((argument == 0) || (argument == 1)){ // If no auto is given or only argument is time
        std::cout << "Provide pulse to GPIO 17 to begin" << std::endl;
        int value = gpioRead(17);
        //read returns 1 if pin is HIGH
        while(value != 1){
            value = gpioRead(17);
        }
    }
    std::cout << "Beginning data collection..." << std::endl;
    rs2::pipeline pipe;// t265 pipeline declaration
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    pipe.start(cfg);
    std::cout << "Pipeline successfully started" << std::endl;
    gpioTerminate();
    float** pos_matrix = new float*[LOOP_LIM]; 
    //creates and initializes csv output file
    std::ofstream myFile("pos_result.csv");
    double time_count = 0.0;
    myFile << "Time,x,y,z,\n";

    std::cout << "Beginning parsing..." << std::endl;
    while(time_count < time_limit){
        //get position and time data 
        float* pos_matrix_item = new float[4];
        auto start_time = std::chrono::high_resolution_clock::now();
        //get data from t265
        auto frames = pipe.wait_for_frames(); 
        auto f = frames.first_or_default(RS2_STREAM_POSE); 
        auto pose_data = f.as<rs2::pose_frame>().get_pose_data();
        //put values into a matrix
        pos_matrix_item[0] = time_count;
        pos_matrix_item[1] = pose_data.translation.x;
        pos_matrix_item[2] = pose_data.translation.y;
        pos_matrix_item[3] = pose_data.translation.z;

        // Print the x, y, z values of the translation, relative to initial position -- DEBUG PURPOSES
        std::cout << "\r" << "Device Position: " << std::setprecision(4) << std::fixed << pose_data.translation.x << " " <<
        pose_data.translation.y << " " << pose_data.translation.z << " (meters)" << std::endl;

        std::cout <<"Time count: " << time_count << std::endl;
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_time = end_time - start_time;
        double elapsed_seconds = elapsed_time.count();
        time_count += elapsed_seconds;
        //puts data into other array
        pos_matrix[n] = pos_matrix_item;
        n++;
    }

    std::cout << "Done! Writing to files..." << std::endl;
    //stores all datapoints in new array
    for(int i = 0; i < n; i++){ 
        myFile << std::setprecision(4) << pos_matrix[i][0] << "," << pos_matrix[i][1] << "," << pos_matrix[i][2] << "," << pos_matrix[i][3] << ",\n";
    }

    std::cout << "Program successfully finished." << std::endl; 
    //deallocate memory
    for(int n = 0; n < LOOP_LIM; n++){
        delete [] pos_matrix[n];
    }

    delete [] pos_matrix;

    return 0;
}
