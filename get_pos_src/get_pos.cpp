
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

#define LOOP_LIM 1000
//0 is default, 1 is time, 2 is auto, 3 is auto + time
int argument;

int main(int argc, char *argv[]){
    if(argc == 1){ // default, no input arguments, 
        argument = 0;
    }else if(argc == 2){ // 1 argument
        if(isdigit(argv[1])){
            argument = 1; // time only
        }
        else {
            if((strcmp(argv[1], "Auto") == 0) || (strcmp(argv[1], "auto") == 0))
            argument = 2; // auto only
            else{
                std::cout << Error: Invalid argument;
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
    if((argument == 0) && (argument == 1)){ // If no auto is given or only argument is time
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
    rs2::frame frame;
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    pipe.start(cfg);
    std::cout << "Pipeline successfully started" << std::endl;
    gpioTerminate();
    //rs2::frame::get_timestamp
    float** pos_matrix = new float*[LOOP_LIM]; 
    //creates and initializes csv output file
    std::ofstream myFile("pos_result.csv");
    //double time_count = 0.0;
    //auto start_time = std::chrono::high_resolution_clock::now();
    myFile << "Time,x,y,z,\n";

    std::cout << "Beginning parsing..." << std::endl;
    for (int n = 0; n < LOOP_LIM; ++n) {
        //get position and time data 
        float* pos_matrix_item = new float[4];
        //clock_t start_time = clock();
        //get data from t265
        auto frames = pipe.wait_for_frames(); 
        auto f = frames.first_or_default(RS2_STREAM_POSE); 
        auto pose_data = f.as<rs2::pose_frame>().get_pose_data();

        //put values into a matrix
        //pos_matrix_item[0] = time_count;
        pos_matrix_item[0] = frame.get_timestamp();
        pos_matrix_item[1] = pose_data.translation.x;
        pos_matrix_item[2] = pose_data.translation.y;
        pos_matrix_item[3] = pose_data.translation.z;

        // Print the x, y, z values of the translation, relative to initial position -- DEBUG PURPOSES
        std::cout << "\r" << "Device Position: " << std::setprecision(4) << std::fixed << pose_data.translation.x << " " <<
        pose_data.translation.y << " " << pose_data.translation.z << " (meters)" << std::endl;



        //std::cout <<"Time count: " << time_count << std::endl;
        std::cout <<"Time count: " << frame.get_timestamp() << std::endl;
        //clock_t end_time = clock();
        //double elapsed_time = double(end_time - start_time) / CLOCKS_PER_SEC;
        //time_count += elapsed_time;
        //puts data into other array
        pos_matrix[n] = pos_matrix_item;
        
    }

    std::cout << "Done! Writing to files..." << std::endl;
    //stores all datapoints in new array
    for(int n = 0; n < LOOP_LIM; n++){ 
        myFile << std::setprecision(4) << pos_matrix[n][0] << "," << pos_matrix[n][1] << "," << pos_matrix[n][2] << "," << pos_matrix[n][3] << ",\n";
    }

    std::cout << "Program successfully finished." << std::endl; 
    //deallocate memory
    for(int n = 0; n < LOOP_LIM; n++){
        delete [] pos_matrix[n];
    }

    delete [] pos_matrix;

    return 0;
}
