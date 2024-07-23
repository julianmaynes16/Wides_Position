
#include <pigpio.h>
#include <vector>
#include <chrono>
// #include <rs_pipeline.h>
#include "realsenselib/rs.hpp"
#include <system_error>
#include <iomanip>
#include <fstream>
#include <iostream>
#include <cstring>
#include <ctype.h>
#include <string>
// Shouldn't be passed ever -- dependent on how long we want loop to run for
#define LOOP_LIM 1000000
// 0 is default, 1 is time, 2 is auto, 3 is auto + time
int argument;
// default time limit in seconds
int time_limit = 10;
// array iterator
int n = 0;
//light delay
int warn_delay = 0;
//default to require pulse
bool auto_flag = false;
//defualt max samplerate
double sample_choke = 0;

int main(int argc, char *argv[])
{
    if(argc >= 6){
        std::cout << "Error: Too many arguments." << std::endl;
        return 0;
    }
    //loop through arguments, first 2 arguments, sudo and command, are skipped
    for(int i = 2; i < argc; i++){
        if(std::string(argv[i]).find("mode=") != std::string::npos){
            //mode= was found, next thing will be auto

            if(std::string(argv[i]).find("auto") != std::string::npos){
                std::cout <<"Hi" << std::endl;
                auto_flag = true;
            }
        }
        else if(std::string(argv[i]).find("time=") != std::string::npos){
            time_limit = stoi(std::string(argv[i]).substr(5));
        }
        else if(std::string(argv[i]).find("sample_rate=") != std::string::npos){
            sample_choke = stoi(std::string(argv[i]).substr(12));
        }
    }
    
    gpioInitialise();
    // establishes pin 17 as input
    gpioSetMode(17, PI_INPUT);
    gpioSetMode(27, PI_OUTPUT);
    std::cout << "T265 Pose - Matrix ver." << std::endl;

    std::cout << "Starting pipeline..." << std::endl;
    rs2::pipeline pipe; // t265 pipeline declaration
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    pipe.start(cfg);
    std::cout << "Pipeline successfully started" << std::endl;

    float **pos_matrix = new float *[LOOP_LIM];
    // creates and initializes csv output file
    std::ofstream myFile("pos_result.csv");
    myFile << "Time,X,Y,Z,C\n";
    

    if(!auto_flag)
    { // If no auto is given or only argument is time
        std::cout << "Provide pulse to GPIO 17 to begin" << std::endl;
        int value = gpioRead(17);
        // read returns 1 if pin is HIGH
        while (value != 1)
        {
            value = gpioRead(17);
        }
    }
    //beginning times
    auto choke_begin = std::chrono::high_resolution_clock::now();
    float choke_time_since_start = 0;
    auto sample_begin = std::chrono::high_resolution_clock::now();
    std::cout << "Beginning parsing..." << std::endl;
    while (choke_time_since_start < time_limit)
    {
        //current time
        auto sample_curr = std::chrono::high_resolution_clock::now();
        //current time for choke, resets to match sample rate
        choke_time_since_start = std::chrono::duration<float>(sample_curr - choke_begin).count();
        //current time for timestamp, never stops / resets
        float curr_sample_time = std::chrono::duration<float>(sample_curr - sample_begin).count();

        if(choke_time_since_start > sample_choke){
            // get position and time data
            float *pos_matrix_item = new float[5];
            // get data from t265
            auto frames = pipe.wait_for_frames();
            auto f = frames.first_or_default(RS2_STREAM_POSE);
            auto pose_data = f.as<rs2::pose_frame>().get_pose_data();
            // put values into a matrix
            pos_matrix_item[0] = curr_sample_time;
            pos_matrix_item[1] = pose_data.translation.x;
            pos_matrix_item[2] = pose_data.translation.y;
            pos_matrix_item[3] = pose_data.translation.z;
            pos_matrix_item[4] = pose_data.tracker_confidence;

            // Print the x, y, z values of the translation, relative to initial position -- DEBUG PURPOSES
            std::cout << "\r" << "Device Position: " << std::setprecision(4) << std::fixed << pose_data.translation.x << " " << pose_data.translation.y << " " << pose_data.translation.z << " (meters)" << std::endl;
            std::cout << "Time count: " << curr_sample_time << std::endl;
            std::cout << "Confidence: " << pose_data.tracker_confidence << std::endl;
            if (pose_data.tracker_confidence < 2)
            {
                std::cout << "WARNING: Data is unreliable. Move to a brighter area and/or move away from the wall." << std::endl;
                gpioWrite(27,1);
                warn_delay = curr_sample_time + 1;
            }
            pos_matrix[n] = pos_matrix_item;
            if (curr_sample_time > warn_delay){
                gpioWrite(27,0);
            }
            n++;
            choke_begin = std::chrono::high_resolution_clock::now();
        }
    }
    gpioTerminate();
    std::cout << "Done! Writing to files..." << std::endl;
    // stores all datapoints in new array
    for (int i = 0; i < n; i++)
    {
        myFile << std::setprecision(4) << pos_matrix[i][0] << "," << pos_matrix[i][1] << "," << pos_matrix[i][2] << "," << pos_matrix[i][3] << "," << pos_matrix[i][4] << ",\n";
    }

    std::cout << "Program successfully finished." << std::endl;
    // deallocate memory
    for (int n = 0; n < LOOP_LIM; n++)
    {
        delete[] pos_matrix[n];
    }

    delete[] pos_matrix;

    return 0;
}
