#include <iostream>
#include <wiringPi.h>
#include <vector>
#include <chrono>
//#include <rs_pipeline.h>
#include "realsenselib/rs.hpp"
#include <system_error>
#include <iomanip>
#include <fstream>
#include <iostream>

#define LOOP_LIM 1000

int main() {
    wiringPiSetupGpio();
    pinMode (17,INPUT);
    std::cout << "T265 Pose - Matrix ver." << std::endl;
    std::cout << "Provide pulse to GPIO 17 to begin" << std::endl;
    int value = digitalRead(17);
    while(value != HIGH){
      value = digitalRead(17);
    }
    std::cout << "Beginning data collection..." << std::endl;
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    pipe.start(cfg);
    std::cout << "Pipeline successfully started" << std::endl;

    //std::vector<std::array<float, 3>> pos_matrix(2000);
    //std::vector<std::vector<int>> pos_matrix());
    //std::vector<float> time_datenum(1000);
    float** pos_matrix = new float*[LOOP_LIM];


    std::ofstream myFile("pos_result.csv");
    double time_count = 0.0;
    auto start_time = std::chrono::high_resolution_clock::now();
    myFile << "Time,x,y,z,\n";

    std::cout << "Beginning parsing..." << std::endl;
    for (int n = 0; n < LOOP_LIM; ++n) {
        
        float* pos_matrix_item = new float[4];

        //auto start_time = std::chrono::high_resolution_clock::now();
        clock_t start_time = clock();

        auto frames = pipe.wait_for_frames(); // Wait for next frame from camera
        auto f = frames.first_or_default(RS2_STREAM_POSE); // get frame from pose stream
        auto pose_data = f.as<rs2::pose_frame>().get_pose_data();

        //put values into a matrix
        pos_matrix_item[0] = time_count;
        pos_matrix_item[1] = pose_data.translation.x;
        pos_matrix_item[2] = pose_data.translation.y;
        pos_matrix_item[3] = pose_data.translation.z;

        // Print the x, y, z values of the translation, relative to initial position
        std::cout << "\r" << "Device Position: " << std::setprecision(4) << std::fixed << pose_data.translation.x << " " <<
        pose_data.translation.y << " " << pose_data.translation.z << " (meters)" << std::endl;



        std::cout <<"Time count: " << time_count << std::endl;
        clock_t end_time = clock();
        double elapsed_time = double(end_time - start_time) / CLOCKS_PER_SEC;
        time_count += elapsed_time;

        //std::cout << std::setprecision(3) << "Pos_matrix_item" << pose_data.translation.z << std::endl;

        pos_matrix[n] = pos_matrix_item;

        //delete [] pos_matrix_item;
        
    }

    std::cout << "Done! Writing to files..." << std::endl;

    for(int n = 0; n < LOOP_LIM; n++){ 
        myFile << std::setprecision(4) << pos_matrix[n][0] << "," << pos_matrix[n][1] << "," << pos_matrix[n][2] << "," << pos_matrix[n][3] << ",\n";
        //std::cout << std::setprecision(3) << "Pos_matrix_item" << pos_matrix[n][1]<< std::endl;
    }
    //End program
    std::cout << "Program successfully finished." << std::endl;

    //std::cout << pos_matrix[2][1]<< std::endl;
   
    
    
    //deallocate memory
    for(int n = 0; n < LOOP_LIM; n++){
        delete [] pos_matrix[n];
    }

    delete [] pos_matrix;

    return 0;
}
