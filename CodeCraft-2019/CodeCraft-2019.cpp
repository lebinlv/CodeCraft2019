
#include <iostream>
#include <cstdio>
#include <vector>
#include <queue>
#include <algorithm>
#include <limits>   // std::numeric_limits
#include <string>
#include <chrono>   // speed test

#include "object.hpp"

#define MAXIMUM_LENGTH_PER_LINE    50      // 每行字符个数不超过 MAXIMUM_LENGTH_PER_LINE
#define CAR_VECTOR_RESERVE_SIZE    11000   // 
#define ROAD_VECTOR_RESERVE_SIZE   150 
#define USE_CXX_SSTREAM 0

using namespace std;


int main(int argc, char *argv[])
{
/* SDK code */
    cout << "Begin" << endl;

	if(argc < 5){
        cout << "please input args: carPath, roadPath, crossPath, answerPath" << endl;
        exit(1);
    }

    string carPath(argv[1]);
    string roadPath(argv[2]);
    string crossPath(argv[3]);
    string answerPath(argv[4]);
    cout << "carPath is " << carPath << endl;
    cout << "roadPath is " << roadPath << endl;
    cout << "crossPath is " << crossPath << endl;
    cout << "answerPath is " << answerPath << endl;
/* END of SDK code */


/* Read configuration from file，考虑到文件体积较小，使用 fgets 函数（大文件可使用 sstream + getline）*/
    FILE *fptr;
    char line_buffer[MAXIMUM_LENGTH_PER_LINE];

  /* Read car configuration */
    //priority_queue<CAR *, vector<CAR *>, CAR::Compare> car_pri_queue; // 建立 priority queue 将所有车辆信息保存
    vector<CAR*> car_vec;
    car_vec.reserve(CAR_VECTOR_RESERVE_SIZE);

    fptr = fopen(carPath.c_str(), "r");
    if (fptr == NULL) {
        cout << "can't open car configuration file: " << carPath << endl;
        exit(-1);
    }

    //auto start = std::chrono::steady_clock::now();
    /* 逐行读取车辆信息 */
    fgets(line_buffer, 50, fptr); // 似乎只有文件第一行为注释......
    while (fgets(line_buffer, 50, fptr)) {
        //if(line_buffer[0] == '(') {
            auto pCar = new CAR;
            sscanf(line_buffer, "(%d, %d, %d, %d, %d)",
                   &(pCar->id), &(pCar->from), &(pCar->to), &(pCar->speed), &(pCar->plan_time));
            //car_pri_queue.push(pCar);
            car_vec.push_back(pCar);
        //}
    }
    fclose(fptr);

    /* 如果两辆车的计划时间相等，则先速度快的车优先级高；否则先出发的车优先级高。 */
    sort(car_vec.begin(), car_vec.end(),
         [](CAR* a, CAR* b) -> bool { return 
         (a->plan_time ==  b->plan_time) ? (a->speed > b->speed) : (a->plan_time < b->plan_time);});
    // auto end = std::chrono::steady_clock::now();
    // std::chrono::duration<double, std::micro> elapsed = end - start; // std::micro 表示以微秒为时间单位
    // std::cout << "time: " << elapsed.count() << "us" << std::endl;

  /* End of read car configuration */



  /* Read cross configuration, mainly to get the number of cross */
    fptr = fopen(crossPath.c_str(), "r");
    if (fptr == NULL) {
        cout << "can't open cross configuration file: " << roadPath << endl;
        exit(-1);
    }
    int cross_count = 0;
    /* 逐行读取信息 */
    fgets(line_buffer, 50, fptr); // 似乎只有文件第一行为注释......
    while (fgets(line_buffer, 50, fptr)) {
        //if(line_buffer[0] == '(') {
            cross_count ++;
        //}
    }
    fclose(fptr);
  /* End of Read cross configuration */



  /* Read road configuration, and build GRAPH at the same time */
    vector<ROAD*> road_vec;
    road_vec.reserve(ROAD_VECTOR_RESERVE_SIZE);

    fptr = fopen(roadPath.c_str(), "r");
    if (fptr == NULL) {
        cout << "can't open road configuration file: " << roadPath << endl;
        exit(-1);
    }

    CROSS::id_type from, to;
    bool isDuplex;

	//(5000, 15, 6, 2, 1, 2, 1)
    /* 逐行读取道路信息 */
    fgets(line_buffer, 50, fptr); // 似乎只有文件第一行为注释......
    while (fgets(line_buffer, 50, fptr)) {
        //if(line_buffer[0] == '(') {
            auto pRoad = new ROAD;
            sscanf(line_buffer, "(%d, %d, %d, %d, %d, %d, %d)",
                &(pRoad->id), &(pRoad->length), &(pRoad->max_speed), &(pRoad->channel), 
                &from, &to, &isDuplex);
            road_vec.push_back(pRoad);
        //}
    }
    fclose(fptr);

  /*End of read road configuration */

/* END of read configuration from file */


    /* TODO:process */

    /* End of process */

    /* TODO:write output file */

    /* End of write out file */

    return 0;
}
