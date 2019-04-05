#include <iostream>
#include <string>
#include <chrono>   // speed test
#include <fstream>

#include <cstdio>
#include <cmath>

#include "lib/object.hpp"

using namespace std;

// key: `road_id`;   value: `ROAD*`;
// **Attention:** you should delete `ROAD*` in `roadMap` manually
map_type<int, ROAD *> roadMap;

// key: `cross_id`;   value: `CROSS*`
map_type<int, CROSS *> crossMap;

// 按 cross id 升序排列的 `CROSS*`;
// **Attention:** you should delete `CROSS*` in `crossVec` manually
static vector<CROSS *> crossVec;


static int global_time = 0; //全局时间

int main(int argc, char *argv[])
{
    auto __start_time = std::chrono::steady_clock::now();

/* SDK code */
    cout << "Begin" << endl;
    
    if(argc < 6){
        cout << "please input args: carPath, roadPath, crossPath, answerPath" << std::endl;
        exit(1);
    }
    
    string carPath(argv[1]);
    string roadPath(argv[2]);
    string crossPath(argv[3]);
    string presetAnswerPath(argv[4]);
    string answerPath(argv[5]);

    cout << "carPath is " << carPath << endl;
    cout << "roadPath is " << roadPath << endl;
    cout << "crossPath is " << crossPath << endl;
    cout << "presetAnswerPath is " << presetAnswerPath << endl;
    cout << "answerPath is " << answerPath << endl;
/* END of SDK code */


/* Read configuration from file */
    FILE *fptr;
    char line_buffer[MAXIMUM_LENGTH_PER_LINE];

  /* Read road configuration, and build GRAPH at the same time */
    GRAPH graph(NODE_VECTOR_RESERVE_SIZE);

    int id, length, speed, channel, from, to, isDuplex;

    fptr = fopen(roadPath.c_str(), "r");
    if (fptr == NULL) {
        cout << "can't open road configuration file: " << roadPath << endl;
        exit(-1);
    }

    /* 逐行读取道路信息 */
    fgets(line_buffer, 50, fptr); // 似乎只有文件第一行为注释......
    while (fgets(line_buffer, 50, fptr)) {
        //if(line_buffer[0] == '(') {
            sscanf(line_buffer,"(%d, %d, %d, %d, %d, %d, %d)", 
                   &id, &length, &speed, &channel, &from, &to, &isDuplex);
            auto pRoad = new ROAD(id, length, speed, channel, from, to, isDuplex);
            graph.add_node(pRoad);
            roadMap.insert(pair<int, ROAD*>(id, pRoad));
        //}
    }
    fclose(fptr);
  /* End of Read road configuration */


  /* Read cross configuration */
    fptr = fopen(crossPath.c_str(), "r");
    if (fptr == NULL) {
        cout << "can't open cross configuration file: " << crossPath << endl;
        exit(-1);
    }

    int roadId[4];
    /* 逐行读取cross信息 */
    fgets(line_buffer, 50, fptr); // 似乎只有文件第一行为注释......
    while (fgets(line_buffer, 50, fptr)) {
        //if(line_buffer[0] == '(') {
            sscanf(line_buffer,"(%d, %d, %d, %d, %d)", 
                   &id, roadId, roadId+1, roadId+2, roadId+3);
            auto pCross = new CROSS(id, roadId);
            crossMap.insert(pair<int, CROSS*>(id, pCross));
            crossVec.push_back(pCross);
        //}
    }
    fclose(fptr);

    // 将CROSS按id升序排列
    sort(crossVec.begin(), crossVec.end(), [](CROSS *a, CROSS *b)->bool{return a->id < b->id;});
  /* End of Read cross configuration */


  /* Read car configuration */
    bool car_speed_detect_array[MAXIMUM_ROAD_LENGTH] = {false};    //用于标记车速

    int planTime, isPrior, isPreset;

    fptr = fopen(carPath.c_str(), "r");
    if (fptr == NULL) {
        cout << "can't open car configuration file: " << carPath << endl;
        exit(-1);
    }

    /* 逐行读取车辆信息 */
    fgets(line_buffer, 50, fptr); // 似乎只有文件第一行为注释......
    while (fgets(line_buffer, 50, fptr)) {
        //if(line_buffer[0] == '(') {
            sscanf(line_buffer, "(%d, %d, %d, %d, %d, %d, %d)",
                   &id, &from, &to, &speed, &planTime, &isPrior, &isPreset);

            crossMap[from]->garage.push_back(new CAR(id, from, to, speed, planTime, isPrior, isPreset));
            car_speed_detect_array[speed] = true; //标记车速
        //}
    }
    fclose(fptr);

    graph.add_weights(car_speed_detect_array, MAXIMUM_ROAD_LENGTH);
  /* End of read car configuration */

/* END of read configuration from file */


/* 打开输出文件*/
    ofstream fout;
    char out_file_buffer[OUT_FILE_RESERVE_SPACE*1024];
    fout.rdbuf()->pubsetbuf(out_file_buffer, OUT_FILE_RESERVE_SPACE*1024);
    fout.open(answerPath);
    if (!fout.is_open()){
        cerr << "无法打开文件 " << answerPath << endl;
        exit(0);
    }
/*End of 打开输出文件*/

  

    fout.close();

/* free memory */
    /* delete ROAD* */
    for(auto val:roadMap){delete val.second;}
/* End of free memory */

    auto __end_time = std::chrono::steady_clock::now();
    std::chrono::duration<double, std::milli> elapsed = __end_time - __start_time; // std::micro 表示以微秒为时间单位
    std::cout << "time: " << elapsed.count() << "ms." << endl;

    return 0;
}
