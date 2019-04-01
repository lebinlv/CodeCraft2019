#include <iostream>
#include <cstdio>
#include <string>
#include <cmath>
#include <chrono>   // speed test
#include "lib/function.hpp"

// 所有文件中每行字符个数不超过 MAXIMUM_LENGTH_PER_LINE
#define MAXIMUM_LENGTH_PER_LINE    50    // char line_buffer[MAXIMUM_LENGTH_PER_LINE]; fgets(line_buffer, 50, fptr);

// 为保存车辆信息的vector预分配的空间大小
#define CAR_VECTOR_RESERVE_SIZE    62000 // vector<CAR*> car_vec; car_vec.reserve(CAR_VECTOR_RESERVE_SIZE);

// 为图中的vector预分配空间，假设图包含n*n个路口，建议设置为 4n(n-1)
#define NODE_VECTOR_RESERVE_SIZE   360   // GRAPH graph(NODE_VECTOR_RESERVE_SIZE);

// 官方保证车辆速度小于道路长度length，因此创建一个长度为length的bool数组检测有多少种车速
#define MAXIMUM_ROAD_LENGTH        50      // 道路的最大长度

// 最大计划出发时间应小于 PLAN_TIME_DETECT_ARRAY_SIZE
#define PLAN_TIME_DETECT_ARRAY_SIZE  50      //

// 为减少磁盘IO操作次数，优化速度，为输出文件answer.txt预分配空间，单位：KB
#define OUT_FILE_RESERVE_SPACE 1024   // char out_file_buffer[OUT_FILE_RESERVE_SPACE*1024]; fout.rdbuf()->pubsetbuf(out_file_buffer, OUT_FILE_RESERVE_SPACE*1024);

using namespace std;
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


/* Read configuration from file，考虑到文件体积较小，使用 fgets 函数（大文件可使用 sstream + getline）*/
    FILE *fptr;
    char line_buffer[MAXIMUM_LENGTH_PER_LINE];

  /* Read car configuration */
    vector<CAR*> car_vec;             // attention: you should delete CAR* manually!!!
    car_vec.reserve(CAR_VECTOR_RESERVE_SIZE);

    bool car_speed_detect_array[MAXIMUM_ROAD_LENGTH] = {false};    //用于标记车速
    int plan_time_record[PLAN_TIME_DETECT_ARRAY_SIZE] = {0};              //用于统计各出发时刻的车辆的数目

    int id, from, to, speed, plan_time;

    fptr = fopen(carPath.c_str(), "r");
    if (fptr == NULL) {
        cout << "can't open car configuration file: " << carPath << endl;
        exit(-1);
    }

    /* 逐行读取车辆信息 */
    fgets(line_buffer, 50, fptr); // 似乎只有文件第一行为注释......
    while (fgets(line_buffer, 50, fptr)) {
        //if(line_buffer[0] == '(') {
            sscanf(line_buffer, "(%d, %d, %d, %d, %d)",&id, &from, &to, &speed, &plan_time);
            car_vec.push_back(new CAR(id, from, to, speed, plan_time));

            car_speed_detect_array[speed] = true; //标记车速
            plan_time_record[plan_time] ++;
        //}
    }
    fclose(fptr);

    /* 如果两辆车的计划时间相等，则先速度快的车优先级高；否则先出发的车优先级高。 */
    stable_sort(car_vec.begin(), car_vec.end(),
         [](CAR* a, CAR* b) -> bool { return
         (a->plan_time ==  b->plan_time) ? (a->speed < b->speed) : (a->plan_time < b->plan_time);});
  /* End of read car configuration */


  /* Read road configuration, and build GRAPH at the same time */
    map<int, ROAD*> road_map;       // attention: you should delete ROAD* manually!!!

    GRAPH graph(NODE_VECTOR_RESERVE_SIZE);

    int length, channel, isDuplex;

    fptr = fopen(roadPath.c_str(), "r");
    if (fptr == NULL) {
        cout << "can't open road configuration file: " << roadPath << endl;
        exit(-1);
    }

    /* 逐行读取道路信息 */
    fgets(line_buffer, 50, fptr); // 似乎只有文件第一行为注释......
    while (fgets(line_buffer, 50, fptr)) {
        //if(line_buffer[0] == '(') {
            sscanf(line_buffer,"(%d, %d, %d, %d, %d, %d, %d)", &id, &length, &speed, &channel, &from, &to, &isDuplex);
            auto pRoad = new ROAD(id, length, speed, channel, from, to, isDuplex);
            graph.add_node(pRoad);
            road_map.insert(pair<int, ROAD*>(id, pRoad));
        //}
    }
    fclose(fptr);
    graph.add_weights(car_speed_detect_array, MAXIMUM_ROAD_LENGTH);

  /*End of read road configuration */

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


/* 调度车辆 */
    list<CAR*> cars_running;//已经在运行的车辆
    list<CAR*> cars_waiting;//等待的车辆

    //取前面一百个
    int element_idx = 0;
    
    float factor=CAPACITY_FACTOR;


  /* WARM UP 阶段 */
    int max_plan_time = 0, plan_time_count = 0;
    for(int i=0; i<PLAN_TIME_DETECT_ARRAY_SIZE; i++){
        if(plan_time_record[i]>0) {plan_time_count++;max_plan_time=i;}
    }
    int BATCH_SIZE = ((car_vec.size()/plan_time_count)/100)*100;
    WARM_UPer warm_uper(plan_time_record, max_plan_time + 1);  // 创建调度器
    int idx=0;
    
    for (; global_time < max_plan_time;){
        global_time++;
        // 调度已出发的车辆
        release_capacity(cars_running, global_time);

        // WARM_UP 调度
        auto car_idx = warm_uper.get_car_in_warm_up(BATCH_SIZE, global_time);
    
        for (auto &val : *car_idx){
            for(int i=val.first; i<val.second; i++) {
                auto car = car_vec[i-idx];
                get_factor(car,global_time);
                auto path = graph.get_least_cost_route(car, global_time);
                if(path->empty()) {
                    continue;
                } else {
                    write_to_file(path, car, fout);
                    car_vec.erase(car_vec.begin()+i-idx);
                    cars_running.push_back(car);
                    idx++;
                }
                delete path;
            }
        }
        delete car_idx;
    }
    warm_uper.warm_up_end(car_vec);
    
  /* End of WARM_UP */


    vector<CAR*>::iterator start = car_vec.begin();
    bool is_new_round=false;//判断是否是新的一轮调度
    int count=0;//缓冲计时器

    while (true) {

        //是否没有车辆需要调度
        if (car_vec.empty()) break;

        global_time++;
        
        if (car_vec.end() - start >= BATCH_SIZE) {
            for (element_idx = BATCH_SIZE - 1; element_idx >= 0; element_idx--) {
                //找到可以出发车辆
                if ((*(start + element_idx))->plan_time <= global_time)
                    break;
            }
            //没有找到可以出发的车辆，进入下一轮循环
            if (element_idx == -1)
                continue;
        } else {
            //剩余车辆不足需要调度的数量，直接全部调度出去
            element_idx = car_vec.end() - start - 1;
        }
    
        //释放行驶车辆的占用容量以及删除理论到达车辆
        release_capacity(cars_running, global_time);

    
        //调度车辆
        for (vector<CAR*>::iterator car = start; car != start + element_idx + 1;)
        {
        get_factor((*car),global_time);

            //(*car)->capacity_factor=get_factor(*car);
            //需要在内部解决开销容量减少问题，以及记录理论到达各个node的时间
            GRAPH::route_type * result = graph.get_least_cost_route(*car, global_time);
            if (result->empty()) {
                car++;
                continue;
            } else {
                write_to_file(result, (*car), fout);
                cars_running.push_back((*car));
                car=car_vec.erase(car);
                element_idx--;
            }
            delete result;
        }
        
        //加入车辆inser
        start += element_idx+1;
        if(start>=car_vec.end())
        {
            start=car_vec.begin();
            is_new_round=true;
        }
    }
/* End of 调度车辆 */

    fout.close();

/* free memory */
    /* delete CAR* */
    for(auto val:car_vec){delete val;}

    /* delete ROAD* */
    for(auto val:road_map){delete val.second;}
/* End of free memory */

    auto __end_time = std::chrono::steady_clock::now();
    std::chrono::duration<double, std::milli> elapsed = __end_time - __start_time; // std::micro 表示以微秒为时间单位
    std::cout << "time: " << elapsed.count() << "ms." << endl;

    return 0;
}
