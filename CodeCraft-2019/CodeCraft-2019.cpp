#include <iostream>
#include <cstdio>
#include <string>

#include <chrono>   // speed test

#include "lib/object.hpp"

#define MAXIMUM_LENGTH_PER_LINE    50      // 每行字符个数不超过 MAXIMUM_LENGTH_PER_LINE
#define CAR_VECTOR_RESERVE_SIZE    11000   // 保存车辆信息的vector的预分配空间
#define NODE_VECTOR_RESERVE_SIZE   360     // 预计边的数量（考虑双向），假设图包含n*n个路口，则设置为 4n(n-1)
#define USE_CXX_SSTREAM            0
#define MAXIMUM_ROAD_LENGTH        20      // 道路的最大长度

using namespace std;

static int global_time = 0; //q全局时间

int main(int argc, char *argv[])
{
    auto __start_time = std::chrono::steady_clock::now();

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
    vector<CAR*> car_vec;             // attention: you should delete CAR* manually!!!
    car_vec.reserve(CAR_VECTOR_RESERVE_SIZE);
    bool car_speed_detect_array[MAXIMUM_ROAD_LENGTH] = {false};    // 

    int id, from, to, speed, plan_time;

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
            sscanf(line_buffer, "(%d, %d, %d, %d, %d)",&id, &from, &to, &speed, &plan_time);
            car_vec.push_back(new CAR(id, from, to, speed, plan_time));
            car_speed_detect_array[speed] = true;
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


  /* Read road configuration, and build GRAPH at the same time */
    map<int, ROAD*> road_map;       // attention: you should delete ROAD* manually!!!
    //vector<ROAD*> road_vec;
    //road_vec.reserve(ROAD_VECTOR_RESERVE_SIZE);

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
            //road_vec.push_back(pRoad);
            road_map.insert(pair<int, ROAD*>(id, pRoad));
        //}
    }
    fclose(fptr);
    graph.add_weights(car_speed_detect_array, MAXIMUM_ROAD_LENGTH);

  /*End of read road configuration */

/* END of read configuration from file */


/* 寻路函数测试*/
    // int i=1;
    // while(i<64){
    //     i++;
    //     auto start = std::chrono::steady_clock::now();
    //     auto tem_vec = graph.get_least_cost_route(0,i,2);
    //     auto end = std::chrono::steady_clock::now();
    //     std::chrono::duration<double, std::micro> elapsed = end - start; // std::micro 表示以微秒为时间单位

    //     std::cout << "time: " << elapsed.count() << "us: ";
    //     if(tem_vec.empty()){cout << "not find road!" << endl; continue;}
    //     for_each(tem_vec.rbegin(), tem_vec.rend(), [](GRAPH::Node* val)->void{cout<<val->cross_id << ' ';});
    //     //tem_vec.front()->capacity -= 10;
    //     cout << endl;
    // }
/* End of 寻路函数测试*/


/* 打开输出文件*/
    ofstream fout(answerPath);
    if (!fout.is_open()){
        cerr << "无法打开文件 " << answerPath << endl;
        exit(0);
    }
/*End of 打开输出文件*/  


/* 调度车辆 */
    list<CAR*> cars_running;//已经在运行的车辆
    list<CAR*> cars_waiting;//等待的车辆

    //取前面一百个
    int element_idx;
    vector<CAR*>::iterator start = car_vec.begin();

    while (true) {
        //是否没有车辆需要调度
        if (start >= car_vec.end() && cars_waiting.empty() ) break;

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
        release_capacity(cars_running, global_time, &graph);

        //需要在内部解决开销容量减少问题，以及记录理论到达各个node的时间
        for (list<CAR*>::iterator car = cars_waiting.begin(); car != cars_waiting.end();) {
            GRAPH::route_type * result = graph.get_least_cost_route(*car, global_time);
            if (!result->empty()) {
                write_to_file(result, (*car), fout);
                cars_running.push_back(*car);
                cars_waiting.erase(car++);
            } else {
                ++car;
            }
            delete result;
        }

        //调度车辆
        for (vector<CAR*>::iterator car = start; car != start + element_idx + 1; car++)
        {
            //需要在内部解决开销容量减少问题，以及记录理论到达各个node的时间
            GRAPH::route_type * result = graph.get_least_cost_route(*car, global_time);
            if (result->empty()) {
                cars_waiting.push_back(*(car));
                continue;
            } else {
                write_to_file(result, (*car), fout);
                cars_running.push_back((*car));
            }
            delete result;
        }

        //加入车辆inser
        start += element_idx+1;
    }
/* End of 调度车辆 */
 
    fout.close();

/* free memory */
    /* delete CAR* */
    for_each(car_vec.begin(), car_vec.end(), [](CAR* pCar)->void{delete pCar;});
    /* delete ROAD* */
    for_each(road_map.begin(), road_map.end(), [](const pair<int, ROAD*> & val)->void{delete val.second;});
/* End of free memory */

    auto __end_time = std::chrono::steady_clock::now();
    std::chrono::duration<double, std::milli> elapsed = __end_time - __start_time; // std::micro 表示以微秒为时间单位
    std::cout << "time: " << elapsed.count() << "ms: ";
    
    return 0;
}
