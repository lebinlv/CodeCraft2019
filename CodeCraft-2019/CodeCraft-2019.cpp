#include <string>
#include <chrono>   // speed test
#include <sstream>
#include <cstdio>
#include <random>

#include "lib/object.hpp"

using namespace std;

int waitStateCarCount = 0;
int totalCarCount = 0;
int start_car_count = 0;

map_type<int, ROAD *> roadMap;

map_type<int, CROSS *> crossMap;

ofstream fout;

/**
 * @brief 长度为 `SPEED_DETECT_ARRAY_LENGTH`(比最大车速大1) 数组，用于标记有多少种车速
 */
bool speedDetectArray[SPEED_DETECT_ARRAY_LENGTH] = {false};
static int global_time = 0; //全局时间

int main(int argc, char *argv[])
{
    auto __start_time = std::chrono::steady_clock::now();
    std::ios::sync_with_stdio(false);

    /**
     * @brief  按 cross id 升序排列的 `CROSS*`;
     * @Attention:  you should delete `CROSS*` in `crossVec` manually
     */
    vector<CROSS *> crossVec;
    vector<ROAD *> roadVec;
vector<CAR *> carVec;

    map_type<int, CAR*> presetCarMap;

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

    GRAPH graph;

/* Read configuration from file */
    FILE *fptr;

  /* 读取道路配置文件，构建 `roadMap` */
    fptr = fopen(roadPath.c_str(), "r");
    if (fptr) {
        int id, length, speed, channel, from, to, isDuplex;
        char line_buffer[MAXIMUM_LENGTH_PER_LINE];
        roadVec.reserve(ROAD_VECTOR_RESERVE_SIZE);

        fgets(line_buffer, 50, fptr); // 似乎只有文件第一行为注释......
        while (fgets(line_buffer, 50, fptr)) {
            //if(line_buffer[0] == '(') {
                sscanf(line_buffer,"(%d, %d, %d, %d, %d, %d, %d)", 
                    &id, &length, &speed, &channel, &from, &to, &isDuplex);
                auto pRoad = new ROAD(id, length, speed, channel, from, to, isDuplex);
                roadMap.insert(pair<int, ROAD*>(id, pRoad));
                roadVec.push_back(pRoad);
            //}
        }
        fclose(fptr);
    } else {
        cout << "can't open road configuration file: " << roadPath << endl;
        exit(-1);
    }
  /* End of Read road configuration */


  /* 读取路口信息文件，构建 `crossVec`, `crossMap`，并对`crossVec`排序 */
    fptr = fopen(crossPath.c_str(), "r");
    if (fptr) {
        int cross_id, roadId[4];
        char line_buffer[MAXIMUM_LENGTH_PER_LINE];
        crossVec.reserve(CROSS_VECTOR_RESERVE_SIZE);

        fgets(line_buffer, 50, fptr); // 似乎只有文件第一行为注释......
        while (fgets(line_buffer, 50, fptr)) {
            //if(line_buffer[0] == '(') {
                sscanf(line_buffer,"(%d, %d, %d, %d, %d)", 
                      &cross_id, roadId, roadId+1, roadId+2, roadId+3);
                auto pCross = new CROSS(cross_id, roadId);
                crossMap.insert(pair<int, CROSS*>(cross_id, pCross));
                crossVec.push_back(pCross);
            //}
        }
        fclose(fptr);
        // 将CROSS按id升序排列
        sort(crossVec.begin(), crossVec.end(), [](CROSS *a, CROSS *b) -> bool { return a->id < b->id; });
    } else {
        cout << "can't open cross configuration file: " << crossPath << endl;
        exit(-1);
    }
  /* End of Read cross configuration */


  /* 读取车辆信息配置文件，构建`presetCarMap`，检测共有多少种车速，将车辆放入到对应的路口的车库中 */
    fptr = fopen(carPath.c_str(), "r");
carVec.reserve(70000);
        if (fptr) {
            int car_id, from, to, speed, planTime, isPrior, isPreset;
            char line_buffer[MAXIMUM_LENGTH_PER_LINE];
            for (int i = 0; i < SPEED_DETECT_ARRAY_LENGTH; i++) speedDetectArray[i] = false;

            fgets(line_buffer, 50, fptr); // 似乎只有文件第一行为注释......
            while (fgets(line_buffer, 50, fptr)) {
                //if(line_buffer[0] == '(') {
                    sscanf(line_buffer, "(%d, %d, %d, %d, %d, %d, %d)",
                        &car_id, &from, &to, &speed, &planTime, &isPrior, &isPreset);

                    auto pCar = new CAR(car_id, from, to, speed, planTime, isPrior, isPreset);
carVec.push_back(pCar);
                crossMap[from]->garage.push_back(pCar);
                speedDetectArray[speed] = true; //标记车速
                if(isPreset) presetCarMap.insert(pair<int, CAR*>(car_id, pCar));
                totalCarCount++;
            //}
        }
        fclose(fptr);
    } else {
        cout << "can't open car configuration file: " << carPath << endl;
        exit(-1);
    }
  /* End of read car configuration */


/* 计算路由表,并对车库内车辆排序 */
    graph.calculateCostMap();
    for(auto val : crossVec) {val->updateRouteTable();}
/* End of 计算路由表，并对车库内车辆排序 */



  /* 读取预置车辆的信息 */
    vector<pair<int, float> > presetCarVec;
    presetCarVec.reserve(512);
    ifstream fin(presetAnswerPath);
    if(fin) {
        int car_id, time, road_id, pre_cross_id;
        char temp;
        string line_buffer;
        CAR *pCar;
        Container *pContainer;
        line_buffer.reserve(150);

        getline(fin, line_buffer); // 似乎只有文件第一行为注释......
        while(getline(fin, line_buffer)){
            //if(line_buffer[0] == '('){
                istringstream line_stream(line_buffer);
                line_stream >> temp >> car_id >> temp >> time;

                pCar = presetCarMap[car_id];
                pCar->planTime = time;
                pre_cross_id = pCar->from;
                int cost = 0;

                while(line_stream >> temp >> road_id) {
                    pContainer = roadMap[road_id]->getContainer(pre_cross_id).second;
                    pre_cross_id = pContainer->nextCrossId;
                    pCar->route.push_back(pContainer);
                    cost += pContainer->length/min(pContainer->maxSpeed, pCar->speed);
                }
                float idealCost = crossMap[pCar->from]->lookUp(pCar->speed, 0, pCar->to).second;
                presetCarVec.push_back(pair<int, float>(pCar->id, cost-idealCost));
                reverse(pCar->route.begin(), pCar->route.end());
            //}
        }
    } else {
        cout << "can't open presetAnswer configuration file: " << presetAnswerPath << endl;
        exit(-1);
    }

    sort(presetCarVec.begin(), presetCarVec.end(), [](pair<int, float> & a, pair<int, float> & b)->bool{return a.second > b.second;});
    int modifyCount = presetCarVec.size()*0.1;
    for(int i=0; i<modifyCount; i++) {
        auto car = presetCarMap[presetCarVec[i].first];
        car->route.clear();
        car->isPreset = false;
    }
  /* End of 读取预置车辆信息 */

/* END of read configuration from file */



/* 打开输出文件*/
    char out_file_buffer[OUT_FILE_RESERVE_SPACE*1024];
    fout.rdbuf()->pubsetbuf(out_file_buffer, OUT_FILE_RESERVE_SPACE*1024);
    fout.open(answerPath);
    if (!fout.is_open()){
        cerr << "无法打开文件 " << answerPath << endl;
        exit(0);
    }
/*End of 打开输出文件*/

    int count =0;
    default_random_engine e;
    e.seed(0);
    bernoulli_distribution u(0.92);

/* 系统运行 */
    int waitStateCarCountBK;
    while(totalCarCount)
    {
        global_time++;count=0;

        for(auto cross:crossVec){
            cross->disable = u(e);
            for(auto car:cross->garage){
                car->notFindRoadThisTime = false;
                if(!car->isPreset && car->planTime < global_time)car->planTime=global_time;
            }

            sort(cross->garage.begin(), cross->garage.end(), [](CAR *a, CAR *b)->bool{
                if(a->isPrior == b->isPrior){
                    if(a->planTime == b->planTime){
                        return a->id < b->id;
                    }
                    return a->planTime < b->planTime;
                }
                return a->isPrior > b->isPrior;
            });
        }

        // 调度所有道路内的车辆 driveCarJustCurrentRoad()
        for(auto val:roadVec) { val->dispatchCarOnRoad(); }

        for (auto val : carVec)
        { 
            if (val->state == CAR::WAIT) 
            { count++; }
        }

        waitStateCarCountBK = waitStateCarCount;

        // 进行一次发车 ,仅优先车辆
        for(auto val:crossVec) { val->driveCarInitList(true, global_time);}

        // 驱动所有等待车辆进入END状态
        while(waitStateCarCount) {
            // 按Cross的id升序依次调度每个路口
            for(auto cross : crossVec) { cross->dispatch(global_time); }

            // 判断是否死锁
            if(waitStateCarCount == waitStateCarCountBK) {
                cout << "dead lock " << waitStateCarCount << "; WaitCar " << count << ":Total Run Count" << start_car_count<<endl;

                fout.close();
                return 0;
            };

            waitStateCarCountBK = waitStateCarCount;
        }

        for (auto val : carVec) { if (val->state == CAR::WAIT){ count++; } }

        // 优先、非优先均上路
        for(auto val:crossVec) { val->driveCarInitList(false, global_time);}
        cout << "time: " << global_time << " Can On:" << start_car_count<<endl;
    }
/* 系统运行结束 */


    fout.close();

/* free memory */
    for(auto val:crossVec){delete val;}
    for(auto val:roadVec){delete val;}
    for(auto val:carVec){delete val;}
/* End of free memory */

    auto __end_time = std::chrono::steady_clock::now();
    std::chrono::duration<double, std::milli> elapsed = __end_time - __start_time; // std::micro 表示以微秒为时间单位
    std::cout << "time: " << elapsed.count() << "ms." << endl;

    return 0;
}
