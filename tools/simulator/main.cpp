#include <string>
#include <chrono>   // speed test
#include <sstream>
#include <cstdio>

#include "lib/object.hpp"

using namespace std;

int waitStateCarCount = 0;
int totalCarCount = 0;
int totalCarRunTime = 0;
int totalPriParRunTime = 0;
int lastPriCarEndTime = 0;
/**
 * @key  `road_id`;
 * @value  value: `ROAD*`;
 */
map_type<int, ROAD *> roadMap;

/**
 * @key: `cross_id`;
 * @value: `CROSS*`
 */
map_type<int, CROSS *> crossMap;
vector<CROSS *> crossVec;

static int global_time = 0; //全局时间

int main(int argc, char *argv[])
{
    auto __start_time = std::chrono::steady_clock::now();
    std::ios::sync_with_stdio(false);

    vector<ROAD *> roadVec;
    map_type<int, CAR*> carMap;

/* SDK code */
    cout << "Begin" << endl;
    
    if(argc < 6){
        cout << "please input args: carPath, roadPath, crossPath, presetAnswerPath, answerPath" << std::endl;
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


  /* 读取车辆信息配置文件，将车辆放入到对应的路口的车库中 */
    unsigned int minPriCarPlanTime = -1;
    fptr = fopen(carPath.c_str(), "r");
    if (fptr) {
        int car_id, from, to, speed, planTime, isPrior, isPreset;
        char line_buffer[MAXIMUM_LENGTH_PER_LINE];

        fgets(line_buffer, 50, fptr); // 似乎只有文件第一行为注释......
        while (fgets(line_buffer, 50, fptr)) {
            //if(line_buffer[0] == '(') {
                sscanf(line_buffer, "(%d, %d, %d, %d, %d, %d, %d)",
                       &car_id, &from, &to, &speed, &planTime, &isPrior, &isPreset);

                auto pCar = new CAR(car_id, from, to, speed, planTime, isPrior, isPreset);
                carMap.insert(pair<int, CAR *>(car_id, pCar));
                crossMap[from]->garage.push_back(pCar);
                if(isPrior) {
                    crossMap[from]->priorGarage.push_back(pCar);
                    if (planTime < minPriCarPlanTime) minPriCarPlanTime = planTime;
                } else {
                    crossMap[from]->ordinaryGarage.push_back(pCar);
                }
                totalCarCount++;
            //}
        }
        fclose(fptr);
    } else {
        cout << "can't open car configuration file: " << carPath << endl;
        exit(-1);
    }
  /* End of read car configuration */

  /* 读取 presetAnswer 的信息 */ 
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

                pCar = carMap[car_id];
                pCar->answerTime = time;
                pre_cross_id = pCar->from;

                while(line_stream >> temp >> road_id) {
                    pContainer = roadMap[road_id]->getContainer(pre_cross_id).second;
                    pCar->route.push(pContainer);
                    pre_cross_id = pContainer->nextCrossId;
                }
            //}
        }
        fin.close();
    } else {
        cout << "can't open presetAnswer configuration file: " << presetAnswerPath << endl;
        exit(-1);
    }
  /* End of 读取预置车辆信息 */


  /* 读取 answer 的信息,认为 answer.txt 中包含了 preSetAnswer.txt */ 
    fin.open(answerPath);
    if(fin) {
        int car_id, time, road_id, pre_cross_id;
        char temp;
        string line_buffer;
        CAR *pCar;
        Container *pContainer;
        line_buffer.reserve(150);

        while(getline(fin, line_buffer)){
            if(line_buffer[0] == '('){
                istringstream line_stream(line_buffer);
                line_stream >> temp >> car_id >> temp >> time;

                pCar = carMap[car_id];

                if(pCar->planTime > time) {cout << "[CarId: " << car_id << "] - start time less than plan time!" << endl; return 0;}
                if(pCar->isPreset) continue;

                pCar->answerTime = time;
                pre_cross_id = pCar->from;

                while(line_stream >> temp >> road_id) {
                    pContainer = roadMap[road_id]->getContainer(pre_cross_id).second;
                    pCar->route.push(pContainer);
                    pre_cross_id = pContainer->nextCrossId;
                }
            }
        }
        fin.close();
    } else {
        cout << "can't open answer file: " << answerPath << endl;
        exit(-1);
    }
  /* End of 读取预置车辆信息 */

/* END of read configuration from file */


/* 对车库内车辆排序 */
    for(auto val : crossVec){
        // 车辆上路的优先级比较函数，优先车辆优先级最高，其次考虑车辆id。优先级高的放在前面，id小的放在前面
        sort(val->garage.begin(), val->garage.end(), [](CAR *a, CAR *b) -> bool {
            return a->isPrior == b->isPrior ? a->id < b->id : a->isPrior > b->isPrior;
        });
        sort(val->priorGarage.begin(), val->priorGarage.end(), [](CAR *a, CAR *b) -> bool {
            return a->answerTime == b->answerTime ? a->id < b->id : a->answerTime < b->answerTime;
        });
        sort(val->ordinaryGarage.begin(), val->ordinaryGarage.end(), [](CAR *a, CAR *b) -> bool {
            return a->answerTime == b->answerTime ? a->id < b->id : a->answerTime < b->answerTime;
        });
    }
/* End of 对车库内车辆排序 */


/* 系统运行 */
    int waitStateCarCountBK;
    while(totalCarCount)
    {
        global_time++;

        // 调度所有道路内的车辆 driveCarJustCurrentRoad()
        for(auto val:roadVec) { val->dispatchCarOnRoad(); }

        waitStateCarCountBK = waitStateCarCount;

        // 进行一次发车 ,仅优先车辆
        //for(auto val:crossVec) { val->driveCarInitList(true, global_time);}
        for(auto val:crossVec) { val->drivePriorCarInGarage(global_time, nullptr);}

        // 驱动所有等待车辆进入END状态
        while(waitStateCarCount) {
            // 按Cross的id升序依次调度每个路口
            for(auto cross : crossVec) { cross->dispatch(global_time); }

            // 判断是否死锁
            if(waitStateCarCount == waitStateCarCountBK) {
                cout << "dead lock" << endl;
                return 0;
            };

            waitStateCarCountBK = waitStateCarCount;
        }

        // 优先、非优先均上路
        //for(auto val:crossVec) { val->driveCarInitList(false, global_time);}
        for(auto val:crossVec) { val->driveAllCarInGarage(global_time);}

        cout << "Time: " << global_time << "; Remaining Car: " << totalCarCount << endl;
    }
/* 系统运行结束 */

    cout << "\n   priCarScheduleTime: " << lastPriCarEndTime-minPriCarPlanTime << endl;
    cout << "allPriCarScheduleTime: " << totalPriParRunTime << endl;
    cout << "      CarScheduleTime: " << global_time << endl;
    cout << "   allCarScheduleTime: " << totalCarRunTime << endl;

/* free memory */
    for(auto val:crossVec){delete val;}
    for(auto val:roadVec){delete val;}
/* End of free memory */

    auto __end_time = std::chrono::steady_clock::now();
    std::chrono::duration<double, std::milli> elapsed = __end_time - __start_time; // std::micro 表示以微秒为时间单位
    std::cout << "\nProgram Run Time: " << elapsed.count() << "ms.\n" << endl;

    return 0;
}
