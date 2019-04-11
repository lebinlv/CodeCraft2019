#ifndef _OBJECT_H_
#define _OBJECT_H_

#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <deque>
#include <list>
#include <stack>
#include <map>
#include <unordered_map>
#include <algorithm>

#include <cstdint>

#include "define.hpp"


class Container;

struct CAR
{
    // 车辆的运行状态
    enum CAR_STATE {WAIT, END};

    int id, from, to, speed, planTime;
    bool isPrior, isPreset;

    // 车的实际出发时间
    int startTime;

    // 该车当前所在车道， 该车在上一道路中的车道;
    // 车在当前道路上的位置, `idx ~ [0, length-1]`, 越小表示离出口越近;
    // 车在当前道路上的速度;
    int preChannel, currentChannel, currentIdx, currentSpeed;

    int nextSpeed, turnWeight;
    bool getNewRoad;

    // 车的运行状态
    CAR_STATE state;

    // 记录该车辆的行驶路线
    std::vector<Container *> route;


    CAR(int _id, int _from, int _to, int _speed, int _time, bool _isPrior, bool _isPreset):
        id(_id), from(_from), to(_to), speed(_speed), planTime(_time),
        isPrior(_isPrior), isPreset(_isPreset), currentIdx(0), state(END){
            route.reserve(GARAGE_RESERVE_SIZE);
    }
    ~CAR() {}

    CAR(const CAR &) = delete;
    CAR(CAR &&) = delete;
    CAR() = delete;

    /**
     * @brief 当车辆进入下一条道路时，调用本函数更新车辆信息
     * 
     * @param newSpeed  车辆在下一条道路上的行驶速度；
     * @param newIdx    车辆在下一条道路上的初始位置
     * @param newChannel 车辆在下一条道路上的车道；
     */
    inline void enterNewRoad(int newIdx, int newChannel);

    // 车辆出路口时的优先比较函数，服务于 priority_queue
    struct CompareWhenTurn {
        bool operator()(CAR* a, CAR* b) {
          if (a->isPrior == b->isPrior) //如果优先级相同
            // 如果位置相同，则车道小的优先，否则位置小的优先
            return (a->currentIdx == b->currentIdx) ? a->currentChannel > b->currentChannel : a->currentIdx > b->currentIdx;
          else // 如果优先级不同，则优先级高的优先
            return a->isPrior < b->isPrior;
        }
    };

    // 车辆上路的优先级比较函数，服务于sort
    //排序优先级第一，id第二。优先级高的放在前面，id小的放在前面
    struct ComparaInGarage{
        bool operator()(CAR* a, CAR* b) {
            return a->isPrior == b->isPrior ? a->id < b->id : a->isPrior > b->isPrior;
        }
    };
};


struct CROSS;

// 单侧道路
class Container
{
  public:
    typedef std::vector<CAR *> container_t;

    // push_back 函数的返回值， FULL_LOAD表示无剩余空间，SUCCESS表示成功进入
    enum PUSH_BACK_STATE{FULL_LOAD=-2, SUCCESS};

    // 道路id， 车道数， 道路长度， 限速， 出口crossId, 容量
    int roadId, channel, length, maxSpeed, nextCrossId, capacity;

    // turn_to[0], [1], [2] 分别指向 从当前道路右转、左转、直行后 到达的Container
    Container *turnTo[3] = {nullptr, nullptr, nullptr};

    // opposite[0], [1], [2] 分别指向右边、左边、前方道路中与当前Container进入同一路口的Container
    Container *opposite[3] = {nullptr, nullptr, nullptr};

    // 指向该道路的起点路口、终点路口的指针
    CROSS *startCross, *endCross;

    int infoIdx;
    static int containerCount;
    double probability; //道路的概率，服务于寻路方法

  private:
    container_t *carInChannel; // 构造函数中使用 new [] 生成
    std::priority_queue<CAR *, std::vector<CAR *>, CAR::CompareWhenTurn> priCar; // 出路口的车的优先队列

    /**
     * @brief 调度指定channel内的车辆; 仅每时刻第一次调度时使用；
     *        该函数涉及路径规划逻辑，目前做法：若车道内的第一辆车的剩余行驶距离小于该车的可行驶速度（即该车有可能出路口）,
     *        则调用路径规划函数为该车寻找路线，然后再依次调度该道路上的车辆。
     * 
     * @param channel_idx 指定的车道
     * @Attention 函数内部并不检查 channel_idx 是否越界
     */
    void dispatchCarInChannelFirst(int channel_idx);

  public:
    // 根据车道数和道路长度初始化容器
    Container(int _id, int _channel, int _length, int _maxSpeed, int _crossId);
    ~Container() { delete [] carInChannel; }

    Container() = delete;
    Container(const Container &) = delete;
    Container(Container &&) = delete;

    /**
     * @brief 获取当前道路内车辆数目
     * 
     * @return int 
     */
    int size(){
        int size=0;
        for(int i=0; i<channel; i++) size+=carInChannel[i].size();
        return size;
    }

    // 获取第 pos 车道的 车辆vector 的引用, `pos~[0, channel]`
    inline container_t &getCarVec(int pos){ return carInChannel[pos]; }
    inline container_t &operator[](int pos) { return carInChannel[pos]; }

    /**
     * @brief 加入新的车辆，若成功加入，则会更新 pCar的`currentChannel`,`currentIdx`等信息
     * 
     * @param pCar 要加入的车辆指针
     * @return `Container::FULL_LOAD`: 道路满载，无法放入新的车辆；
     *         `Container::SUCCESS`: 成功加入；
     *         `others(>=0)`: 因等待车辆而无法进入，返回道路入口与等待车辆的距离
     */
    int push_back(CAR *pCar, int s2);

    /**
     * @brief 获取最先行驶的车辆的指针
     * 
     * @return 如果所有车辆都不会出路口，返回 nullptr
     */
    inline CAR *top() { return priCar.empty() ? nullptr : priCar.top(); }

    /**
     * @brief 从出路口优先队列中取出第一辆车, 注意，该函数会同时将该车从 内部vector中erase
     * 
     * @return true  成功取出
     * @return false 取出失败
     */
    void pop();

    /**
     * @brief 调度指定channel内的车辆; 仅有车辆成功出路口时调用；
     *        该函数涉及路径规划逻辑，目前做法：若车道内的第一辆车的剩余行驶距离小于该车的可行驶速度（即该车有可能出路口）,
     *        则调用路径规划函数为该车寻找路线，然后再依次调度该道路上的车辆。
     * 
     * @param channel_idx 指定的车道
     * @Attention 函数内部并不检查 channel_idx 是否越界
     * @Other 该函数仅调度指定车道上能在该车道内行驶后成为终止状态的车辆（对于调度后依然是等待状态的车辆不进行调度，且依然标记为等待状态）。
     *        同时会重新识别真正等待出路口的车辆，因为有些车辆是因为前车是等待状态而导致自己也是等待状态，他自己这次根本不会出路口，
     *        也就是其不参与出路口的优先级排序（只有出路口的车辆才参与优先级排序）
     */
    void dispatchCarInChannel(int channel_idx);

    /**
     * @brief 指定车道的第一辆车因下一道路满载而无法过路口时，该车应被标记为 END，然后对本车道的其他车进行一次调度
     * 
     * @param channel_idx 指定的车道
     * @Attention 函数内部并不检查 channel_idx 是否越界
     */
    void updateWhenNextRoadFull(int channel_idx);

    /**
     * @brief 调度容器内的车辆，仅每时刻第一次调度时使用
     * 
     */
    inline void dispatchCarInContainer(){
        for(int i=0; i<channel; i++) dispatchCarInChannelFirst(i);
    }

    /**
     * @brief 寻找路径
     * 
     * @param 需要调度的车辆，是否是初始上路车辆
     *  
     * @nitice 需要在道路上调用，传入一个car，之后会对car的route vector进行幅值
     * 
     * @return 如果找到路返回该条路，如果没有找到，返回null；这个返回值主要是给发车寻路使用的，对于已经上路的车辆，只会返回nullptr
     */
    void searchRoad(CAR *car);
};


class ROAD
{
  public:
    int id, length, maxSpeed, channel, from, to;
    bool isDuplex;

  private:
    // 车辆容器，如果为单向道路，则 backward 为空指针，构造函数中使用new生成
    Container *forward, *backward;

  public:
    ROAD(int _id, int _length, int _speed, int _channel, int _from, int _to, bool _isDuplex);
    ~ROAD() { delete forward; delete backward; }

    ROAD() = delete;
    ROAD(const ROAD &) = delete;
    ROAD(ROAD &&) = delete;

    /**
     * @brief 获取该道路上进入和离开指定路口的车辆的容器
     * 
     * @param cross_id 路口id 
     * @return std::pair<container ,container>  .first: 进入路口的车辆容器； .second: 离开路口的车辆容器
     */
    inline std::pair<Container* ,Container*> getContainer(int cross_id) {
        return cross_id == to ? std::pair<Container*, Container*>(forward, backward) : std::pair<Container*, Container*>(backward, forward);
    }

    /**
     * @brief 调度该道路上的车辆
     * 
     */
    inline void dispatchCarOnRoad(){ 
        forward->dispatchCarInContainer();
        if(isDuplex) backward->dispatchCarInContainer();
    };
};


struct CROSS
{
    typedef std::pair<Container*, float> routeInfo_t;
    int id;
    bool disable;

    int pre_time=-1;//用于发车的时间记忆
    
    // 记录CROSS总数
    static int crossCount;

    // `MERGE(id1, id2)` 表示 从 道路id1 转向 道路id2 的转向优先级；右转:0，左转:1，直行:2
    static map_type<int, uint8_t> turnMap;

    // 进入该路口的Container指针，按id升序排列
    std::vector<Container *>  enterRoadVec;
    // 离开该路口的Container指针, 无序排列
    std::vector<Container *>  awayRoadVec;
    // 车库，只用于发车...
    std::deque<CAR *> garage; 
    std::deque<CAR *> priorCarGarage;    // 优先车辆车库
    std::deque<CAR *> ordinaryCarGarage; // 普通车辆车库

  private:
    int drive_count;
    // 路由表的索引@release： uint32_t    | speed 6bit(0~63) | removeRoadId 14bit(0~16383) | 目的crossId 12bit(0~4095) |
    // 路由表的索引@debug： uint32_t    speed*1e8 + removeRoadId*1e4 + 目的crossId
    map_type<uint32_t, routeInfo_t> routeTable;
    routeInfo_t findNothing;

    /**
     * @brief 更新路由表的内部实现
     * 
     * @param speed 车速（不同的车速具有不同的路由表）
     * @param removeRoadId （对于要转向的车，查询路由表时要排除车辆来的道路）
     */
    void updateRouteTableInternall(int speed, int removeRoadId);

  public:
    /**
     * @brief 根据路口id和与其相连的道路构建cross
     * 
     * @param crossId 路口id
     * @param roadId  与路口相连的道路id数组，顺时针顺序
     */
    CROSS(int crossId, int roadId[]);
    ~CROSS(){}

    CROSS() = delete;
    CROSS(const CROSS &) = delete;
    CROSS(CROSS &&) = delete;


    //inline void sortCarInGarage(){sort(garage.begin(),garage.end(),CAR::ComparaInGarage);}

    /**
     * @brief 更新路由表，在将所有信息从文件中读取完毕后，必须调用此函数更新路由表
     */
    void updateRouteTable();

    /**
     * @brief 查找路由表
     * 
     * @param car_speed        车辆速度；
     * @param current_road_id  车辆所在道路的id；
     * @param destination      目的cross的id；
     * @return routeInfo_t&    `std::pair<Container*, float>` （<指向下一跳的指针, 预计总开销>）
     */
    inline routeInfo_t & lookUp(int car_speed, int current_road_id, int destination){
        uint32_t key = ROUTEID(car_speed, current_road_id, destination);
        if (routeTable.find(key) == routeTable.end()) return findNothing;
        return routeTable[ROUTEID(car_speed, current_road_id, destination)];
    }

    /**
     * @brief Get the Turn Direction
     * 
     * @param _from 车辆所在道路的id
     * @param _to   车辆要转去的道路的id
     * @return int  右转：0， 左转：1， 直行：0
     */
    static inline int getTurnDirection(int _from, int _to) { return turnMap[MERGE(_from, _to)];}

    /*
    * @brief 该函数分为两个内容，调度优先车辆与非优先车辆，具体使用见官方伪代码
    * @notice 这边需要garage已经排序完成，优先级第一，id第二。
    * @param 第一个是是否是优先车辆调度，按照伪代码那边来
    * 第二个是全局时间
    */
    void driveCarInitList(bool is_prior, int global_time);
    void drivePriorCarInGarage(int global_time, Container *road);
    void driveAllCarInGarage(int global_time);

    /**
     * @brief 路口调度
     * 
     * @param priority  true:只上路优先车辆  false:非优先车辆也可上路，但优先车辆先上路
     * @param global_time  时间
     */
    void dispatch(int global_time);

    Container *searchRoad(CAR *pCar);
};


struct GRAPH
{
    static std::vector<Container *> containerVec;
    static std::unordered_map<uint8_t, float *> costMap;
    static std::vector<uint8_t> speedDetectResultVec;

    GRAPH() { containerVec.reserve(CONTAINER_VECTOR_RESERVE_SIZE); };
    ~GRAPH(){ for(auto &val : costMap) delete [] val.second; };

    GRAPH(const GRAPH &) = delete;
    GRAPH(GRAPH &&) = delete;

    /**
     * @brief 计算不同速度通过各边的开销（粗略开销，length/min(car.speed, road.maxSpeed）)
     * @Attention 必须在读取road, car, cross文件后，更新路由表前调用
     */
    void calculateCostMap();
};


#endif