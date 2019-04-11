#include <cfloat>

#include "lib/object.hpp"

/*
*调参区间
*/
static double max_factor = 0.8; //用于道路容量计算的最大因子，也就是车数量>factor*容量，则初始概率为0
static double max_factor_drive_car = 0.025; //用于道路容量计算的最大因子，也就是车数量>factor*容量，则初始概率为0
static int batch_size = 0;

using namespace std;

extern ofstream fout;
extern int waitStateCarCount;
extern int totalCarCount;
extern map_type<int, ROAD*> roadMap;
extern map_type<int, CROSS *> crossMap;
extern int start_car_count;
extern bool speedDetectArray[SPEED_DETECT_ARRAY_LENGTH];


inline void CAR::enterNewRoad(int newIdx, int newChannel)
{
    preChannel = currentChannel;
    currentChannel = newChannel;

    currentIdx = newIdx;
    currentSpeed = nextSpeed;

    getNewRoad = false;
    state = END;
    // for (auto val : route) { std::cout << val->roadId << ", "; }
    // std::cout << std::endl;

    if (isPreset) route.pop_back();
}

/*************************** class Container *****************************/
int Container::containerCount = 0;
Container::Container(int _roadId, int _channel, int _length, int _maxSpeed, int _crossId):
                     roadId(_roadId), channel(_channel), length(_length), maxSpeed(_maxSpeed),
                     nextCrossId(_crossId), capacity(_channel * _length), 
                     startCross(nullptr), endCross(nullptr)
{
    carInChannel = new container_t [channel];
    for(int i=0; i<channel; ++i) {
        carInChannel[i].reserve(length);
    }
    infoIdx = containerCount++;
    GRAPH::containerVec.push_back(this);
}

int Container::push_back(CAR *pCar, int s2)
{
    //int s2 = max(0, pCar->nextSpeed - pCar->currentIdx);          // 计算出 pCar 在此道路上的可行距离 s2
    int new_idx = length - s2;

    CAR *pre_car;
    int pre_car_idx;

    // 逐车道遍历
    for (int i = 0; i < channel; i++) {
        // 如果当前车道为空
        if(carInChannel[i].empty()) {
            pCar->enterNewRoad(new_idx, i);
            carInChannel[i].push_back(pCar);
            return SUCCESS;
        }

        pre_car = carInChannel[i].back();
        pre_car_idx = pre_car->currentIdx;

        if (pre_car->state == CAR::END) { // 如果该车道最后一辆车状态为 END
            if (pre_car_idx < length-1) { // 如果该车道还有空间
                pCar->enterNewRoad((new_idx>pre_car_idx ? new_idx:pre_car_idx+1), i);
                carInChannel[i].push_back(pCar);
                return SUCCESS;
            }
        } else { // 如果最后一辆车是 WAIT
            if (new_idx > pre_car_idx) { // 如果不被阻挡，则成功进入并标记为END
                pCar->enterNewRoad(new_idx, i);
                carInChannel[i].push_back(pCar);
                return SUCCESS;
            } // End of 如果不被阻挡
            return length-pre_car_idx-1; // 被阻挡则返回该车道剩余空间大小
        } // End of 如果最后一辆车是WAIT
    } // End of 逐车道遍历
    return FULL_LOAD;
}

void Container::pop()
{
    // 获取车辆所在车道id
    auto &channelVec = carInChannel[priCar.top()->preChannel];

    // 删除该车
    channelVec.erase(channelVec.begin()); //TODO: 性能待优化
    priCar.pop();
}

void Container::dispatchCarInChannel(int channel_idx)
{
    auto & car_vec = carInChannel[channel_idx];
    int len = car_vec.size();

    if(len==0) return;    // 如果车道为空，则无需调度

    CAR *temp_car = car_vec.front(); // 取离出口最近的一辆车
    if(temp_car->state == CAR::END) return; 

    CAR::CAR_STATE pre_car_state = CAR::WAIT;
    int pre_car_idx = temp_car->currentIdx;
    int new_idx = pre_car_idx - temp_car->currentSpeed; // 计算新坐标

    if (new_idx < 0) // 若 new_idx<0，则该车有可能出路口
    {
        searchRoad(temp_car); //为车辆分配路线
        priCar.push(temp_car);  // 此时 state 必为 WAIT
    }
    else // 否则（即new_idx>=0） 该车无法出路口
    {
        temp_car->currentIdx = new_idx; // 则移动该车
        pre_car_idx = new_idx;
        temp_car->state = CAR::END;     // 并标记为END
        pre_car_state = CAR::END;
        waitStateCarCount--;//modify(temp_car);
    }

    if(pre_car_state == CAR::WAIT) return;

    // 第一辆车已处理过，接下来从第二辆开始遍历剩余车辆
    for(int i=1; i<len; i++) {
        temp_car = car_vec[i];
        if(temp_car->state == CAR::END) return;

        new_idx = temp_car->currentIdx - temp_car->currentSpeed;

        if(new_idx > pre_car_idx) { // 如果不受阻挡
            temp_car->currentIdx = new_idx; // 则移动该车
            pre_car_idx = new_idx;
            temp_car->state = CAR::END; // 并标记为 END
            waitStateCarCount--;//modify(temp_car);
        } else { // 如果受到阻挡
            pre_car_idx += 1;
            temp_car->currentIdx = pre_car_idx; //移动该车
            temp_car->state = CAR::END;
            waitStateCarCount--;//modify(temp_car);
        }
    } // End of 遍历剩余车辆
}

void Container::updateWhenNextRoadFull(int channel_idx)
{
    priCar.pop(); //此函数被调用说明优先队列第一辆车因前路满载无法进入前方道路，该车将变为End，需要将其从priCar pop

    int pre_car_idx = -1, new_idx; // 计算新坐标

    // 遍历车辆
    for(auto temp_car : carInChannel[channel_idx]) {
        if(temp_car->state == CAR::END) return;

        new_idx = temp_car->currentIdx - temp_car->currentSpeed;
        if(new_idx > pre_car_idx) { // 如果不受阻挡
            temp_car->currentIdx = new_idx; // 则移动该车
            pre_car_idx = new_idx;
            temp_car->state = CAR::END; // 并标记为 END
            waitStateCarCount--;//modify(temp_car);
        } else { // 如果受到阻挡
            pre_car_idx += 1;
            temp_car->currentIdx = pre_car_idx; //移动该车
            temp_car->state = CAR::END;
            waitStateCarCount--;//modify(temp_car);
        }
    } // End of 遍历剩余车辆
}

void Container::dispatchCarInChannelFirst(int channel_idx)
{
    auto & car_vec = carInChannel[channel_idx];
    int len = car_vec.size();

    // 如果车道为空，则无需调度
    if(len==0) return;

    CAR *temp_car = car_vec.front(); // 取离出口最近的一辆车

    int new_idx = temp_car->currentIdx - temp_car->currentSpeed; // 计算新坐标

    CAR::CAR_STATE pre_car_state = CAR::WAIT;
    int pre_car_idx = 0;

    if (new_idx < 0) // 若 new_idx<0，则该车有可能出路口
    {  
        // 若没有为该车分配过道路，则调用路径规划函数为其分配道路
        if(temp_car->getNewRoad == false) { searchRoad(temp_car); }

        pre_car_idx = temp_car->currentIdx; // 不移动该车
        temp_car->state = CAR::WAIT;        // 并将该车标记为WAIT
        waitStateCarCount++;

        priCar.push(temp_car); // 该车能出路口，所以将其放入出路口优先队列
    }
    else // 否则（即new_idx>=0） 该车无法出路口
    { 
        temp_car->currentIdx = new_idx; // 则移动该车
        pre_car_idx = new_idx;
        temp_car->state = CAR::END;     // 并标记为END
        pre_car_state = CAR::END;
    }

    // 第一辆车已处理过，接下来从第二辆开始遍历剩余车辆
    for(int i=1; i<len; i++){
        temp_car = car_vec[i];
        new_idx = temp_car->currentIdx - temp_car->currentSpeed;
        switch (pre_car_state) 
        {
            case CAR::WAIT:// 如果前车状态为 WAIT
            {
                if (new_idx > pre_car_idx) {  // 如果不受前车阻挡
                    temp_car->currentIdx = new_idx; // 则移动该车
                    pre_car_idx = new_idx;
                    temp_car->state = CAR::END; // 并标记为 END
                    pre_car_state = CAR::END;
                } else { // 如果受前车阻挡
                    pre_car_idx = temp_car->currentIdx; // 则不移动该车
                    temp_car->state = CAR::WAIT; //并标记为 WAIT
                    waitStateCarCount++;
                }
            } break;

            case CAR::END: // 如果前车状态为 END
            {
                if(new_idx > pre_car_idx) { // 如果不受阻挡
                    temp_car->currentIdx = new_idx; // 则移动该车
                    pre_car_idx = new_idx;
                    temp_car->state = CAR::END; // 并标记为 END
                } else { // 如果受到阻挡
                    pre_car_idx += 1;
                    temp_car->currentIdx = pre_car_idx; //移动该车
                    temp_car->state = CAR::END;
                }
            } break;
        } // End of switch
    } // End of 遍历剩余车辆
}

void Container::searchRoad(CAR* car)
{
    //如果下一个路口是目的地
    if(nextCrossId == car->to)
    {
        car->turnWeight = 2; // 当做直行处理
        car->nextSpeed = car->currentSpeed;
        car->preChannel = car->currentChannel;
        car->getNewRoad = true;
        return;
    }

    // 预置车辆
    if (car->isPreset)
    {
        Container *next_road = car->route.back();
        car->nextSpeed = min(car->speed, next_road->maxSpeed);
        car->turnWeight = CROSS::getTurnDirection(this->roadId, next_road->roadId);
        car->getNewRoad = true;
        return;
    }

    int speed=car->speed;
    int destination = car->to;

    Container *best_road = endCross->lookUp(speed, roadId, destination).first; //最优的路

    //最优的路径没有满载
    if(best_road->size() <= max_factor*best_road->capacity) {
        car->nextSpeed = min(car->speed, best_road->maxSpeed);
        car->turnWeight = CROSS::getTurnDirection(this->roadId, best_road->roadId);
        car->getNewRoad = true;
        (car->route).push_back(best_road);
        return;
    }


    float *cost_array = GRAPH::costMap[speed];
    float distance=FLT_MAX;//最小距离变量
    float local_distance;//局部距离变量
    bool is_second_best_exist=false;//标记是否第二近的路存在
    Container *second_best_road;    //第二近的路的临时保存变量

    for (auto i : endCross->awayRoadVec)
    {
        if(i==best_road || i->roadId==this->roadId) continue;

        //得到本道路到目的地的开销
        local_distance=i->endCross->lookUp(speed,i->roadId, destination).second;
        if(local_distance > 0) {
            local_distance += cost_array[i->infoIdx];

            if(local_distance<distance && i->size() <= max_factor*i->capacity)
            {
                distance=local_distance;
                second_best_road=i;
                is_second_best_exist=true;
            }
        }
    }

    if(is_second_best_exist)//第二优的路存在
    {
        car->nextSpeed = min(speed, second_best_road->maxSpeed);
        car->turnWeight = CROSS::getTurnDirection(this->roadId, second_best_road->roadId);
        car->getNewRoad = true;
        (car->route).push_back(second_best_road);
    }
    else//第二优的路不存在，直接取得最优路放进去
    {
        car->nextSpeed = min(speed, best_road->maxSpeed);
        car->turnWeight = CROSS::getTurnDirection(this->roadId, best_road->roadId);
        car->getNewRoad = true;
        (car->route).push_back(best_road);
    }

    return;
}

/*************************** end of class Container *****************************/



/*************************** class ROAD  *****************************/
ROAD::ROAD(int _id, int _length, int _speed, int _channel, int _from, int _to, bool _isDuplex) : 
           id(_id), length(_length), maxSpeed(_speed), channel(_channel),
           from(_from), to(_to), isDuplex(_isDuplex)
{
    forward = new Container(_id, _channel, _length, _speed, to);
    backward = _isDuplex ? new Container(_id, _channel, _length, _speed, from) : nullptr;
}
/*************************** end of class ROAD *****************************/



/*************************** class CROSS *****************************/
map_type<int, uint8_t> CROSS::turnMap;
int CROSS::crossCount = 0;
CROSS::routeInfo_t findNothing = CROSS::routeInfo_t(nullptr, -1);

CROSS::CROSS(int crossId, int roadId[]):id(crossId)
{
    Container *enter[4] = {nullptr, nullptr, nullptr, nullptr};
    Container *away[4] = {nullptr, nullptr, nullptr, nullptr};

    for(uint8_t i=0; i<4; ++i) {
        if(roadId[i] > 0) {
            auto val = roadMap[roadId[i]]->getContainer(id);
            enter[i] = val.first;
            away[i] = val.second;
        }
    }

    Container *temp;
    int real_idx, weight_idx;
    for(uint8_t i=0; i<4; ++i) {
        temp = enter[i];
        if(temp) {
            for(uint8_t j=i+1; j<i+4; ++j) {
                real_idx = j&3; // j&3 == j%4
                weight_idx = (j-i)%3;
                temp->opposite[weight_idx] = enter[real_idx];
                if(away[real_idx]) {
                    temp->turnTo[weight_idx] = away[real_idx];
                    turnMap.insert(pair<int, uint8_t>(MERGE(roadId[i], roadId[real_idx]),weight_idx));
                }
            }
            temp->endCross = this;
            enterRoadVec.push_back(temp);
        }
        if(away[i]) {
            away[i]->startCross = this;
            awayRoadVec.push_back(away[i]);
        }
    }

    sort(enterRoadVec.begin(), enterRoadVec.end(), [](Container *a, Container *b)->bool{return a->roadId < b->roadId;});
    crossCount++;
}

struct Node
{
    float cost;
    int crossId;
    Container *first;

    Node(int _cost, int _id, Container * _first):cost(_cost), crossId(_id), first(_first){}

    struct Compare{
        inline bool operator()(Node *a, Node *b){return a->cost > b->cost;}
    };
};

void CROSS::updateRouteTable()
{
    for(auto car_speed : GRAPH::speedDetectResultVec) {
        for(auto val : enterRoadVec){
            updateRouteTableInternall(car_speed, val->roadId);
        }
        updateRouteTableInternall(car_speed, 0);
    }
}

void CROSS::updateRouteTableInternall(int speed, int removeRoadId)
{
#if __DEBUG_MODE__
    // 路由表的索引： uint32_t    speed*1e8 + removeRoadId*1e4 + 目的crossId
    uint32_t temp_idx = speed*1e8 + removeRoadId*1e4;
#else
    // 路由表的索引： uint32_t    | speed 6bit(0~63) | removeRoadId 14bit(0~16383) | 目的crossId 12bit(0~4095) |
    uint32_t temp_idx = (speed<<26) | (removeRoadId<<12);
#endif

    float *pWeight = GRAPH::costMap[speed]; // 根据车速重定向 pWeight
    int count = 1; // 记录已经经过的节点数,如果等于总路口数则退出
    bool not_find_new_node;

    priority_queue<Node *, vector<Node *>, Node::Compare> candidates;
    unordered_map<int, Node *> visited_map; // 记录节点是否被访问过

    Node *record = new Node(0, id, nullptr);
    visited_map.insert(pair<int, Node *>(id, record)); // 将起始节点标记为已访问


  /* 特殊对待第一次向外辐射 */
    for (auto node : awayRoadVec){
        if(node->roadId != removeRoadId) candidates.push(new Node(pWeight[node->infoIdx], node->nextCrossId, node));
    }

    if(candidates.empty()) return; // 如果第一次辐射就未找到节点，说明舍掉 removeRoadId 后出度为0，直接返回

    record = candidates.top(); // 取队列顶节点
    candidates.pop();
    visited_map.insert(pair<int, Node *>(record->crossId, record)); // 将当前节点标记为已访问

#if __DEBUG_MODE__
    routeTable.insert(pair<uint32_t, routeInfo_t >(temp_idx+record->crossId, routeInfo_t(record->first, record->cost)));
#else
    routeTable.insert(pair<uint32_t, routeInfo_t>(temp_idx|record->crossId, routeInfo_t(record->first, record->cost)));
#endif

    count++;
  /* 第一次辐射结束 */

    while (count < crossCount)
    {
        for (auto node : crossMap[record->crossId]->awayRoadVec){
            // 如果这个节点代表的边没有被访问过，建立Node实例, 并把它压入优先队列
            if (visited_map.find(node->nextCrossId) == visited_map.end()) {
                candidates.push(new Node(record->cost + pWeight[node->infoIdx], node->nextCrossId, record->first));
            }
        }

        // 一直pop直到队顶节点没被访问过
        not_find_new_node = true;
        while (!candidates.empty()) {
            record = candidates.top(); // 取队列顶节点

            // 如果在visited_map中没有找到该节点，即该节点未被访问过，则退出while
            if (visited_map.find(record->crossId) == visited_map.end()) {
                candidates.pop();
                // 将未被访问到的节点写入routeMap和visited_map
                visited_map.insert(pair<int, Node *>(record->crossId, record)); // 将当前节点标记为已访问

            #if __DEBUG_MODE__
                routeTable.insert(pair<uint32_t, routeInfo_t >(temp_idx+record->crossId, routeInfo_t(record->first, record->cost)));
            #else
                routeTable.insert(pair<uint32_t, routeInfo_t>(temp_idx|record->crossId, routeInfo_t(record->first, record->cost)));
            #endif
                not_find_new_node = false;
                count++;
                break;
            }
            // 否则就弹出顶点并释放资源
            delete record;
            candidates.pop();
        }

        if (not_find_new_node) {
            for (auto &val : visited_map) { delete val.second;}  // 释放 visited_map中的 Node*
            break;
        }
    }

    // 释放空间
    while(!candidates.empty()){ delete candidates.top(); candidates.pop(); } // 释放优先队列中的 Node*
    for (auto &val : visited_map){delete val.second;} // 释放 visited_map中的 Node*
}

Container *CROSS::searchRoad(CAR* pCar)
{
    int speed = pCar->speed;
    int destination = pCar->to;

    //最优的路径没有满载
    Container *best_road = lookUp(speed, 0, destination).first; //最优的路
    if(best_road->size() <= max_factor_drive_car*best_road->capacity)
    {
        return best_road;
    }

    float *cost_array = GRAPH::costMap[speed];
    float distance=FLT_MAX;             //最小距离变量
    float local_distance;               //局部距离变量
    bool is_second_best_exist=false;    //标记是否第二近的路存在
    Container *second_best_road;        //第二近的路的临时保存变量

    for(auto i : awayRoadVec)
    {
        //如果这条道路容量没有满，并且道路很近,并且不是最优路径
        if(i != best_road)
        {
            //得到本道路到目的地的开销
            local_distance=i->endCross->lookUp(speed,i->roadId, destination).second;

            if(local_distance > 0){
                local_distance += cost_array[i->infoIdx];

                if(local_distance<distance && i->size()<=max_factor_drive_car*i->capacity)
                {
                    distance=local_distance;
                    second_best_road=i;
                    is_second_best_exist=true;
                }
            }
        }
    }
    if(is_second_best_exist)//第二优的路存在
    {
        return second_best_road;
    }

    return nullptr;
}

void CROSS::driveCarInitList(bool is_prior,int global_time)
{
    if(pre_time != global_time){drive_count=0; pre_time=global_time;}

    if (is_prior) //如果is_prior是true，则只上路优先车辆
    {
        for (deque<CAR *>::iterator i = garage.begin(); i != garage.end();)
        {
            if (drive_count > batch_size) return;
            CAR *temp_car = *i;

            // 如果计划时间不晚于当前时间 且 是优先车辆
            if ((temp_car->planTime) <= global_time && (temp_car->isPrior)) 
            {
                if (temp_car->isPreset)
                {
                    Container *temp_road = temp_car->route.back();
                    if (temp_road->size() <= max_factor_drive_car * temp_road->capacity)
                    {
                        if (temp_road->push_back(temp_car, temp_car->nextSpeed) == Container::SUCCESS) //success then delete it from garage
                        {
                            drive_count++;
                            start_car_count++;
                            temp_car->startTime = global_time;
                            i = garage.erase(i);
                            continue;
                        }
                    }
                    ++i;
                    continue;
                }

                Container* temp_road = searchRoad(temp_car);
                if (temp_road) // find road
                {
                    temp_car->nextSpeed = min(temp_car->speed, temp_road->maxSpeed);
                    if(temp_road->push_back(temp_car, temp_car->nextSpeed)==Container::SUCCESS)//success then delete it from garage
                    {
                        drive_count++;
                        start_car_count++;
                        temp_car->startTime=global_time;
                        temp_car->route.push_back(temp_road);
                        i = garage.erase(i);
                        continue;
                    }
                }
            }
            ++i;
        }
    }
    else
    {
        for (deque<CAR*>::iterator i= garage.begin(); i!=garage.end();)
        {
            if(drive_count>batch_size)return;

            CAR *temp_car = *i;

            if(temp_car->planTime <= global_time)
            {
                if (temp_car->isPreset)
                {
                    Container *temp_road = temp_car->route.back();
                    if (temp_road->size() <= max_factor_drive_car * temp_road->capacity)
                    {
                        if (temp_road->push_back(temp_car, temp_car->nextSpeed) == Container::SUCCESS) //success then delete it from garage
                        {
                            drive_count++;
                            start_car_count++;
                            temp_car->startTime = global_time;
                            i = garage.erase(i);
                            continue;
                        }
                    }
                    ++i;
                    continue;
                }

                Container* temp_road = searchRoad(temp_car);
                if(temp_road)
                {
                    temp_car->nextSpeed = min(temp_car->speed, temp_road->maxSpeed);
                    if(temp_road->push_back(temp_car, temp_car->nextSpeed)==Container::SUCCESS)//success then delete it from garage
                    {
                        drive_count++;
                        start_car_count++;
                        temp_car->startTime=global_time;
                        temp_car->route.push_back(temp_road);
                        i = garage.erase(i); continue;
                    }
                }
            }
            ++i;
        }
    }
}

void CROSS::dispatch(int global_time)
{
    // 道路按ID升序遍历
    for(auto current_road : enterRoadVec)
    {
        CAR *pCar = current_road->top();
        while(pCar)
        {
            // 如果到达目的地
            if (current_road->nextCrossId == pCar->to)
            {
                totalCarCount--;
                if(!pCar->isPreset){
                    fout << '(' << pCar->id << ", " << pCar->startTime;
                    for(auto road : pCar->route) fout << ", " << road->roadId;
                    #if __DEBUG_MODE__
                        fout << ")" << endl;
                    #else 
                        fout << ")\n";
                    #endif
                }
                waitStateCarCount--;
                current_road->pop();
                current_road->dispatchCarInChannel(pCar->currentChannel);      //调度该车之前所在车道
                current_road->startCross->driveCarInitList(true, global_time); //执行一次优先车辆上路
                delete pCar;
                pCar = current_road->top();
                continue;
            }

            int turn_weight = pCar->turnWeight; 
            Container *temp_road;
            CAR *temp_car;

          /* 判断是否冲突 */
            bool conflict = false;
            if(pCar->isPrior) {
                switch(turn_weight)
                {
                    case 0: //右转
                    {
                        // 首先看左边道路有没有直行的优先车辆
                        temp_road = current_road->opposite[1];
                        if(temp_road){
                            temp_car = temp_road->top();
                            if (temp_car && temp_car->isPrior && temp_car->turnWeight == 2) {
                                conflict = true;
                                break;
                            }
                        }
                        // 再看前方道路有没有左转的优先车辆
                        temp_road = current_road->opposite[2];
                        if(temp_road){
                            temp_car = temp_road->top();
                            if (temp_car && temp_car->isPrior && temp_car->turnWeight == 1)
                                conflict = true;
                        }
                    } break;

                    case 1: // 左转
                    {
                        // 看右方道路有没有直行的优先车辆
                        temp_road = current_road->opposite[0];
                        if(temp_road) {
                            temp_car = temp_road->top();
                            if(temp_car && temp_car->isPrior && temp_car->turnWeight == 2){
                                conflict = true;
                            }
                        }
                    } break;
                }
            } else {
                switch(turn_weight)
                {
                    case 0: //右转
                    {
                        // 首先看左边道路有没有直行车辆
                        temp_road = current_road->opposite[1];
                        if(temp_road){
                            temp_car = temp_road->top();
                            if (temp_car && temp_car->turnWeight == 2) {
                                conflict = true;
                                break;
                            }
                        }

                        // 再看前方道路有没有左转车辆
                        temp_road = current_road->opposite[2];
                        if(temp_road){
                            temp_car = temp_road->top();
                            if (temp_car && temp_car->turnWeight == 1)
                                conflict = true;
                        }
                    } break;

                    case 1: // 左转
                    {
                        // 看右方道路有没有车执行
                        temp_road = current_road->opposite[0];
                        if(temp_road) {
                            temp_car = temp_road->top();
                            if(temp_car && temp_car->turnWeight == 2){
                                conflict = true;
                            }
                        }
                    } break;
                }
            }
            if(conflict) break;
          /* End of 判断是否冲突 */

            int s2 = pCar->nextSpeed - pCar->currentIdx;
            if (s2 <= 0)
            {
                current_road->updateWhenNextRoadFull(pCar->currentChannel);
                current_road->startCross->driveCarInitList(true, global_time);
                pCar = current_road->top();
                continue;
            }

            switch (current_road->turnTo[turn_weight]->push_back(pCar, s2))
            {
                // 成功进入下一个道路
                case Container::SUCCESS:{
                    waitStateCarCount--; //成功进入，该车在 push_back() 函数内置为 END，所以计数减一
                    current_road->pop(); //已经过路口，从优先队列弹出
                    current_road->dispatchCarInChannel(pCar->preChannel); //调度该车之前所在车道
                    current_road->startCross->driveCarInitList(true, global_time); //执行一次优先车辆上路
                    pCar = current_road->top(); 
                } break;

                // 下一个道路满载
                case Container::FULL_LOAD:{
                    current_road->updateWhenNextRoadFull(pCar->currentChannel);
                    current_road->startCross->driveCarInitList(true, global_time);
                    pCar = current_road->top();
                } break;

                // 因被等待车辆阻挡而无法进入下一道路
                default:{
                    pCar = nullptr;
                } break;
            }
        }
    }
}
/*************************** end of class CROSS *****************************/



/*************************** class GRAPH *****************************/
vector<Container *> GRAPH::containerVec;
unordered_map<uint8_t, float *> GRAPH::costMap;
vector<uint8_t> GRAPH::speedDetectResultVec;
void GRAPH::calculateCostMap()
{
    for(int i=0; i<SPEED_DETECT_ARRAY_LENGTH; i++){
        if(speedDetectArray[i]){
            float *pCost = new float [Container::containerCount];
            float *temp = pCost;
            for(auto val : containerVec){
                *temp = float(val->length)/(min(val->maxSpeed, i));
                temp++;
            }
            costMap.insert(pair<uint8_t, float *>(i, pCost));
            speedDetectResultVec.push_back(i);
        }
    }
}
/*************************** end of class GRAPH *****************************/