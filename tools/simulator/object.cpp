#include <cfloat>

#include "lib/object.hpp"

using namespace std;

extern int waitStateCarCount;
extern int totalCarCount;
extern int totalCarRunTime;
extern int totalPriParRunTime;
extern int lastPriCarEndTime;
extern map_type<int, ROAD*> roadMap;
extern map_type<int, CROSS *> crossMap;
extern vector<CROSS *> crossVec;

inline void CAR::enterNewRoad(int newIdx, int newChannel)
{
    currentSpeed = nextSpeed;
    currentIdx = newIdx;
    preChannel = currentChannel;
    currentChannel = newChannel;
    state = END;
    getNewRoad = false;

    route.pop();
}

/*************************** class Container *****************************/
Container::Container(int _roadId, int _channel, int _length, int _maxSpeed, int _crossId):
                     roadId(_roadId), channel(_channel), length(_length), maxSpeed(_maxSpeed),
                     nextCrossId(_crossId), capacity(_channel * _length), 
                     startCross(nullptr), endCross(nullptr)
{
    carInChannel = new container_t [channel];
    for(int i=0; i<channel; ++i) {
        carInChannel[i].reserve(length);
    }
}

int Container::push_back(CAR *pCar, int s2)
{
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
    channelVec.erase(channelVec.begin());
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
        getNextRoad(temp_car); //为车辆分配路线
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

    if(new_idx < 0){  // 若 new_idx<0，则该车有可能出路口
        if(temp_car->getNewRoad) { // 若已经为该车分配过道路，（这种情况下该车的 currentIdx 必然为0）
            temp_car->state = CAR::WAIT;
            waitStateCarCount++;
            priCar.push(temp_car); // 该车能出路口，所以将其放入出路口优先队列
        }
        else // 若没有为该车分配过道路，则调用路径规划函数为其分配道路
        { 
            getNextRoad(temp_car); //为车辆分配路线
            pre_car_idx = temp_car->currentIdx; // 不移动该车
            temp_car->state = CAR::WAIT;        // 并将该车标记为WAIT
            waitStateCarCount++;
            priCar.push(temp_car); // 该车能出路口，所以将其放入出路口优先队列
        }
    } else { // 否则（即new_idx>=0） 该车无法出路口
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

void Container::getNextRoad(CAR *pCar)
{
    if(nextCrossId == pCar->to)
    {
        pCar->preChannel = pCar->currentChannel;
        pCar->nextSpeed = pCar->currentSpeed;
        pCar->turnWeight = 2;
        pCar->getNewRoad = true;
        return;
    }

    Container *next_road = pCar->route.front();
    pCar->nextSpeed = min(pCar->speed, next_road->maxSpeed);
    pCar->turnWeight = CROSS::getTurnDirection(this->roadId, next_road->roadId);
    pCar->getNewRoad = true;
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
}


void CROSS::driveCarInitList(bool is_prior,int global_time)
{
    if (is_prior) //如果is_prior是true，则只上路优先车辆
    {
        for (deque<CAR*>::iterator i =garage.begin();i!=garage.end();)
        {
            CAR *temp_car = *i;

            // 如果计划时间不晚于当前时间 且 是优先车辆
            if ((temp_car->answerTime) <= global_time && (temp_car->isPrior)) 
            {
                Container *temp_road = temp_car->route.front();
                temp_car->nextSpeed = min(temp_car->speed, temp_road->maxSpeed);

                if(temp_road->push_back(temp_car, temp_car->nextSpeed)==Container::SUCCESS) 
                {
                    temp_car->startTime = global_time;
                    i = garage.erase(i); continue;
                }
            }
            ++i;
        }
    }
    else
    {
        for (deque<CAR*>::iterator i=garage.begin(); i!=garage.end();)
        {
            CAR *temp_car = *i;
            int speed = temp_car->speed;

            if(temp_car->answerTime <= global_time)
            {
                Container *temp_road = temp_car->route.front();
                temp_car->nextSpeed = min(speed, temp_road->maxSpeed);
                if(temp_road->push_back(temp_car, temp_car->nextSpeed)==Container::SUCCESS) 
                { 
                    temp_car->startTime = global_time;
                    i = garage.erase(i); continue;
                }
            }
            ++i;
        }
    }
}

void CROSS::driveAllCarInGarage(int global_time)
{

    for (deque<CAR*>::iterator i =priorGarage.begin();i!=priorGarage.end();)
    {
        CAR *temp_car = *i;

        // 如果计划时间不晚于当前时间
        if ((temp_car->answerTime) <= global_time) 
        {
            Container *temp_road = temp_car->route.front();
            temp_car->nextSpeed = min(temp_car->speed, temp_road->maxSpeed);

            if(temp_road->push_back(temp_car, temp_car->nextSpeed)==Container::SUCCESS) 
            {
                temp_car->startTime = global_time;
                i = priorGarage.erase(i); continue;
            }
        }
        ++i;
    }


    for (deque<CAR*>::iterator i=ordinaryGarage.begin(); i!=ordinaryGarage.end();)
    {
        CAR *temp_car = *i;
        int speed = temp_car->speed;

        if(temp_car->answerTime <= global_time)
        {
            Container *temp_road = temp_car->route.front();
            temp_car->nextSpeed = min(speed, temp_road->maxSpeed);
            if(temp_road->push_back(temp_car, temp_car->nextSpeed)==Container::SUCCESS) 
            { 
                temp_car->startTime = global_time;
                i = ordinaryGarage.erase(i); continue;
            }
        }
        ++i;
    }
}

void CROSS::drivePriorCarInGarage(int global_time, Container *road)
{
    if(road)
    {
        for (deque<CAR *>::iterator i = priorGarage.begin(); i != priorGarage.end();)
        {
            CAR *temp_car = *i;
            Container *temp_road = temp_car->route.front();

            // 如果计划时间不晚于当前时间 且 是优先车辆
            if ((temp_car->answerTime) <= global_time && temp_road==road)
            {
                temp_car->nextSpeed = min(temp_car->speed, temp_road->maxSpeed);

                if (temp_road->push_back(temp_car, temp_car->nextSpeed) == Container::SUCCESS)
                {
                    temp_car->startTime = global_time;
                    i = priorGarage.erase(i);
                    continue;
                }
            }
            ++i;
        }
    }
    else
    {
        for (deque<CAR *>::iterator i = priorGarage.begin(); i != priorGarage.end();)
        {
            CAR *temp_car = *i;

            // 如果计划时间不晚于当前时间
            if ((temp_car->answerTime) <= global_time)
            {
                Container *temp_road = temp_car->route.front();
                temp_car->nextSpeed = min(temp_car->speed, temp_road->maxSpeed);

                if (temp_road->push_back(temp_car, temp_car->nextSpeed) == Container::SUCCESS)
                {
                    temp_car->startTime = global_time;
                    i = priorGarage.erase(i);
                    continue;
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
                waitStateCarCount--;
                totalCarRunTime += (global_time-pCar->planTime);
                if(pCar->isPrior) {totalPriParRunTime += (global_time-pCar->planTime); lastPriCarEndTime = global_time;}

                current_road->pop();
                current_road->dispatchCarInChannel(pCar->currentChannel);      //调度该车之前所在车道
                current_road->startCross->drivePriorCarInGarage(global_time, nullptr); //执行一次优先车辆上路

                delete pCar;
                pCar = current_road->top();
                continue;
            }

            int turn_weight = pCar->turnWeight;
            Container *temp_road;
            CAR *temp_car;

            /* 判断是否冲突 */
            bool conflict = false;
            if(pCar->isPrior){
                switch(turn_weight)
                {
                    case 0: //右转
                    {
                        // 首先看左边道路有没有直行车辆
                        temp_road = current_road->opposite[1];
                        if(temp_road){
                            temp_car = temp_road->top();
                            if (temp_car && temp_car->isPrior && temp_car->turnWeight == 2) {
                                conflict = true;
                                break;
                            }
                        }

                        // 再看前方道路有没有左转车辆
                        temp_road = current_road->opposite[2];
                        if(temp_road){
                            temp_car = temp_road->top();
                            if (temp_car && temp_car->isPrior && temp_car->turnWeight == 1)
                                conflict = true;
                        }
                    } break;

                    case 1: // 左转
                    {
                        // 看右方道路有没有车直行
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
                        // 看右方道路有没有车直行
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

            int s2 = pCar->nextSpeed - pCar->currentIdx;
            if(s2 <= 0){
                current_road->updateWhenNextRoadFull(pCar->currentChannel);
                current_road->startCross->drivePriorCarInGarage(global_time, current_road);
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
                    current_road->startCross->drivePriorCarInGarage(global_time, current_road); //执行一次优先车辆上路
                    pCar = current_road->top(); 
                } break;

                // 下一个道路满载
                case Container::FULL_LOAD:{
                    current_road->updateWhenNextRoadFull(pCar->currentChannel);
                    current_road->startCross->drivePriorCarInGarage(global_time, current_road);
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
