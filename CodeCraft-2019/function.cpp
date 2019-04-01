#include "lib/function.hpp"
#include <iostream>
using namespace std;

//容量释放与相关车辆删除
void release_capacity(std::list<CAR*>& car_running, int global_time)
{
    CAR::Past_node* node;
    for (auto car = car_running.begin(); car!=car_running.end();) {
	node = (*car)->past_nodes.top();
        if (node->arrive_time < global_time){
            //已经到达这个节点，恢复容量并且将它从路径stack中删
            node->node->capacity += (*car)->capacity_factor;
            (*car)->past_nodes.pop();
            delete node;
        }
        
        if ((*car)->past_nodes.empty()){
            car_running.erase(car++);
        }else {
            car++;
        }
    }
}


// 答案写入接口
void write_to_file(std::vector<GRAPH::Node*> * tem_vec, CAR * car, std::ofstream &fout)
{
    fout << '(' << car->id << ", " << car->start_time;
    for_each(tem_vec->rbegin(), tem_vec->rend(), [&fout](GRAPH::Node *val) -> void { fout << ", " << val->pRoad->id; });
    fout << ")\n";
}




WARM_UPer::WARM_UPer(int plan_time_record[], int end_plan_time) : _end_plan_time(end_plan_time)
{
    int count = 0;
    record_vec.reserve(end_plan_time);
    for(int i=0; i<end_plan_time; i++){
        record_vec.push_back(new Recorder(count, plan_time_record[i]));
        count += plan_time_record[i];
    }
}

int WARM_UPer::get_car(int size, int t, vector<pair<int, int> > *answer)
{
    auto pRecorder = record_vec[t];
    auto remain = pRecorder->remain;
    // 如果当前时刻的可出发车辆数目不小于 dispatch_size
    if(remain >= size){
        answer->push_back(pair<int, int>(pRecorder->p_start, pRecorder->p_start + size));
        pRecorder->p_start += size;
        pRecorder->remain -= size;
        return size;
    } else if(remain > 0) {
        answer->push_back(pair<int, int>(pRecorder->p_start, pRecorder->p_start + remain));
        pRecorder->p_start += remain;
        pRecorder->remain = 0;
        return remain;
    } else return 0;
}

vector< pair<int, int> > * WARM_UPer::get_car_in_warm_up(int dispatch_size, int global_time)
{
    auto answer = new vector< pair<int, int> >();
    //get_car(dispatch_size, global_time, answer);
    get_car(dispatch_size, global_time, answer);
    return answer;
}

void WARM_UPer::warm_up_end(std::vector<CAR *> & car_vec)
{
    // erase car_vec
    //int temp = car_vec.size();
    //for_each(record_vec.rbegin(), record_vec.rend(),
    //        [&car_vec](Recorder * val)->void{car_vec.erase(car_vec.begin()+val->start, car_vec.begin()+val->p_start);});
    //temp = car_vec.size();
    // 重新排序
    stable_sort(car_vec.begin(), car_vec.end(), [](CAR *a, CAR *b)->bool{return a->speed < b->speed;});
}
void get_factor(CAR* car,int time){
    if(time<175)
    {
        car->capacity_factor=0.48;
    }
    else if(time<325)
    {
        car->capacity_factor=0.48;
    }
    else
    {
        car->capacity_factor=0.45;
    }
}