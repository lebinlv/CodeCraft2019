#ifndef _FUNCTION_H_
#define _FUNCTION_H_

#include "object.hpp"

//释放在行驶车辆占用的容量以及删除到达车辆
void release_capacity(std::list<CAR *> &car_running, int global_time);

//输出answer文件
void write_to_file(std::vector<GRAPH::Node *> *tem_vec, CAR *car, std::ofstream &fout);

// WARM_UP 调度器
class WARM_UPer
{
  private:
    struct Recorder{
        int start;
        int p_start;
        int remain;

        Recorder(int _start, int _remain):start(_start),p_start(_start),remain(_remain){}
        Recorder() = delete;
        Recorder(const Recorder &) = delete;
        Recorder(Recorder &&) = delete;
    };
    
    int _end_plan_time;      // WARM_UP 阶段只会发出计划时间小于 end_plan_time 的车辆
    
    std::vector< Recorder* > record_vec; // 记录已经安排出去的车辆

    int get_car(int size, int t, std::vector<std::pair<int,int> > *answer);

  public:
    /**
     * @brief Construct a new WARM_UPer object
     * 
     * @param plan_time_record 记录了 0～end_plan_time 时间段内各时刻计划出发的车辆的数组的地址
     * @param end_plan_time    WARM_UP 阶段只会发出计划时间小于 end_plan_time 的车辆
     */
    WARM_UPer(int plan_time_record[], int end_plan_time);
    WARM_UPer() = delete;
    WARM_UPer(const WARM_UPer &) = delete;
    WARM_UPer(WARM_UPer &&) = delete;
    ~WARM_UPer(){}

    /**
     * @brief Get the car in warm up object
     * 
     * @param car_vec            保存了所有车辆地址的vector（需按时间由小到大排序）
     * @param dispatch_size      此次安排的车辆数目
     * @param global_time        全局时间
     * @return std::vector<std::pair<int, int>>*  保存所安排的车辆在car_vec中的位置，需要手动释放该指针
     *                                     车辆下标区间左闭右开[ pair<int, int>.first ～ pair<int, int>.second)
     */
    std::vector<std::pair<int, int>> *get_car_in_warm_up(int dispatch_size, int global_time);

    /**
     * @brief 结束热启动，将已安排出去的车辆从 car_vec 中删除
     *
     * @param car_vec    保存了所有车辆地址的vector（需按时间由小到大排序）
     */
    void warm_up_end(std::vector<CAR*> & car_vec);
};

#endif