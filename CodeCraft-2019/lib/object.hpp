#ifndef _OBJECT_H_
#define _OBJECT_H_

#include <cstdint>
#include <vector>      // 寻路函数返回vector
#include <map>         // 权重字典
#include <queue>
#include <algorithm>
#include <list>
#include <fstream>
#include <stack>
#include <unordered_map>

static float CAPACITY_FACTOR = 0;   //容量因子
static float ROAD_VALID_THRESHOLD = 0;  //当边的容量小于此值时认为该边无效

struct CAR;

// 
class Container
{
  public:
    typedef std::vector<CAR *> container_t;

    // 根据车道数和道路长度初始化容器
    Container(int channel, int length);

    Container() = delete;
    Container(const Container &) = delete;
    Container(Container &&) = delete;

  private:
    int channel;//后面优先队列需要，车道数量
    int length;//道路长度

    container_t  *carInChannel;
    std::priority_queue<CAR*, vector<CAR*>, CAR::Compare> priCar; // 出路口的车的优先队列

  public:
    // 获取第 pos 车道的 车辆vector 的引用
    inline container_t & operator[](int channel){return carInChannel[channel];}
    inline container_t & getCarVec(int channel){return carInChannel[channel];}

    /**
     * @brief 加入新的车辆
     * 
     * @param pCar 要加入的车辆指针
     *         length 上个道路的长度
     * @return true 成功加入
     * @return false 加入失败
     * @notice 这边假设传入的pcar 保留了原来道路上的idx，他的next_road就是当前道路
     * pCar如果返回成功需要把该车从原来的vector中删除，也就是说，这个函数加入成功后不会自行把这个车从
     * 原来的vector中删除。
     * 调度完成后记得运行update_prior_queue
     */
    bool push_back(CAR* pCar);

    /**
     * @brief 获取最先行驶的车辆的指针
     * 
     * @return 如果所有车辆都不会出路口，返回 nullptr
     */
    inline CAR *top();

    /**
     * @brief 取出第一辆车
     * 
     * @return true  成功取出
     * @return false 取出失败
     */
    inline bool pop();

    /**
     * @brief 更新优先队列，可以保证队列里面的车辆都是wait状态，
     * 但只是单纯的优先顺序，不保证该辆车一定是可以开出去的，
     * 也就是说，top得到的第一辆车
     * 可能会因为他要驶入的道路中存在wait车辆的影响，而无法行驶.
     * @notice 道路变化之后，也就是调度完成后需要运行该函数，保证优先队列的正确性
     */
    void update_prior_queue(int);

};

class ROAD
{
  public:
    int id, length, max_speed, channel, from, to;
    int capacity;
    bool isDuplex;

  private:
    // 车辆容器
    Container *forward, *backward;

  public:
    ROAD(int _id, int _length, int _speed, int _channel, int _from, int _to, bool _isDuplex);
    ROAD() = delete;
    ROAD(const ROAD &) = delete;
    ROAD(ROAD &&) = delete;
    ~ROAD(){ delete [] forward, backward;}

    /**
     * @brief 获取该道路上进入和离开指定路口的车辆的容器
     * 
     * @param cross_id 路口id 
     * @return std::pair<container ,container>  .first: 进入本路口的车辆容器； .second: 离开本路口的车辆容器
     */
    inline std::pair<Container* ,Container*> getContainer(int cross_id) {
        return cross_id == to ? std::pair<Container*, Container*>(forward, backward) : std::pair<Container*, Container*>(backward, forward);
    }

    /**
     * @brief 调度该道路上的车辆
     * 
     */
    void moveOnRoad();

    /**
     * @brief 移动该道路上指定车道内的车辆
     * 
     * @param channel 车道id（从0开始）
     */
    void moveInChannel(Container::container_t *container, int channel);


    /**
     * @brief 移动该道路上指定车道内的车辆
     * 
     * @param 车道的vec，主要是为了moveOnRoad函数服务
     * 
     * @notice 
     * 该函数内部需要得到下一条道路，目前假设该道路会赋值给car的next_road指针。
     * 需要寻路函数完成后再考虑。
     */
    static void dispatch_one_channel(std::vector<CAR*> & carVec);
    
};


class CROSS
{
  public:
    int id;




};


class GRAPH
{
  public:
    struct Node{
        int                           cross_id;   // 本节点代表的路口的id
        double                        capacity;   // 到达本节点的边的容量
        int                           info_idx;   // 权重、长度、限速等信息均通过info_idx访问
        ROAD *                        pRoad;      // 通过 pRoad 获取道路长度、最大速度、车道数、是否双向等信息

        static int                    node_count; // 静态变量用于统计节点个数，初始值为0
        static std::vector<ROAD *>    pRoad_vec;  // 通过pRoad_vec[info_idx] 可读取 length, channel, max_speed, from, to 等信息

        Node(int _cross_id, ROAD *_pRoad);
        Node() = delete;
        Node(const Node &) = delete;
        Node(Node &&) = delete;
        ~Node(){}
    };

    typedef std::vector<Node *> route_type; // 寻最短路函数的返回数据类型

    GRAPH(int reserve_node_count);
    GRAPH() = delete;
    GRAPH(const GRAPH &) = delete;
    GRAPH(GRAPH &&) = delete;
    ~GRAPH();

    /**
     * @brief 根据道路信息添加节点
     * 
     * @param pRoad 指向道路的指针
     * @param cross_id 这条边的终点
     */
    void add_node(ROAD* pRoad);

    /**
     * @brief 根据车速计算新的权重数组
     * 
     * @param speed 车速
     * @attention   必须在添加所有节点之后调用该函数
     */
    void add_weights(int speed);
    void add_weights(bool speed_detect_array[], int size);

    /**
     * @brief Get the least cost route object
     * 
     * @param from 
     * @param to 
     * @param speed 
     * @return route_type& 返回 vector<Node *>，如果失败则vector为空，如果成功，逆序遍历此vector便可得到路径
     */
    route_type * get_least_cost_route(CAR* car, int global_time);
    route_type * get_least_cost_route(int from, int to, int speed);


  private:
    std::unordered_map<int, double*>               weight_map;  // 边对于不同速度的车，具有不同的权重
    std::unordered_map<int, std::vector<Node*> >   graph_map;   //

    double*                              p_weight;    // 每次计算最短路径之前，根据车速重定向该指针

    struct __Node{
        double       cost;
        int          cross_id;

        __Node *     parent;   // 用于到达终点时回溯得到路径
        Node *       p_Node;

        __Node(double _cost, int _cross_id, __Node* _parent, Node* _p_Node):
               cost(_cost), cross_id(_cross_id), parent(_parent), p_Node(_p_Node){}
        __Node() = delete;
        __Node(const __Node &) = delete;
        __Node(__Node &&) = delete;
        ~__Node(){}

        struct Compare{
            bool operator()(const __Node* a, const __Node* b){
                return a->cost > b->cost;
            }
        };
    };
};


struct CAR
{
    // (id, from, to, speed, planTime)
    int id, from, to, speed, plan_time;
    int   start_time;
    float capacity_factor;
    struct Past_node {
        GRAPH::Node* node;
        double arrive_time;
        Past_node() {}
        Past_node(GRAPH::Node* _node, double _arrive_time) :node(_node), arrive_time(_arrive_time) {}
    };
    std::stack<Past_node*>      past_nodes;

    // 车辆的运行状态
    enum CAR_STATE {WAIT, RUNNING, END};
    int channel_idx;//所处在的车道序号，目前只是服务pop,也就是说只要初始化为0就好
    int idx;        // 车在道路上的位置
    int v;  // 车在道路上的可行速度
    bool prior;  // 是否优先
    bool preset; // 
    ROAD *next_road;

    CAR_STATE state;

    CAR(int _id, int _from, int _to, int _speed, int _time):
        id(_id), from(_from), to(_to), speed(_speed), plan_time(_time) {}
    CAR(const CAR &) = delete;
    CAR(CAR &&) = delete;
    CAR() = delete;
    ~CAR() {}

    /** 
     * @brief 自定义优先队列的比较方法
     * 
     * 如果两辆车的计划时间相等，则先速度快的车优先级高；
     * 否则先出发的车优先级高。
     */
    struct Compare {
        bool operator()(const CAR* a, const CAR* b) {
          if(a->prior == b->prior)
            return (a->idx==b->idx)?a->channel_idx>b->channel_idx:a->idx>b->idx;
          else return a->prior < b->prior;
        }
        bool operator()(const CAR & a, const CAR & b) {
            return (a.plan_time == b.plan_time) ?
                   (a.speed < b.speed) : (a.plan_time > b.plan_time);
        }
    };
};
void get_factor(CAR* car,int time);
#endif