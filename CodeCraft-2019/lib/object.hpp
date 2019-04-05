#ifndef _OBJECT_H_
#define _OBJECT_H_

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


struct TURN
{
    //右转：0   左转：1   直行：2
    enum direction{RIGHT, LEFT, STRAIGHT};
};


struct CAR;
struct CROSS;

// 单侧道路
class Container
{
  public:
    typedef std::vector<CAR *> container_t;

    // push_back 函数的返回值， FULL_LOAD表示无剩余空间，SUCCESS表示成功进入
    enum PUSH_BACK_STATE{FULL_LOAD=-2, SUCCESS};

    // 道路id， 车道数， 道路长度
    int id, channel, length;

    // turn_to[0], [1], [2] 分别指向 从当前道路右转、左转、直行后 到达的Container
    Container *turn_to[3] = {nullptr, nullptr, nullptr};

    // opposite[0], [1], [2] 分别指向 当前道路的右边、左边、前方道路中 与当前逆向的Container
    Container *opposite[3] = {nullptr, nullptr, nullptr};

    // 指向该道路的起点路口、终点路口的指针
    CROSS *from, *to;

  private:
    container_t *carInChannel; // 构造函数中使用 new [] 生成
    std::priority_queue<CAR *> priCar; // 出路口的车的优先队列

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
    Container(int _id, int _channel, int _length);
    ~Container() { delete [] carInChannel; }

    Container() = delete;
    Container(const Container &) = delete;
    Container(Container &&) = delete;

    // 获取第 pos 车道的 车辆vector 的引用, `pos~[0, channel]`
    inline container_t & getCarVec(int pos){return carInChannel[pos];}
    inline container_t &operator[](int pos) { return carInChannel[pos]; }

    /**
     * @brief 加入新的车辆
     * 
     * @param pCar 要加入的车辆指针
     * @return `Container::FULL_LOAD`: 道路满载，无法放入新的车辆；
     *         `Container::SUCCESS`: 成功加入；
     *         `others(>=0)`: 因等待车辆而无法进入，返回道路入口与等待车辆的距离
     */
    int push_back(CAR* pCar);

    /**
     * @brief 获取最先行驶的车辆的指针
     * 
     * @return 如果所有车辆都不会出路口，返回 nullptr
     */
    inline CAR *top() { return priCar.empty() ? nullptr:priCar.top(); }

    /**
     * @brief 取出第一辆车
     * 
     * @return true  成功取出
     * @return false 取出失败
     */
    bool pop();

    /**
     * @brief 判断是否有车要出路口
     * 
     * @return true  没有车要出路口
     * @return false 有车要出路口
     */
    inline bool empty(){ return priCar.empty();}

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
        for(int i=0; i<length; i++) dispatchCarInChannelFirst(i);
    }
};

class ROAD
{
  public:
    int id, length, maxSpeed, channel, from, to;
    int capacity;
    bool isDuplex;

  private:
    // 车辆容器，如果为单向道路，则 backward 为空指针，构造函数中使用new生成
    Container *forward, *backward;

  public:
    ROAD(int _id, int _length, int _speed, int _channel, int _from, int _to, bool _isDuplex);
    ~ROAD() { delete forward, backward; }

    ROAD() = delete;
    ROAD(const ROAD &) = delete;
    ROAD(ROAD &&) = delete;

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
    inline void dispatchCarOnRoad(){ 
        forward->dispatchCarInContainer();
        if(isDuplex) backward->dispatchCarInContainer();
    };
};


struct CROSS
{
    int id;

    // 进入该路口的Container指针，按id升序排列
    std::vector<Container *>  enterRoadVec;
    // 离开该路口的Container指针, 无序排列
    std::vector<Container *>  awayRoadVec;

    // 车库，只用于发车...
    std::vector<CAR *> garage;

    // `turnMap[(id1<<16)|id2]` 表示 从 道路id1 转向 道路id2 的转向优先级；右转:0，左转:1，直行:2
    static map_type<int, uint8_t> turnMap;

    /**
     * @brief 根据路口id和与其相连的道路构建cross
     * 
     * @param crossId 路口id
     * @param roadId  与路口相连的道路id数组，存储顺序为 up, right, down, left
     */
    CROSS(int crossId, int roadId[]);

    CROSS() = delete;
    CROSS(const CROSS &) = delete;
    CROSS(CROSS &&) = delete;
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

    static map_type<int, void *> routeMap; //TODO: routeMap

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
    // 车辆的运行状态
    enum CAR_STATE {WAIT, RUNNING, END};

    int id, from, to, speed, planTime;
    bool isPrior, isPreset;

    // 该车当前所在车道， 该车在上一道路中的车道;
    // 车在当前道路上的位置, `idx ~ [0, length-1]`, 越小表示离出口越近;
    // 车在当前道路上的速度;
    int currentChannel, preChannel, currentIdx, currentSpeed;

    // TODO: 路径规划函数在为车规划下一条道路时，应计算出车在道路上的速度nextSpeed,
    //       并将 `getNewRoad` 置为 `True`
    int nextSpeed;
    bool getNewRoad;

    // 车的运行状态
    CAR_STATE state;

    // 车的实际出发时间
    int startTime;

    // 记录该车辆的行驶路线
    std::vector<ROAD *> route;


    CAR(int _id, int _from, int _to, int _speed, int _time, bool _isPrior, bool _isPreset):
        id(_id), from(_from), to(_to), speed(_speed), planTime(_time),
        isPrior(_isPrior), isPreset(_isPreset) {}
    ~CAR() {}

    CAR(const CAR &) = delete;
    CAR(CAR &&) = delete;
    CAR() = delete;

    /**
     * @brief 当车辆进入下一条道路时，可使用本函数更新车辆信息
     * 
     * @param newSpeed  车辆在下一条道路上的行驶速度；
     * @param newIdx    车辆在下一条道路上的初始位置
     * @param newChannel 车辆在下一条道路上的车道；
     */
    inline void enterNewRoad(int newIdx, int newChannel) {
        speed = nextSpeed;
        currentIdx = newIdx;
        state = END;
        getNewRoad = false;
        preChannel = currentChannel;
        currentChannel = newChannel;
    }

    /** 
     * @brief 自定义出路口优先队列的比较方法。
     * 
     * 如果两辆车的计划时间相等，则先速度快的车优先级高；
     * 否则先出发的车优先级高。
     */
    struct Compare {
        bool operator()(const CAR* a, const CAR* b) {
          if (a->isPrior == b->isPrior) //如果优先级相同
            // 如果位置相同，则车道小的优先，否则位置小的优先
            return (a->currentIdx == b->currentIdx) ? a->currentChannel > b->currentChannel : a->currentIdx > b->currentIdx;
          else // 如果优先级不同，则优先级高的优先
            return a->isPrior < b->isPrior;
        }
        bool operator()(const CAR & a, const CAR & b) {
            return (a.planTime == b.planTime) ?
                   (a.speed < b.speed) : (a.planTime > b.planTime);
        }
    };
};

#endif