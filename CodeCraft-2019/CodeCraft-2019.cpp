#include <iostream>
#include <cstdio>
#include <vector>
#include <string>
#include <queue>
#include <list>
#include <algorithm>

#include <chrono>   // speed test

#include "object.hpp"

#define MAXIMUM_LENGTH_PER_LINE    50      // 每行字符个数不超过 MAXIMUM_LENGTH_PER_LINE
#define CAR_VECTOR_RESERVE_SIZE    11000   // 
#define ROAD_VECTOR_RESERVE_SIZE   150 
#define USE_CXX_SSTREAM 0


using namespace std;

static int global_time = 0;//q全局时间

int main(int argc, char *argv[])
{
	/* SDK code */
	cout << "Begin" << endl;

	if (argc < 5) {
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
	//priority_queue<CAR *, vector<CAR *>, CAR::Compare> car_pri_queue; // 建立 priority queue 将所有车辆信息保存
	vector<CAR*> car_vec;
	car_vec.reserve(CAR_VECTOR_RESERVE_SIZE);

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
		auto pCar = new CAR;
		sscanf(line_buffer, "(%d, %d, %d, %d, %d)",
			&(pCar->id), &(pCar->from), &(pCar->to), &(pCar->speed), &(pCar->plan_time));
		//car_pri_queue.push(pCar);
		car_vec.push_back(pCar);
		//}
	}
	fclose(fptr);
	sort(car_vec.begin(), car_vec.end(),
		[](CAR* a, CAR* b) -> bool { return
		(a->plan_time == b->plan_time) ? (a->speed > b->speed) : (a->plan_time < b->plan_time); });

	// auto end = std::chrono::steady_clock::now();
	// std::chrono::duration<double, std::micro> elapsed = end - start; // std::micro 表示以微秒为时间单位
	// std::cout << "time: " << elapsed.count() << "us" << std::endl;

	/* End of read car configuration */



	/* Read cross configuration, mainly to get the number of cross */
	// fptr = fopen(crossPath.c_str(), "r");
	// if (fptr == NULL) {
	//     cout << "can't open cross configuration file: " << roadPath << endl;
	//     exit(-1);
	// }
	// int cross_count = 0;
	// /* 逐行读取信息 */
	// fgets(line_buffer, 50, fptr); // 似乎只有文件第一行为注释......
	// while (fgets(line_buffer, 50, fptr)) {
	//     //if(line_buffer[0] == '(') {
	//         cross_count ++;
	//     //}
	// }
	// fclose(fptr);
	/* End of Read cross configuration */



	/* Read road configuration, and build GRAPH at the same time */
	//vector<ROAD*> road_vec;
	//road_vec.reserve(ROAD_VECTOR_RESERVE_SIZE);
	map<ROAD::id_type, ROAD*> road_map;

	GRAPH graph;

	int test_road_id, length, max_speed, channel, from, to, isDuplex;

	fptr = fopen(roadPath.c_str(), "r");
	if (fptr == NULL) {
		cout << "can't open road configuration file: " << roadPath << endl;
		exit(-1);
	}

	/* 逐行读取道路信息 */
	fgets(line_buffer, 50, fptr); // 似乎只有文件第一行为注释......
	while (fgets(line_buffer, 50, fptr)) {
		//if(line_buffer[0] == '(') {
		sscanf(line_buffer, "(%d, %d, %d, %d, %d, %d, %d)", &test_road_id, &length, &max_speed, &channel, &from, &to, &isDuplex);
		auto pRoad = new ROAD(test_road_id, length, max_speed, channel);
		graph.add_node(test_road_id, from, to, pRoad->capacity, isDuplex);
		//road_vec.push_back(pRoad);
		road_map[test_road_id] = pRoad;
		//}
	}
	fclose(fptr);

	/*End of read road configuration */

	/* END of read configuration from file */


	/* TODO:process */


	vector<CAR*> cars_running;//已经在运行的车辆
	list<CAR*> cars_waiting;//已经在运行的车辆

				  //取前面一百个
	int element_idx;
	vector<CAR*>::iterator start = car_vec.begin();

	while (true) {
		/*
		车辆按照出发时间、行驶速度依次排序
		*/
		//是否没有车辆需要调度
		if (start == car_vec.end())
			break;
		/*sort(car_vec.begin(), car_vec.end(),
		[](CAR* a, CAR* b) -> bool { return a->speed > b->speed; });
		stable_sort(car_vec.begin(), car_vec.end(),
		[](CAR* a, CAR* b) -> bool { return a->plan_time < b->speed; });*/

		global_time++;
		

		if (car_vec.end() - start >= BATCH_SIZE)
		{
			for (element_idx = BATCH_SIZE - 1; element_idx--; element_idx >= 0)
			{
				//找到可以出发车辆
				if ((*(start + element_idx))->plan_time <= global_time)
					break;
			}
			//没有找到可以出发的车辆，进入下一轮循环
			if (element_idx == -1)
				continue;
		}
		else
		{
			//剩余车辆不足需要调度的数量，直接全部调度出去
			element_idx = car_vec.end() - start - 1;
		}
			

		//释放行驶车辆的占用容量以及删除理论到达车辆
		release_capacity(cars_running, global_time, &graph);



		for (list<CAR*>::iterator car = cars_waiting.begin(); car != cars_waiting.end(); car++)
		{
			//需要在内部解决开销容量减少问题，以及记录理论到达各个node的时间
			GRAPH::route_type &result = graph.get_least_cost_route((*car)->from, (*car)->to, (*car)->speed, (*car), global_time);
			if (!result.empty())
			{
				cars_waiting.erase(car);
			}
		}

		//调度车辆
		for (vector<CAR*>::iterator car = start; car != start + element_idx + 1; car++)
		{
			//需要在内部解决开销容量减少问题，以及记录理论到达各个node的时间
			GRAPH::route_type &result = graph.get_least_cost_route((*car)->from, (*car)->to, (*car)->speed, (*car), global_time);
			if (result.empty())
			{
				cars_waiting.push_back(*(car));
				continue;
			}
			cars_running.push_back((*car));
		}

		//加入车辆inser
		start += element_idx+1;


		//删除已经调度车辆
		//指针清理可以删除，节省运行时间
		/*for (vector<CAR*>::iterator car = car_vec.begin(); car != car_vec.begin() + element_idx + 1; car++)
		{
			delete (*car);
		}
		car_vec.erase(car_vec.begin, car_vec.begin() + element_idx + 1);*/




		/* End of process */


	}
	/* TODO:write output file */
	ofstream fout(answerPath);
	if (!fout.is_open())
	{
		cerr << "无法打开文件 " << answerPath << endl;
		exit(0);
	}
	//写入文件接口
	//write_to_file(tem_vec,car,fout)
	fout.close();
	/* End of write out file */
	return 0;
}
