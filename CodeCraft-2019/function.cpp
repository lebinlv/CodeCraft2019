//----------------------
//释放在行驶车辆占用的容量以及删除到达车辆
void release_capacity(std::vector<CAR*>& car_running, int global_time, GRAPH *graph);
//--------------------------

//------------
//
//输出answer文件
void write_to_file(vector<GRAPH::Node*> & tem_vec, CAR & car, ofstream &fout);
//------------
//----------------
void release_capacity(std::vector<CAR*>& car_running, int global_time, GRAPH *graph)
{
	CAR::Past_node node;
	for (vector<CAR*>::iterator car = car_running.begin(); car != car_running.end(); car++)
	{
		node = (*car)->past_nodes.top();
		if (node.arrive_time < global_time)
		{
			//已经到达这个节点，恢复容量并且将它从路径stack中删除
			graph->capacity_vec[node.node->capacity_idx] += CAPACITY_FACTOR;
			(*car)->past_nodes.pop();
		}

		if ((*car)->past_nodes.size() == 0)
			car_running.erase(car);
	}
}
//--------------------
//--------------
void write_to_file(vector<GRAPH::Node*> & tem_vec, CAR & car, std::ofstream &fout)
{
	fout << '(' << car.id << ", " << car.start_time;
	for (vector<GRAPH::Node *>::reverse_iterator start = tem_vec.rbegin(); start != tem_vec.rend(); ++start)
		fout << ", " << (*start)->road_id;
	fout << ")\n";
}
//---------------