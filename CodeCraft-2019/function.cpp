//----------------------
//�ͷ�����ʻ����ռ�õ������Լ�ɾ�����ﳵ��
void release_capacity(std::vector<CAR*>& car_running, int global_time, GRAPH *graph);
//--------------------------

//------------
//
//���answer�ļ�
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
			//�Ѿ���������ڵ㣬�ָ��������ҽ�����·��stack��ɾ��
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