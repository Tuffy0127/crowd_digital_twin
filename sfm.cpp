#include"sfm.h"
#include<iostream>
#include<fstream>
#include<vector>
#include<sstream>
#include<string>
#include<stdio.h>
#include<omp.h>
#include<time.h>

using namespace std;

const int thread_num = 12; // OpenMP�߳���

// �ɵ�����
int agent_num = 500;
int step_num = 5000;
double tick = 0.03;//timestep
int jam_time_threshole = 50;

// ��������
int  obstical_line_num = 0;
clock_t total_start, total_end;
vector<AGENT> agent_list; // agent vector
struct OBLINE obstical_lines[MAX_OBLINE_NUM]; // obstical line array

// ����ļ�
FILE* f = fopen("C:/Users/leesh/Desktop/srp/output/output.txt", "w");
FILE* ff = fopen("C:/Users/leesh/Desktop/srp/output/test.txt", "w");

// �յ�
struct cordinate goal[6] = { {5,22},{33,22},{5,34},{33,34},{5,56},{33,56} };



// ��������
void init_obline(string);//�ϰ������
void init_map(string);
void init_agent(int);
void init();
void output(AGENT*);//д��agent����
void step();
void update_density();
void test();


// ����ʵ��
void init_obline(string obline_file)
{
	fstream infile(obline_file, ios::in);
	string buf;
	int num = 0;
	getline(infile, buf);
	while (!infile.eof()) {
		stringstream ss(buf);
		ss >> obstical_lines[num].sy >> obstical_lines[num].sx >> obstical_lines[num].ey >> obstical_lines[num].ex;
		obstical_lines[num].sx /= map_factor;
		obstical_lines[num].sy /= map_factor;
		obstical_lines[num].ex /= map_factor;
		obstical_lines[num].ey /= map_factor;
		obstical_lines[num].len = sqrt((obstical_lines[num].sx - obstical_lines[num].ex) * (obstical_lines[num].sx - obstical_lines[num].ex) +
			(obstical_lines[num].sy - obstical_lines[num].ey) * (obstical_lines[num].sy - obstical_lines[num].ey));
		num++;
		getline(infile, buf);

	}
	obstical_line_num = num;
	infile.close();
}


void init_map(string map_file)
{
	fstream infile(map_file, ios::in);
	string buf = "";
	getline(infile, buf);
	while (!infile.eof())
	{
		row_num++;
		stringstream ss(buf);
		map_matrix.push_back(vector<int>());
		int temp;
		while (!ss.eof())
		{
			if (row_num == 1)
			{
				col_num++;
			}
			ss >> temp;
			map_matrix[map_matrix.size() - 1].push_back(temp);
		}
		getline(infile, buf);


	}
	col_num--;
	infile.close();
	for (int i = 0; i < row_num; i++)
	{
		vector<node> temp;
		for (int j = 0; j < col_num; j++)
		{
			temp.push_back(node(j, i));
		}
		map_matrix_A.push_back(temp);
	}
	for (int i = 0; i < row_num; i++)
	{
		vector<int> temp;
		for (int j = 0; j < col_num; j++)
		{
			temp.push_back(0);
		}
		density_map.push_back(temp);
	}

}


void init_agent(int agent_num)
{
	cout << "Initializing " << agent_num << " agent(s)" << endl;;
	for (int i = 0; i < agent_num; i++)
	{
		int rand = int(randval(0, 6));
		//cout << rand << endl;
		AGENT a;
		a.id = i;
		do 
		{
			a.x = randval(0, col_num-1) / map_factor;
			a.y = randval(0, row_num-1) / map_factor;
			//a.x = 31.35;
			//a.y = 50.61;
			
		} while (map_matrix[int(a.y*map_factor)][int(a.x*map_factor)]==0);
		
		a.m = 80;
		a.vx = randval(-2, 2);
		a.vy = randval(-2, 2);
		a.v0 = MAX_V;
		//a.gx = 32;
		//a.gy = 22.5;
		a.gx = goal[rand].x;
		a.gy = goal[rand].y;
		a.color = rand;
		//cout << a.gx << " " << a.gy << endl;
		//a.next_gx = 32;
		//a.next_gy = 22.5;
		a.next_gx = goal[rand].x;
		a.next_gy = goal[rand].y;
		a.dis = sqrt((a.x - a.gx) * (a.x - a.gx) + (a.y - a.gy) * (a.y - a.gy));
		agent_list.push_back(a);
	}
	/*for (auto& a : agent_list)
	{
		cout << a.id << endl;
	}*/

}


void init()
{
	init_obline("./map/obstacles.txt");
	init_map("./map/matrix.txt");
	cout << "row_num: " << row_num << " col_bnum: " << col_num << endl;
	init_agent(agent_num);
}


void output(AGENT* a)
{
	fprintf(f, "%d %g %g %d %d\n", a->id, a->x, a->y, a->np, a->color);
	fprintf(ff, "%d,%g,%g,%g,%g\n", a->id, a->x, a->y, a->vx, a->vy);
}


void step()
{
	fprintf(f, "%lld\n", agent_list.size());
	fprintf(ff, "%lld\n", agent_list.size());

	//����������
	//fprintf(f, "%lu\n", 2);

	for (int i =0;i<agent_list.size();++i)
	{
		AGENT* a = &agent_list[i];
		if (a->np == 1)
		{
			output(a);
			continue;
		}
		//first compute the desired direction;
		double goal_dis = sqrt((a->x - a->next_gx) * (a->x - a->next_gx) + (a->y - a->next_gy) * (a->y - a->next_gy));
		a->tao_1 = 0.1 + (goal_dis / a->dis) * 0.6;
		double dx = a->v0 * (a->next_gx - a->x) / goal_dis;//��������������x
		double dy = a->v0 * (a->next_gy - a->y) / goal_dis;
		
		double ax = (dx - a->vx) / a->tao_1;//���ٶ�
		double ay = (dy - a->vy) / a->tao_1;

		double total_force_x = 0;
		double total_force_y = 0;

		if (a->vx <= 0.1 && a->vy <= 0.1 && a->np != 1)
		{
			a->tao_1 *= ((jam_time_threshole - a->jam_time) / jam_time_threshole);
			a->jam_time += 1;
			
		}
		else
		{
			a->jam_time = 0;
		}
		
		
		int agent_counter = 0;

		for (int j = 0;j<agent_list.size();++j)
		{
			AGENT* b = &agent_list[j];
			if (b->id == a->id)continue;
			double dis = agent_dis(a, b);
			if (dis <= sense_range)//������֪��Χ������
			{
				agent_counter++;
				agent_force(a, b, &total_force_x, &total_force_y,dis);
			}
		}
		//cout << agent_counter << endl;
		//12 25
		if (a->jam_time >= jam_time_threshole && agent_counter>10 && agent_counter<=20) // ��û��������Ⱥ����·��,�Ѿ�����Ⱥ��ľͱ���������ʵʵ����
		{
			a->jam_time = 0;
			a->path.clear();
			A_star(a);

		}
		else if (a->jam_time >= jam_time_threshole + 50)
		{
			a->jam_time = 0;
			a->path.clear();
			A_star(a);
		}

		
		//int id = omp_get_thread_num();
		for (int j = 0; j < obstical_line_num; j++)
		{
			obline_force(a, &obstical_lines[j], &total_force_x, &total_force_y);
		}

		ax += total_force_x / a->m;//���ٶȼ����������Ӱ��
		ay += total_force_y / a->m;

		a->vx += ax * tick;
		a->vy += ay * tick;

		double new_v = sqrt(a->vx * a->vx + a->vy * a->vy);//���º���ٶȴ�С
		if (new_v > a->v0)//��������ٶȺ����
		{
			a->vx = a->vx * a->v0 / new_v;
			a->vy = a->vy * a->v0 / new_v;
		}
		new_v = sqrt(a->vx * a->vx + a->vy * a->vy);

		a->x += a->vx * tick;
		a->y += a->vy * tick;

		if (a->x < 0)
		{
			a->x = 1e-10;
		}
		if (a->y < 0)
		{
			a->y = 1e-10;
		}
		
		if ( a->path.size()>1 && sqrt((a->x * map_factor - a->path.front().x) * (a->x * map_factor - a->path.front().x) + (a->y * map_factor - a->path.front().y) * (a->y * map_factor - a->path.front().y)) < 2*a_step )
		{

			a->path.pop_front();
			a->next_gx = a->path.front().x / map_factor;
			a->next_gy = a->path.front().y / map_factor;
		}

		output(a);

	}
}


void update_density()
{
	for (int i = 0; i < row_num; ++i)
	{
		
		for (int j = 0; j < col_num; ++j)
		{
			density_map[i][j] = 0;
		}
	}
	for (int i = 0; i < agent_list.size(); ++i)
	{
		for (int j = -2; j <= 2; ++j)
		{
			for (int k = -2; k <= 2; ++k)
			{
				
				if (in_map(int(agent_list[i].x * map_factor+j), int(agent_list[i].y * map_factor+k)))
				{
					density_map[int(agent_list[i].y * map_factor+k)][int(agent_list[i].x * map_factor+j)] += 10;
				}
			}
		}
		
		
	}
}


void test()
{
	init_obline("./map/obstacles3.txt");
	init_map("./map/matrix3.txt");
	cout << "row_num: " << row_num << " col_bnum: " << col_num << endl;

	AGENT a;
	a.id = 1;
	a.m = 80;
	a.gx = 1;
	a.gy = 0;
	a.x = 0;
	a.y = 0;

	A_star(&a);



}



// main
int main()
{
	total_start = clock();
	
	// omp_set_num_threads(thread_num);

	init();


	// ��ʼ״̬���
	fprintf(f, "%lld\n", agent_list.size());
	fprintf(ff, "%d,%g,%d,%d\n", step_num, map_factor, col_num, row_num);
	fprintf(ff, "%lld\n", agent_list.size());
	for (int i = 0;i<agent_num;++i)
	{
		output(&agent_list[i]);
	}

	// ��ʼA��
	int counter = 0;
	cout << "Initializing path" << endl;
	for (auto& a : agent_list)
	{
		A_star(&a);
		counter++;
		printf("A_star: %d / %d \r", counter, agent_num);
	}
	cout << endl;

	// ��ʼģ��
	clock_t start, end;
	cout << "Start steps..." << endl;
	start = clock();
	for (int i = 0; i < step_num; ++i)
	{
		step();
		
		if ((i + 1) % 50 == 0)
		{
			update_density();
		}
		if ((i+1) % 10 == 0)
		{
			
			end = clock();
			printf("Step: %d / %d ;Remaining time: %d s             \r", i + 1, step_num, (end - start) * (step_num - i - 1)/ 10 / 1000);
			start = clock();
		}
		
	}
	cout << endl;

	total_end = clock();
	cout << "Done " << step_num << " step(s). Total time :"<< (total_end-total_start)/1000/60<< " min(s)" << endl;

}