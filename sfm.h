#include<math.h>
#include "stdio.h"
#include "stdlib.h"
#include "time.h"
#include<vector>
#include<iostream>
#include<list>

using namespace std;

// 数量限制
#define MAX_AGENT_NUM 1000 // 最大agent数量，因为使用的是vector所以实际上没用到
#define MAX_OBLINE_NUM 1000 // 最大障碍物数量

// 建模参数
#define MAX_V 1.02 // agent 最大速度
#define sense_range 5 // 感知范围：m
#define density 320 // 质量/半径，m/density = r 320
#define tao 0.5 // 目前方向转移到目标方向的加速度参数



// sfm参数
#define k1 120000 // body force parameter
#define k2 240000 // tangential force parameter 切向力参数s
#define A 2000 // N// A * exp[dis/B] only social force
#define B 0.08 // m

//行列数
int col_num = 0;
int row_num = 0;//init in init_map

// 点阵图放大比
double  map_factor = 10;

// 程序数据
vector<vector<int>> map_matrix; // 点阵图
vector<vector<int>> density_map; // 人群密度图,给A*考虑选取人少的路径



// 结构体
struct cordinate {
	double x;
	double y;
	cordinate() = default;
	cordinate(double a, double b) {
		x = a;
		y = b;
	}
}; // 坐标结构体
struct AGENT
{
	int id;

	double m; // mass
	double x; // pos
	double y;
	double vx; // v
	double vy;
	double gx; // final goal pos
	double gy;
	double next_gx; // next goal pos
	double next_gy;
	double v0; // max velocity
	double tao_1 = 0.1;

	double dis; // 起点到终点距离
	int jam_time=0; // 连续多次速度过小时增加，达到设定次数时判定为卡住，重新寻路
	int np = 0;// no path flag
	int color = 0;
	double arrive_time = 0;


	list<cordinate> path; // A* 路径存储
}; // agent 结构体
struct OBLINE
{
	double sx; // start point
	double sy;
	double ex; // end point
	double ey;
	double len; // length

}; // 障碍物结构体



// 函数声明
double randval(double, double);
double agent_dis(AGENT, AGENT); // agents 之间距离
void agent_force(AGENT*, AGENT*, double*, double*,double); // agents 之间作用力
double point_to_line_dis(double, double, double, double, double, double, double, double*, double*); // 计算点到线距离
void obline_force(AGENT*, OBLINE*, double*, double*); // 计算障碍物作用力



// 函数实现
double randval(double a, double b)
{
	return a + (b - a) * rand() / (double)RAND_MAX;
}


double agent_dis(AGENT* a1, AGENT* a2)
{
	return sqrt((a1->x - a2->x) * (a1->x - a2->x) + (a1->y - a2->y) * (a1->y - a2->y));
}


void agent_force(AGENT* a1, AGENT* a2, double* fx, double* fy,double dis) // a1<-a2
{
	// 感知距离判断放在函数调用之前了
	if (dis == 0)
	{
		dis = 1e-10;//防止出现dis=0
	}
	double dx = (a1->x - a2->x) / dis; // a2对a1施加的力的单位方向向量的x
	double dy = (a1->y - a2->y) / dis; // a2对a1施加的力的单位方向向量的y

	//对冲判断
	double cos1 =  (a2->vx*dx + a2->vy * dy) / sqrt(dx * dx + dy * dy) / sqrt(a2->vx * a2->vx + a2->vy * a2->vy);  // a2速度和a2对a1作用力方向夹角cos
	double sin = (a2->vx * dy - a2->vy * dx) / sqrt(dx * dx + dy * dy) / sqrt(a2->vx * a2->vx + a2->vy * a2->vy); // a2速度和a2对a1作用力方向夹角sin
	double cos2 = (-a1->vx * dx - a1->vy * dy) / sqrt(dx * dx + dy * dy) / sqrt(a1->vx * a1->vx + a1->vy * a1->vy); // a1速度的反方向和a2对a1作用力方向夹角

	if (cos1 >= 0.866 && cos2 >= 0.866)//60`
	{
		if (sin >= 0)
		{
			dx = dx + dy;
			dy = dy - dx;

		}
		else
		{
			dx = dx - dy;
			dy = dy + dx;

		}
	}
	
	double rij = a1->m / density + a2->m / density; // 两个agent半径和
	double delta_d = rij - dis; // 半径和减去中心点距离
	double rif = A * exp(delta_d / B); // repulsive interaction force 斥力
	double bf = delta_d < 0 ? 0 : k1 * delta_d; // body force agents接触时存在的力
	// 斥力和body force的合力
	double res_f_x = (rif + bf) * dx;
	double res_f_y = (rif + bf) * dy;

	// 切向力 agents接触时存在
	double tfx = 0;
	double tfy = 0;
	if (delta_d > 0) // 身体半径有接触时
	{
		double tix = -dy;
		double tiy = dx;
		double delta_v = (a2->vx - a1->vx) * tix + (a2->vy - a1->vy) * tiy; // 切向速度差
		tfx = k2 * delta_d * delta_v * tix;
		tfy = k2 * delta_d * delta_v * tiy;
	}
	*fx += res_f_x + tfx;
	*fy += res_f_y + tfy;

	return;
		
}


double point_to_line_dis(double ax, double ay, double sx, double sy, double ex, double ey, double d, double* px, double* py)//p为垂足
{
	// 该函数需要计算垂足，用于障碍物对agent作用力方向的计算
	double dis=0;
	double dot_pro = ((ax - sx) * (ex - sx) + (ay - sy) * (ey - sy)) / (d*d); // dot product of s_a and s_e divide by d2(s->start point of line;e->end point of line;a->agent)
	
	if (dot_pro <= 0) // 垂足在start point外侧
	{
		dis = sqrt((ax - sx) * (ax - sx) + (ay - sy) * (ay - sy));
		*px = sx;
		*py = sy;

	}
	else if (dot_pro < 1) // 垂足在线段内
	{
		*px = sx + (ex - sx) * dot_pro;
		*py = sy + (ey - sy) * dot_pro;
		dis = sqrt((ax - *px) * (ax - *px) + (ay - *py) * (ay - *py));
		
	}
	else if (dot_pro >= 1) // 垂足在end point外侧
	{
		dis = sqrt((ax - ex) * (ax - ex) + (ay - ey) * (ay - ey));
		*px = ex;
		*py = ey;

	}
	return dis;

}


void obline_force(AGENT* a, OBLINE* l, double* fx, double* fy)
{
	double px, py;
	double dis = point_to_line_dis(a->x, a->y, l->sx, l->sy, l->ex, l->ey, l->len, &px, &py);
	if (dis <= 0)
	{
		dis = 1e-10;//防止出现dis=0
	}
	if (dis > sense_range) // 感知距离判断在函数内
	{
		return;
	}
	double riw = a->m / density - dis; // 半径-距离
	double rif = A * exp(riw / B);
	double bf = riw < 0 ? 0 : k1*riw;
	//if (bf != 0)cout << endl;
	// 点与线作用力点的方向向量
	double nx = (a->x-px) / dis;
	double ny = (a->y-py) / dis;
	// 斥力和body force合力
	double res_f_x = (rif + bf) * nx;
	double res_f_y = (rif + bf) * ny;
	// 切向力
	double tfx = 0;
	double tfy = 0;
	if (riw > 0)
	{
		double tix = -ny;
		double tiy = nx;
		double delta_v = a->vx * tix + a->vy * tiy;//切向速度差
		tfx = k2 * riw * delta_v * tix;
		tfy = k2 * riw * delta_v * tiy;
	}

	*fx += res_f_x - tfx;
	*fy += res_f_y - tfy;


	return;

}


//---------------------------------------------以上为SFM部分------------------------------------------------

//---------------------------------------------以下为A★部分------------------------------------------------

//函数声明
void A_star(AGENT*);
bool in_map(int, int);

// 结构体
struct node
{
	int x;
	int y;
	int g; // 移动代价
	int h; // 估算成本
	bool flag; // 这里用flag来表示node被推入close_list
	bool check;
	
	

	node* parent; // 父节点,从终点依次指向起点

	node() = default;

	node(int a, int b, double h = 0) {
		flag = 0;
		check = 0;
		x = a;
		y = b;
		h = h;
	}

	/*static bool compare(node& a, node& b)
	{
		return (a.g + a.h) > (b.g + b.h);
	}*/

};

// 数据
vector<vector<node>> map_matrix_A;

// 可调参数
const int a_step = 5;
vector<cordinate> direction = { {a_step,0}, {-a_step,0}, {0,a_step}, {0,-a_step} }; // 正向
vector<cordinate> ob_direction = { {a_step,a_step},{a_step,-a_step},{-a_step,-a_step},{-a_step,a_step} }; // 斜向
const int path_len = 1;
const int max_time = 800; // ms


//函数实现
void A_star(AGENT* a)
{
	list<node*> open_list;

	//初始化close_list 和 check
	for (auto& y : map_matrix_A)
	{
		for (auto& x : y)
		{
			x.flag = 0;
			x.check = 0;
		}
	}
	
	// 初始化起点 目前社会力模型正确 应该是不会出现被挤出地图的情况 所以这部分可以注掉
	/*if (!in_map(int(a->x * map_factor), int(a->y * map_factor)))
	{
		a->path.clear();
		a->gx = a->x;
		a->gy = a->y;
		a->next_gx = a->x;
		a->next_gy = a->y;
		cout << "***********************************************no path" << endl;
		a->np = 1;
		return;
	}*/
	open_list.push_back(&map_matrix_A[int(a->y * map_factor)][int(a->x * map_factor)]);
	map_matrix_A[int(a->y * map_factor)][int(a->x * map_factor)].x = int(a->x * map_factor);
	map_matrix_A[int(a->y * map_factor)][int(a->x * map_factor)].y = int(a->y * map_factor);
	map_matrix_A[int(a->y * map_factor)][int(a->x * map_factor)].flag = 1;
	map_matrix_A[int(a->y * map_factor)][int(a->x * map_factor)].h = (abs(int(a->gx*map_factor) - int(a->x*map_factor)) + abs(int(a->gy*map_factor) - int(a->y*map_factor)))*10;
	map_matrix_A[int(a->y * map_factor)][int(a->x * map_factor)].g = 0;
	map_matrix_A[int(a->y * map_factor)][int(a->x * map_factor)].parent = nullptr;

	clock_t start, end; // 加个时间统计,用于判断无路径的情况
	start = clock();

	node* temp = open_list.front(); // 目前搜索的点

	
	while (!(abs(temp->x - a->gx * map_factor) <= a_step && abs(temp->y - a->gy * map_factor) <= a_step)) 
	{
		double f = INT_MAX;
		//找F最小的节点
		for (auto* a : open_list)
		{
			if ((a->g + a->h) < f)
			{
				f = (a->g + a->h);
				temp = a;
			}
		}

		bool flag = 0;
		// 判断周围是否有墙
		/*for (auto d : direction)
		{
			if (in_map(temp->x + d.x, temp->y + d.y) && map_matrix[temp->y + d.y][temp->x + d.x] == 0)
			{
				flag = 1;
				break;
			}
		}
		for (auto d : ob_direction)
		{
			if (in_map(temp->x + d.x, temp->y + d.y) && map_matrix[temp->y + d.y][temp->x + d.x] == 0)
			{
				flag = 1;
				break;
			}
		}*/

		// 正方向上搜索,此时移动代价为10
		for (auto d : direction)
		{
			if (in_map(temp->x + d.x, temp->y + d.y))
			{
				bool wall = 0;
				if (in_map(temp->x + d.x, temp->y + d.y))
				{
					for (double i = 1; i <= a_step; ++i)
					{
						// cout << d.y * (i / a_step) <<" " << d.x * (i / a_step) << endl;
						if (map_matrix[temp->y + d.y * (i / a_step)][temp->x + d.x * (i / a_step)] == 0)
						{
							wall = 1;
							flag = 1;
							break;
						}
					}
					if (wall)
					{
						continue;
					}
				int point_type = map_matrix[temp->y + d.y][temp->x + d.x];
				//if (density_map[temp->y + d.y][temp->x + d.x])point_type == 0;
				//cout << density_map[temp->y + d.y][temp->x + d.x] << endl;
				if (point_type != 0 && map_matrix_A[temp->y + d.y][temp->x + d.x].check == 0)//节点在地图内，且不在障碍物部分,且未check过
				{
					if (map_matrix_A[temp->y + d.y][temp->x + d.x].flag == 0)//表示该点第一次进入open_list
					{
						map_matrix_A[temp->y + d.y][temp->x + d.x].x = temp->x + d.x;
						map_matrix_A[temp->y + d.y][temp->x + d.x].y = temp->y + d.y;
						map_matrix_A[temp->y + d.y][temp->x + d.x].h = (abs(int(a->gx * map_factor) - (temp->x + d.x)) + abs(int(a->gy * map_factor) - (temp->y + d.y))) * 10;
						map_matrix_A[temp->y + d.y][temp->x + d.x].g = temp->g + 10 * a_step*(point_type*0.75 + density_map[temp->y + d.y][temp->x + d.x]);
						map_matrix_A[temp->y + d.y][temp->x + d.x].parent = temp;
						map_matrix_A[temp->y + d.y][temp->x + d.x].flag = 1;
					}
					else // 再次筛查一个点d,只有在通过(这次搜索的点temp到达d的代价)比(d的父节点到d)更小时,才会更新
					{
						if (temp->g + 10 * a_step < map_matrix_A[temp->y + d.y][temp->x + d.x].g)
						{
							map_matrix_A[temp->y + d.y][temp->x + d.x].g = temp->g + 10 * a_step * (point_type*0.75 + density_map[temp->y + d.y][temp->x + d.x]);
							map_matrix_A[temp->y + d.y][temp->x + d.x].parent = temp;
						}
					}
					open_list.push_back(&map_matrix_A[temp->y + d.y][temp->x + d.x]);
				}
			}

		}

		// 斜方向上搜索,此时移动代价为14,实际上应该是10*2^(1/2),小于这个值会导致结果更偏向斜向移动
		if (flag==0)
		{
			for (auto d : ob_direction)
			{
				if (in_map(temp->x + d.x, temp->y + d.y))
				{
					int point_type = map_matrix[temp->y + d.y][temp->x + d.x];
					//if (density_map[temp->y + d.y][temp->x + d.x])point_type == 0;
					if (point_type != 0 && map_matrix_A[temp->y + d.y][temp->x + d.x].check == 0)//节点在地图内，且不在障碍物部分,且未check过 && map_matrix_A[temp->y + d.y][temp->x + d.x].check == 0
					{
						if (map_matrix_A[temp->y + d.y][temp->x + d.x].flag == 0) // 表示该点第一次进入open_list
						{
							map_matrix_A[temp->y + d.y][temp->x + d.x].x = temp->x + d.x;
							map_matrix_A[temp->y + d.y][temp->x + d.x].y = temp->y + d.y;
							map_matrix_A[temp->y + d.y][temp->x + d.x].h = (abs(int(a->gx * map_factor) - (temp->x + d.x)) + abs(int(a->gy * map_factor) - (temp->y + d.y))) * 10;
							map_matrix_A[temp->y + d.y][temp->x + d.x].g = temp->g + 14 * a_step * (point_type*0.75 + density_map[temp->y + d.y][temp->x + d.x]);
							map_matrix_A[temp->y + d.y][temp->x + d.x].parent = temp;
							map_matrix_A[temp->y + d.y][temp->x + d.x].flag = 1;
						}
						else // 再次筛查一个点d,只有在通过(这次搜索的点temp到达d的代价)比(d的父节点到d)更小时,才会更新
						{
							if (temp->g + 14 * a_step < map_matrix_A[temp->y + d.y][temp->x + d.x].g)
							{
								map_matrix_A[temp->y + d.y][temp->x + d.x].g = temp->g + 14 * a_step * (point_type*0.75 + density_map[temp->y + d.y][temp->x + d.x]);
								map_matrix_A[temp->y + d.y][temp->x + d.x].parent = temp;
							}
						}
						open_list.push_back(&map_matrix_A[temp->y + d.y][temp->x + d.x]);
					}
				}
			}
		}

		open_list.remove(temp);
		temp->check = 1; // 已经检查过这个点

		// 无路径判断,绝大部分在n s之内可以搜到,超过视为不存在路径
		end = clock();
		if (end - start > max_time)
		{
			a->path.clear();
			a->gx = a->x;
			a->gy = a->y;
			a->next_gx = a->x;
			a->next_gy = a->y;
			a->np = 1;
			return;
		}

	}
	// push 路径到 path,这里可以选择路径密度
	int counter = 0;
	do
	{
		counter++;
		if (counter == path_len) // 每n个取一个点
		{
			a->path.push_front(cordinate(temp->x, temp->y));
			counter = 0;
		}
		else if (map_matrix[temp->y][temp->x] == 2) // 门口重要节点也要push
		{
			a->path.push_front(cordinate(temp->x, temp->y));
		}
		if (temp->parent == nullptr)break;
		temp = temp->parent;
		
	} while (temp!=nullptr);
	if (!a->path.empty())
	{
		a->next_gx = a->path.front().x / map_factor;
		a->next_gy = a->path.front().y / map_factor;
	}
	else 
	{
		a->next_gx = a->gx;
		a->next_gy = a->gy;
	}

}


bool in_map(int x,int y)
{
	return (x >= 0 && x < col_num-1) && (y >= 0 && y < row_num-1);
}


