# SRP

[TOC]

11



## SFM框架



### 参数

| Key            | Value  | Description                                                  |
| -------------- | ------ | ------------------------------------------------------------ |
| agent_num      | 500    | agent数量                                                    |
| MAX_OBLINE_NUM | 1000   | 障碍物线的数量                                               |
| MAX_V          | 1.02   | agent最大速度(m/s)                                           |
| sense_range    | 5      | agent感知范围(m)                                             |
| density        | 320    | agent重量半径比(kg/m) <br />E.g 80(kg) / 320(kg/m) = 0.25m   |
| tao            | 0.5    | agent驱动力参数(数值越小agent驱动力越强)<br />(更新驱动力衰减后在agent的属性里) |
| k1             | 120000 | body force(躯干力) 参数                                      |
| k2             | 240000 | tangential force(切向力) 参数                                |
| A              | 2000   | social force(社会力) 参数(N)                                 |
| B              | 0.08   | social force(社会力) 参数(m)                                 |
| map_factor     | 10     | 点阵图与实际的比例                                           |



### 结构体



#### cordinate

| Key  | Type   | Value | Description |
| ---- | ------ | ----- | ----------- |
| x    | double | 0     | x坐标       |
| y    | double | 0     | y坐标       |

构造函数

```C++
cordinate (double a, double b) 
{
		x = a;
		y = b;
}
```



#### AGENT

| Key      | Type   | Value          | Description                                 |
| -------- | ------ | -------------- | ------------------------------------------- |
| id       | int    | 0              | agent id                                    |
| m        | double | 80             | agent 质量                                  |
| x        | double | 1              | agent x坐标                                 |
| y        | double | 1              | agent y坐标                                 |
| vx       | double | 0.5            | agent 速度x方向                             |
| vy       | double | 0.5            | agent 速度y方向                             |
| gx       | double | 10             | agent 最终目标x坐标                         |
| gy       | double | 10             | agent 最终目标y坐标                         |
| next_gx  | double | 2              | agent 下一目标x坐标(A⭐)                     |
| next_gy  | double | 2              | agent 下一目标y坐标(A⭐)                     |
| v0       | double | 1.02           | agent 最大速度(默认使用MAX_V)               |
| dis      | double | 9√2            | agent 从起点到最终目标的距离(驱动力衰减)    |
| jam_time | int    | 0              | 用于agent卡在某一位置的判断                 |
| np       | int    | 0              | np==1时, agent 搜索不到到最终目标的路径(A⭐) |
| tao_1    | double | 0.5            | agent 的驱动力参数(驱动力衰减)              |
| path     | list   | cordinate(2,2) | agent 的A⭐路径                              |



#### OBLINE

| Key  | Type   | Value | Description |
| ---- | ------ | ----- | ----------- |
| sx   | double | 0     | 起点 x坐标  |
| sy   | double | 0     | 起点 y坐标  |
| ex   | double | 1     | 终点 x坐标  |
| ey   | double | 1     | 终点 y坐标  |
| len  | double | √2    | 长度        |



### 函数



#### agent_force

该函数用于计算agents之间的作用力

公式:

![image-20230725112642790](C:\Users\leesh\AppData\Roaming\Typora\typora-user-images\image-20230725112642790.png)

(距离为0的情况需要判断)

(这里的感知距离判断放在函数外)

计算作用力方向: ![image-20230725141806614](C:\Users\leesh\AppData\Roaming\Typora\typora-user-images\image-20230725141806614.png)

```C++
double dx = (a1->x - a2->x) / dis; // a2对a1施加的力的单位方向向量的x
double dy = (a1->y - a2->y) / dis; // a2对a1施加的力的单位方向向量的y
```



💥对冲偏转

```C++
double cos1 =  (a2->vx*dx + a2->vy * dy) / sqrt(dx * dx + dy * dy) / sqrt(a2->vx * a2->vx + a2->vy * a2->vy); // a2速度和a2对a1作用力方向夹角cos
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
```

对冲偏转是为了让迎面走来的两个agent能相互错开，a1在a2左侧,a2就对a1增加向左的力，反之则反。

此处不仅要判断a1是否在a2的前进方向上，也需要判断a2是否在a1的方向上，不然a1会被它背后的agent拉住。





计算半径减去中心点距离: ![image-20230725113351181](C:\Users\leesh\AppData\Roaming\Typora\typora-user-images\image-20230725113351181.png)

```C++
double rij = a1->m / density + a2->m / density; // 两个agent半径和
double delta_d = rij - dis; // 半径和减去中心点距离
```



排斥性相互作用力(社会力): ![image-20230725113546393](C:\Users\leesh\AppData\Roaming\Typora\typora-user-images\image-20230725113546393.png)

```C++
double rif = A * exp(delta_d / B); // repulsive interaction force 斥力
```



躯干力: ![image-20230725113702643](C:\Users\leesh\AppData\Roaming\Typora\typora-user-images\image-20230725113702643.png)

```C++
double bf = delta_d < 0 ? 0 : k1 * delta_d; // body force agents接触时存在的力
```



切向力: ![image-20230725113733647](C:\Users\leesh\AppData\Roaming\Typora\typora-user-images\image-20230725113733647.png)

```C++
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
```



#### point_to_line_dis

该函数用于计算点到直线距离, 用于agent与障碍物线之间作用力的计算, 同样需要计算作用力的点

首先计算点积和线段长度平方比值

再分为三种情况:

1. 垂足在线段内(0<=比值<=1): 返回点到垂足的距离,垂足为作用力点

   ~~~C++
   *px = sx + (ex - sx) * dot_pro; // 垂足坐标
   *py = sy + (ey - sy) * dot_pro;
   dis = sqrt((ax - *px) * (ax - *px) + (ay - *py) * (ay - *py));
   ~~~

   

2. 垂足在线段外靠近起点一端(比值<=0): 返回点到起点的距离,起点为作用力点

   ```C++
   dis = sqrt((ax - sx) * (ax - sx) + (ay - sy) * (ay - sy));
   *px = sx;
   *py = sy;
   ```

   

3. 垂足在线段外靠近终点一端(比值>=1): 返回点到终点的距离,终点为作用力点

   ```c++
   dis = sqrt((ax - ex) * (ax - ex) + (ay - ey) * (ay - ey));
   *px = ex;
   *py = ey;
   ```



#### obline_force

该函数用于计算agent和障碍物线之间的作用力

公式:

![image-20230725141231859](C:\Users\leesh\AppData\Roaming\Typora\typora-user-images\image-20230725141231859.png)

(距离为0的情况需要判断)

(这里的感知距离判断放在函数内)

计算作用力方向: ![image-20230725141749949](C:\Users\leesh\AppData\Roaming\Typora\typora-user-images\image-20230725141749949.png)

```C++
// 点与线作用力点的方向向量
double nx = (a->x-px) / dis;
double ny = (a->y-py) / dis;
```



计算半径减去距离: ![image-20230725141500491](C:\Users\leesh\AppData\Roaming\Typora\typora-user-images\image-20230725141500491.png)

```c++
double dis = point_to_line_dis(a->x, a->y, l->sx, l->sy, l->ex, l->ey, l->len, &px, &py);
double riw = a->m / density - dis; // 半径-距离
```



排斥性相互作用力(社会力): ![image-20230725141708918](C:\Users\leesh\AppData\Roaming\Typora\typora-user-images\image-20230725141708918.png)

```C++
double rif = A * exp(riw / B);
```



躯干力: ![image-20230725141734729](C:\Users\leesh\AppData\Roaming\Typora\typora-user-images\image-20230725141734729.png)

```C++
double bf = riw < 0 ? 0 : k1*riw;
```



切向力: ![image-20230725141857903](C:\Users\leesh\AppData\Roaming\Typora\typora-user-images\image-20230725141857903.png)

```C++
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
```

 



## A⭐



### 参数

| Key          | Type   | Value                                                        | Description                                     |
| ------------ | ------ | ------------------------------------------------------------ | ----------------------------------------------- |
| direction    | vector | cordinate(1,0), cordinate(0,1), cordinate(-1,0),cordinate(0,-1) | A⭐搜索方向的正方向                              |
| ob_direction | vector | cordinate(1,1), cordinate(1,-1), cordinate(-1,-1),cordinate(-1,1) | A⭐搜索方向的斜方向                              |
| a_step       | int    | 3                                                            | A⭐搜索步长,相当于几个格合并为一个格             |
| path_len     | int    | 2                                                            | A⭐搜索出的路径push间隔(每多少个路径点取一个)    |
| max_time     | int    | 800                                                          | A⭐搜索时间限制(ms),大于该值则判定为搜索不到路径 |
| flag         | int    | 0                                                            | 判断目前搜索的点附近是否有障碍物                |

a_step * path_len 太大会导致在拐角或者门口卡住,因为下个目标点直接透过墙面作用



### 结构体



#### node

| Key    | Type  | Value   | Description                                                  |
| ------ | ----- | ------- | ------------------------------------------------------------ |
| x      | int   | 0       | x坐标                                                        |
| y      | int   | 0       | y坐标                                                        |
| g      | int   | 10      | 从起点到这个节点的移动代价(正向10,斜向14)                    |
| h      | int   | 200     | 估算到终点的成本,采用欧几里得距离(横向差绝对值加纵向差绝对值) |
| flag   | bool  | 0       | 这里用flag来表示节点进入close_list                           |
| check  | bool  | 0       | check为1表示该节点已经作为主节点搜索过,不再通过其他节点访问该点 |
| parent | node* | nullptr | 指向父节点,最终通过该链接反向推出路径                        |



构造函数

```C++
node(int a, int b, double h = 0) {
    flag = 0;
    check = 0;
    x = a;
    y = b;
    h = h;
}
```





#### map_matrix_A



```C++
vector<vector<node>> map_matrix_A;
```





### 主体



open list, close list, check初始化:

```c++
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
```



初始化起点(把agent目前的点作为起点首先推入open list)

```c++
open_list.push_back(&map_matrix_A[int(a->y * map_factor)][int(a->x * map_factor)]);
map_matrix_A[int(a->y * map_factor)][int(a->x * map_factor)].x = int(a->x * map_factor);
map_matrix_A[int(a->y * map_factor)][int(a->x * map_factor)].y = int(a->y * map_factor);
map_matrix_A[int(a->y * map_factor)][int(a->x * map_factor)].flag = 1;
map_matrix_A[int(a->y * map_factor)][int(a->x * map_factor)].h = (abs(int(a->gx*map_factor) - int(a->x*map_factor)) + abs(int(a->gy*map_factor) - int(a->y*map_factor)))*10;
map_matrix_A[int(a->y * map_factor)][int(a->x * map_factor)].g = 0;
map_matrix_A[int(a->y * map_factor)][int(a->x * map_factor)].parent = nullptr;
```



当前点

```c++
node* temp = open_list.front(); // 目前搜索的点
```



当目前的点不在终点的一个step的范围内时不断搜索

```c++
while (!(abs(temp->x - a->gx * map_factor) <= a_step && abs(temp->y - a->gy * map_factor) <= a_step))
```



首先判断G+H最小的点作为当前搜索点

```c++
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
```



判断八个方向是否存在障碍物线, 存在就不进行斜向搜索, 减少agent路线穿过墙角的可能.

```c++
bool flag = 0;
// 判断周围是否有墙
for (auto d : direction)
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
}
```



正方向搜索

```c++
// 正方向上搜索,此时移动代价为10
for (auto d : direction)
{
    if (in_map(temp->x + d.x, temp->y + d.y))
    {
        int point_type = map_matrix[temp->y + d.y][temp->x + d.x];
        //cout << density_map[temp->y + d.y][temp->x + d.x] << endl;
        if (point_type != 0 && map_matrix_A[temp->y + d.y][temp->x + d.x].check == 0)//节点在地图内，且不在障碍物部分,且未check过
        {
            if (map_matrix_A[temp->y + d.y][temp->x + d.x].flag == 0)//表示该点第一次进入open_list
            {
                map_matrix_A[temp->y + d.y][temp->x + d.x].x = temp->x + d.x;
                map_matrix_A[temp->y + d.y][temp->x + d.x].y = temp->y + d.y;
                map_matrix_A[temp->y + d.y][temp->x + d.x].h = (abs(int(a->gx * map_factor) - (temp->x + d.x)) + abs(int(a->gy * map_factor) - (temp->y + d.y))) * 10;
                map_matrix_A[temp->y + d.y][temp->x + d.x].g = temp->g + 10 * a_step*(point_type+ density_map[temp->y + d.y][temp->x + d.x]);
                map_matrix_A[temp->y + d.y][temp->x + d.x].parent = temp;
                map_matrix_A[temp->y + d.y][temp->x + d.x].flag = 1;
            }
            else // 再次筛查一个点d,只有在通过(这次搜索的点temp到达d的代价)比(d的父节点到d)更小时,才会更新
            {
                if (temp->g + 10 * a_step < map_matrix_A[temp->y + d.y][temp->x + d.x].g)
                {
                    map_matrix_A[temp->y + d.y][temp->x + d.x].g = temp->g + 10 * a_step * (point_type + density_map[temp->y + d.y][temp->x + d.x]);
                    map_matrix_A[temp->y + d.y][temp->x + d.x].parent = temp;
                }
            }
            open_list.push_back(&map_matrix_A[temp->y + d.y][temp->x + d.x]);
        }
    }

}
```

此处已经考虑了density_map



斜向搜索

```c++

// 斜方向上搜索,此时移动代价为14,实际上应该是10*2^(1/2),小于这个值会导致结果更偏向斜向移动
if (flag==0)
{
    for (auto d : ob_direction)
    {
        if (in_map(temp->x + d.x, temp->y + d.y))
        {
            int point_type = map_matrix[temp->y + d.y][temp->x + d.x];
            if (point_type != 0 && map_matrix_A[temp->y + d.y][temp->x + d.x].check == 0)//节点在地图内，且不在障碍物部分,且未check过 && map_matrix_A[temp->y + d.y][temp->x + d.x].check == 0
            {
                if (map_matrix_A[temp->y + d.y][temp->x + d.x].flag == 0) // 表示该点第一次进入open_list
                {
                    map_matrix_A[temp->y + d.y][temp->x + d.x].x = temp->x + d.x;
                    map_matrix_A[temp->y + d.y][temp->x + d.x].y = temp->y + d.y;
                    map_matrix_A[temp->y + d.y][temp->x + d.x].h = (abs(int(a->gx * map_factor) - (temp->x + d.x)) + abs(int(a->gy * map_factor) - (temp->y + d.y))) * 10;
                    map_matrix_A[temp->y + d.y][temp->x + d.x].g = temp->g + 14 * a_step * (point_type + density_map[temp->y + d.y][temp->x + d.x]);
                    map_matrix_A[temp->y + d.y][temp->x + d.x].parent = temp;
                    map_matrix_A[temp->y + d.y][temp->x + d.x].flag = 1;
                }
                else // 再次筛查一个点d,只有在通过(这次搜索的点temp到达d的代价)比(d的父节点到d)更小时,才会更新
                {
                    if (temp->g + 14 * a_step < map_matrix_A[temp->y + d.y][temp->x + d.x].g)
                    {
                        map_matrix_A[temp->y + d.y][temp->x + d.x].g = temp->g + 14 * a_step * (point_type + density_map[temp->y + d.y][temp->x + d.x]);
                        map_matrix_A[temp->y + d.y][temp->x + d.x].parent = temp;
                    }
                }
                open_list.push_back(&map_matrix_A[temp->y + d.y][temp->x + d.x]);
            }
        }
    }
}
```



推出open list, 标记这个点已经查过

```c++
open_list.remove(temp);
temp->check = 1; // 已经检查过这个点
```



超时判断

```c++
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
```



到此处表示已经搜索完成



推入路径

```c++
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
```







## 密度图



### 初始化

````c++
vector<vector<int>> density_map; // 人群密度图,给A*考虑选取人少的路径

for (int i = 0; i < row_num; i++)
{
    vector<int> temp;
    for (int j = 0; j < col_num; j++)
    {
        temp.push_back(0);
    }
    density_map.push_back(temp);
}
````



### 更新

```c++
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
					density_map[int(agent_list[i].y * map_factor+j)][int(agent_list[i].x * map_factor+k)] += 10;
				}
			}
		}
		
		
	}
}
```



### 使用

```c++
map_matrix_A[temp->y + d.y][temp->x + d.x].g = temp->g + 10 * a_step*(point_type+ density_map[temp->y + d.y][temp->x + d.x]);
```







## 问题与改进



### 拐角卡住✔

门口增加标识，防止跳过重要路径点



原SFM障碍物作用力的方向是否有问题，垂直于墙面所在直线的力会导致agent在拐角处被卡住。在直线外的情况应该考虑方向改为与障碍物最近端点的相对方向。✔



### 卡入墙体✔

tick 太高了，agent会被推到障碍物里（agent在离障碍物还有一段距离时，计算的速度乘上tick，已经会导致agent进入障碍物，可能需要优化tick或者障碍物力的参数）❌



可能是下一个目标点在墙对面的关系❌



障碍物body force 不够大❌



A⭐算法贴墙太近了会容易出现这个问题❌

①增加靠近墙的点的cost

②增大A⭐搜索步长（没解决根本问题，但是提高了A⭐速度）



好像垂足求的有问题❌

原来是距离算错了，调整了垂足在赋值之前就调用了😅



### 生成在墙里✔

看着逻辑没问题啊怎么会生成agent在墙内呢

判断出问题了😅

output4之后

### 一部分agent会往原点走✔



可能是后面间隔push路径的问题



原来是数组越界,一天一个逆天错误治好我的低血压😅



### 出去和进来同一个门的agent互相卡住✔

设置一个离终点越近越小的tao来限制agent的驱动力，离终点远的agent驱动力更强更优先。✔

output4



#### 谦让一下

让部分agent判断拥挤的状态(配合jam和周围agent数量判断)

这个状态下,这部分agent会暂时的调大tao值,减小自己的驱动力,给其他agent让出道路



#### 加一个偏转向左的力❌

如果agent2对agent1的作用力方向在agent2的前进方向上，给agent1施加一个向左的力，让agent都尽量靠右行

之前用的直接把力方向转向v2对侧,效果较差

output8

试一下是偏转力更好，还是直接施加侧向力更好



#### 只施加向左的力可能导致本来在右边的被拉到中间对冲✔

不如直接在左边的往左推,在右边的往右推✔





### agent更倾向于走穿过房间的最近道路✔

A*算法让agent总是走最近路径，但是未考虑到人群密度。

效果不好,因为只在agent一个点加了密度指数,但是实际搜索可能跳过这个点,因为A⭐有步长,所以要给附近区域都加密度指数✔

output11





可以增加在门口增加判断区域，强制给终点不在房间但是要经过房间的agent修改路径。（类似*EVALUATION OF GUIDANCE SYSTEMS AT DYNAMIC PUBLIC TRANSPORT HUBS USING CROWD SIMULATION*里的方法，可以在这里引入指示牌可见性）



或者拓展map matrix，扩大参数范围，给房间内的块增加开销。✔

output9

比之前好了一点但是还是会出现拥挤，虽然不走房间里面不参与挤入房间的过程，但是最近的路径仍然在拥挤人群中，具体解决方案考虑第一点







### 通过提高A*步长来提速✔

想要实现这一点必须保证在拐角或门口等关键点添加标签, 使得这些位置的路径点不可被省略✔

并且agent视为到达目标点的距离也必须限制,防止在仍未达到关键点时识别为达到,推入下一个较远的目标点后卡住(目前就是限制在步长两倍比较好)





 ### Recognition-Primed Decision❗



### OpenMP Parallel

byd并行内存爆了😭



### agent 在拥堵时重新寻路 导致模拟时间大幅增加

想想怎么优化





## 完整代码

7.27 9:40



### sfm.h

```c++
#include<math.h>
#include "stdio.h"
#include "stdlib.h"
#include "time.h"
#include<vector>
#include<iostream>
#include<list>

using namespace std;

#define MAX_AGENT_NUM 1000 // 最大agent数量，因为使用的是vector所以实际上没用到
#define MAX_OBLINE_NUM 1000 // 最大障碍物数量

#define MAX_V 1.02 // agent 最大速度
#define sense_range 5 // 感知范围：m
#define density 320 // 质量/半径，m/density = r 320
#define tao 0.5 // 目前方向转移到目标方向的加速度参数


#define k1 120000 // body force parameter
#define k2 240000 // tangential force parameter 切向力参数s
#define A 2000 // N// A * exp[dis/B] only social force
#define B 0.08 // m

int col_num = 0;
int row_num = 0;//init in init_map


double  map_factor = 10; // 点阵图放大比

vector<vector<int>> map_matrix; // 点阵图
vector<vector<int>> density_map; // 人群密度图,给A*考虑选取人少的路径


// 坐标结构体
struct cordinate {
	double x;
	double y;
	cordinate() = default;
	cordinate(double a, double b) {
		x = a;
		y = b;
	}
};


// agent 结构体
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

	list<cordinate> path; // A* 路径存储
};

// 障碍物结构体
struct OBLINE
{
	double sx; // start point
	double sy;
	double ex; // end point
	double ey;
	double len; // length

};

double randval(double, double);

// agents 之间距离
double agent_dis(AGENT, AGENT);

// agents 之间作用力
void agent_force(AGENT*, AGENT*, double*, double*,double);

// 计算点到线距离
double point_to_line_dis(double, double, double, double, double, double, double, double*, double*);

// 计算障碍物作用力
void obline_force(AGENT*, OBLINE*, double*, double*);



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


//----------------------------------以上为SFM部分-------------------------------------


void A_star(AGENT*);
bool in_map(int, int);

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

vector<vector<node>> map_matrix_A;
vector<cordinate> direction = { {a_step,0}, {-a_step,0}, {0,a_step}, {0,-a_step} };
vector<cordinate> ob_direction = { {a_step,a_step},{a_step,-a_step},{-a_step,-a_step},{-a_step,a_step} };//斜向


const int a_step = 3;
const int path_len = 2;
const int max_time = 800; // ms

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
		for (auto d : direction)
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
		}

		// 正方向上搜索,此时移动代价为10
		for (auto d : direction)
		{
			if (in_map(temp->x + d.x, temp->y + d.y))
			{
				int point_type = map_matrix[temp->y + d.y][temp->x + d.x];
				//cout << density_map[temp->y + d.y][temp->x + d.x] << endl;
				if (point_type != 0 && map_matrix_A[temp->y + d.y][temp->x + d.x].check == 0)//节点在地图内，且不在障碍物部分,且未check过
				{
					if (map_matrix_A[temp->y + d.y][temp->x + d.x].flag == 0)//表示该点第一次进入open_list
					{
						map_matrix_A[temp->y + d.y][temp->x + d.x].x = temp->x + d.x;
						map_matrix_A[temp->y + d.y][temp->x + d.x].y = temp->y + d.y;
						map_matrix_A[temp->y + d.y][temp->x + d.x].h = (abs(int(a->gx * map_factor) - (temp->x + d.x)) + abs(int(a->gy * map_factor) - (temp->y + d.y))) * 10;
						map_matrix_A[temp->y + d.y][temp->x + d.x].g = temp->g + 10 * a_step*(point_type+ density_map[temp->y + d.y][temp->x + d.x]);
						map_matrix_A[temp->y + d.y][temp->x + d.x].parent = temp;
						map_matrix_A[temp->y + d.y][temp->x + d.x].flag = 1;
					}
					else // 再次筛查一个点d,只有在通过(这次搜索的点temp到达d的代价)比(d的父节点到d)更小时,才会更新
					{
						if (temp->g + 10 * a_step < map_matrix_A[temp->y + d.y][temp->x + d.x].g)
						{
							map_matrix_A[temp->y + d.y][temp->x + d.x].g = temp->g + 10 * a_step * (point_type + density_map[temp->y + d.y][temp->x + d.x]);
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
					if (point_type != 0 && map_matrix_A[temp->y + d.y][temp->x + d.x].check == 0)//节点在地图内，且不在障碍物部分,且未check过 && map_matrix_A[temp->y + d.y][temp->x + d.x].check == 0
					{
						if (map_matrix_A[temp->y + d.y][temp->x + d.x].flag == 0) // 表示该点第一次进入open_list
						{
							map_matrix_A[temp->y + d.y][temp->x + d.x].x = temp->x + d.x;
							map_matrix_A[temp->y + d.y][temp->x + d.x].y = temp->y + d.y;
							map_matrix_A[temp->y + d.y][temp->x + d.x].h = (abs(int(a->gx * map_factor) - (temp->x + d.x)) + abs(int(a->gy * map_factor) - (temp->y + d.y))) * 10;
							map_matrix_A[temp->y + d.y][temp->x + d.x].g = temp->g + 14 * a_step * (point_type + density_map[temp->y + d.y][temp->x + d.x]);
							map_matrix_A[temp->y + d.y][temp->x + d.x].parent = temp;
							map_matrix_A[temp->y + d.y][temp->x + d.x].flag = 1;
						}
						else // 再次筛查一个点d,只有在通过(这次搜索的点temp到达d的代价)比(d的父节点到d)更小时,才会更新
						{
							if (temp->g + 14 * a_step < map_matrix_A[temp->y + d.y][temp->x + d.x].g)
							{
								map_matrix_A[temp->y + d.y][temp->x + d.x].g = temp->g + 14 * a_step * (point_type + density_map[temp->y + d.y][temp->x + d.x]);
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



```



### sfm.cpp

```c++
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

const int thread_num = 12;

int agent_num = 500;
int step_num = 5000;
double tick = 0.03;//timestep
int  obstical_line_num = 0;

clock_t total_start, total_end;

vector<AGENT> agent_list; // agent vector
struct OBLINE obstical_lines[MAX_OBLINE_NUM]; // obstical line array
FILE* f = fopen("output.txt", "w");
struct cordinate goal[6] = { {5,22},{33,22},{5,34},{33,34},{5,56},{33,56} };

int jam_time_threshole = 50;




void test();
void init_agent(int);
void step();
void output(AGENT*);//写出agent坐标
void init_obline(string);//障碍物读入
void init_map(string);
void init();
void update_density();


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


void step()
{
	fprintf(f, "%lu\n", agent_list.size());

	//单个测试用
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
		a->tao_1 = 0.2 + (goal_dis / a->dis) * 0.2;
		double dx = a->v0 * (a->next_gx - a->x) / goal_dis;//期望方向向量的x
		double dy = a->v0 * (a->next_gy - a->y) / goal_dis;
		
		double ax = (dx - a->vx) / a->tao_1;//加速度
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
			if (dis <= sense_range)//超出感知范围不计算
			{
				agent_counter++;
				agent_force(a, b, &total_force_x, &total_force_y,dis);
			}
		}

		if (a->jam_time >= jam_time_threshole && agent_counter>8)
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


		//cout << total_force_x << endl << total_force_y << endl;

		ax += total_force_x / a->m;//加速度加上社会力的影响
		ay += total_force_y / a->m;

		a->vx += ax * tick;
		a->vy += ay * tick;

		double new_v = sqrt(a->vx * a->vx + a->vy * a->vy);//更新后的速度大小
		if (new_v > a->v0)//超过最大速度后调整
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

		//单个测试用
		//if(!a->path.empty())
		//fprintf(f, "%d %g %g %d\n", a->id+1, a->path.front().x/map_factor, a->path.front().y/map_factor, 1);
	}
}



void output(AGENT* a)
{
	fprintf(f, "%d %g %g %d %d\n",a->id,a->x,a->y,a->np,a->color);
}


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


void init()
{
	init_obline("./map/obstacles4.txt");
	init_map("./map/matrix4.txt");
	cout << "row_num: " << row_num << " col_bnum: " << col_num << endl;
	init_agent(agent_num);
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
					density_map[int(agent_list[i].y * map_factor+j)][int(agent_list[i].x * map_factor+k)] += 10;
				}
			}
		}
		
		
	}
}




int main()
{
	total_start = clock();
	
	omp_set_num_threads(thread_num);

	init();

	//test();

	

	//初始状态输出
	fprintf(f, "%lu\n", agent_list.size());

	for (int i = 0;i<agent_num;++i)
	{
		output(&agent_list[i]);
	}
	int counter = 0;
	cout << "Initializing path" << endl;
	for (auto& a : agent_list)
	{
		A_star(&a);
		counter++;
		printf("A_star: %d / %d \r", counter, agent_num);
		/*for (int i = 0; i < a.path.size(); ++i)
		fprintf(f, "%d %g %g %d\n", a.id+i, a.x, a->y, a->np);*/
	}
	cout << endl;

	clock_t start, end;
	cout << "Start steps..." << endl;
	start = clock();
	for (int i = 0; i < step_num; ++i)
	{
		//fstream of("output.txt", ios::app | ios::out);
		//of << agent_list.size() << endl;
		
		step();
		
		if ((i+1) % 10 == 0)
		{
			update_density();
			end = clock();
			printf("Step: %d / %d ;Remaining time: %d s             \r", i + 1, step_num, (end - start) * (step_num - i - 1)/ 10 / 1000);
			start = clock();
		}
		
	}
	cout << endl;


	total_end = clock();
	cout << "Done " << step_num << " step(s). Total time :"<< (total_end-total_start)/1000/60<< " min(s)" << endl;


	//test();
}
```





