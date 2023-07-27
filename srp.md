# SRP

[TOC]



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

agent给周围的密度图赋值后，导致A⭐搜索时间大幅增加。

和密度图更新关系不大



a_step从3调到5之后速度快了很多，效果没有太大影响✔

5再调高提速不明显，但是效果变差很多



判断周围人群数量, 之前是大于某个值就开始重新搜索, 现在也需要考虑小于某个值, 因为已经在人堆里面挣扎的agent这时候搜索就浪费时间, 不如直接挤挤算了✔

这个对提速帮助也不小





直接在A⭐中把密度图密度高的部分识别为墙体，跳过这部分的路径搜索。❌

这样直接把在人群中的agent判定在墙里卡死了



A⭐增加一个逃逸点，离这个点越近开销越高，把人群密度高的地方设置逃逸点





## 输出格式

output.txt

总帧数,放大倍数,宽,高

这一帧的agent数量

id,x坐标,y坐标,速度x方向,速度y方向

这一帧的agent数量

id,x坐标,y坐标,速度x方向,速度y方向



E.g

2,10,382,679

3

0,1,1,1,1

1,2,2,1,1

2,3,3,1,1

3

0,1,1,1,1

1,2,2,1,1

2,3,3,1,1





