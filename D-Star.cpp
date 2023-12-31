#include <iostream>
#include <string>
#include <set>
#include <iomanip>
#include <fstream>
#include <map>
#include <cmath>
using namespace std;

#define INF 0x3f3f3f
const int maxn = 2500;
const int s_index = 0;
const int e_index = 2499;

#define OUTPUT_FILE "ouput_1.md"

// 定义遍历邻接节点的方向
int directions[8][2] = {
    {0, -1},  // 上
    {0, 1},   // 下
    {-1, 0},  // 左
    {1, 0},   // 右
    {-1, -1}, // 左上
    {1, -1},  // 右上
    {-1, 1},  // 左下
    {1, 1},   // 右下

};

// 定义节点的结构
struct Node
{
    // 当前节点的状态
    // 0 - new
    // 1 - open
    // -1 - closed
    int state;

    // 当前节点的坐标值
    int x;
    int y;

    // 当前节点的 h 值（也就是 dijkstra 中的 dis ）
    int h;

    // 当前节点的 k 值
    int k;

    // 当前节点指向的父节点
    int father;

    // 当前节点的下标
    int index;
};

// 节点列表
Node node[maxn];

// 地图信息
std::string mp[55];

// 地图维数
int n;

std::map<int, bool> isPath;

// OpenList & CloseList
std::set<Node *> OpenList;
std::set<Node *> CloseList;

// 初始化
void init()
{
    // n * n 大小的栅格化地图
    std::cin >> n;
    // 输入地图数据
    for (int i = 0; i < n; i++)
        std::cin >> mp[i];

    // 初始化节点数据
    int cnt = 0;
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            node[cnt].father = -1;
            node[cnt].h = INF;
            node[cnt].k = INF;
            node[cnt].state = 0;
            node[cnt].x = i;
            node[cnt].y = j;
            node[cnt].index = cnt;
            cnt++;
        }
    }
    // node[0].h = 0;
    // node[0].k = 0;

    node[99].h = 0;
    node[99].k = 0;
}

// 判断节点的合法性
bool isValidNode(int x, int y)
{
    if (x < 0 || x >= n || y < 0 || y >= n)
        return false;
    if (mp[x][y] == '#')
        return false;

    return true;
}

// 邻接边的路径成本
int cost(int i)
{
    return (i > 3 ? 14 : 10);
}

// 使用 dijkstra 规划最短路径
void dijkstra(int s, int e)
{
    // 将起点 S 放入开放列表
    OpenList.insert(&node[s]);
    node[s].state = 1;

    while (OpenList.size())
    {
        // 遍历 OpenList，找到 h 值最小的节点 n
        int min_h_value = INF;
        // 找到当前正在处理的节点的指针
        Node *N_ptr;
        for (std::set<Node *>::iterator i = OpenList.begin(); i != OpenList.end(); i++)
        {
            Node *nodePtr = *i;
            if (nodePtr->h < min_h_value)
            {
                min_h_value = nodePtr->h;
                N_ptr = nodePtr;
            }
        }

        // 遍历 N 的邻接节点
        for (int i = 0; i < 8; i++)
        {
            // 先判断当前节点是否存在（合法）
            int n_near_x = N_ptr->x + directions[i][0];
            int n_near_y = N_ptr->y + directions[i][1];
            if (!isValidNode(n_near_x, n_near_y))
                continue;

            Node *N_near_Ptr = &node[n_near_x * n + n_near_y];

            // 当前节点的状态为 new（没有访问过）
            if (N_near_Ptr->state == 0)
            {
                // 加入开放列表
                OpenList.insert(N_near_Ptr);
                N_near_Ptr->state = 1;
                // 更新 h 值
                N_near_Ptr->h = N_ptr->h + cost(i);
                // 将 n 设置为 n_near 的父结点
                N_near_Ptr->father = N_ptr->index;
            }
            // 当前节点的状态为 open（已经访问过但是还没有确定最短路径）
            else if (N_near_Ptr->state == 1)
            {
                double new_h = N_ptr->h + cost(i);
                if (N_near_Ptr->h > new_h)
                {
                    N_near_Ptr->h = new_h;
                    N_near_Ptr->father = N_ptr->index;
                }
            }
            // 当前节点处理完毕，移入CloseList
            N_ptr->state = -1;
            OpenList.erase(N_ptr);
            CloseList.insert(N_ptr);
            // 终点进入 CloseList 算法结束
            if (N_ptr->index == e)
                return;
        }
    }
}

void Insert(Node *x, int h_new)
{
    if (x->state == 0)
        x->k = h_new;
    else if (x->state == 1)
        x->k = std::min(x->k, h_new);
    else if (x->state == -1)
        x->k = std::min(x->h, h_new);
    x->h = h_new;
    x->state = 1;
    OpenList.insert(x);
}

// 获取 OpenList 中 K 值最小的节点
Node* Min_state()
{
    Node *X = *OpenList.begin();
    for (std::set<Node *>::iterator i = OpenList.begin(); i != OpenList.end(); i++)
    {
        Node *n = *i;
        if (n->k < X->k)
            X = n;
    }

    return X;
}



int process_state()
{
    Node *X = Min_state();
    if (X == NULL)
        return -1;
    int k_old = X->k;
    OpenList.erase(X);
    X->state = -1;
    
    // 在将路径成本变化传播到邻居节点之前，先遍历其成本最佳的邻居，看看是否可以减少 X.h
    if (k_old < X->h)
    {
        // cout<<"ERROR_1"<<endl;
        // cout<<"K-old:" <<k_old << " X.h : "<< X->h<<endl;
        // cout<<endl;
        // 遍历邻接节点
        for (int i = 0; i < 8; i++)
        {
            // 先判断当前节点是否存在（合法）
            int n_near_x = X->x + directions[i][0];
            int n_near_y = X->y + directions[i][1];
            if (!isValidNode(n_near_x, n_near_y))
                continue;

            // 找到当前合法的邻接节点 Y
            Node *Y = &node[n_near_x * n + n_near_y];

            if (Y->h < k_old && X->h > Y->h + cost(i))
            {
                X->father = Y->index;
                X->h = Y->h + cost(i);
            }
        }
    }

    if (X->h == k_old)
    {
        // 遍历邻接节点
        for (int i = 0; i < 8; i++)
        {
            // 先判断当前节点是否存在（合法）
            int n_near_x = X->x + directions[i][0];
            int n_near_y = X->y + directions[i][1];
            if (!isValidNode(n_near_x, n_near_y))
                continue;

            // 找到当前合法的邻接节点 Y
            Node *Y = &node[n_near_x * n + n_near_y];

            if (Y->state == 0 || (Y->father == X->index && Y->h != X->h + cost(i)) || (Y->father != X->index && Y->h > X->h + cost(i)))
            {

                if(Y->state != 0) std::cout<<"ERROR\n";

                Y->father = X->index;
                Insert(Y, X->h + cost(i));
            }
        }
    }
    else
    {
        for (int i = 0; i < 8; i++)
        {
            // 先判断当前节点是否存在（合法）
            int n_near_x = X->x + directions[i][0];
            int n_near_y = X->y + directions[i][1];
            if (!isValidNode(n_near_x, n_near_y))
                continue;

            // 找到当前合法的邻接节点 Y
            Node *Y = &node[n_near_x * n + n_near_y];

            if (Y->state == 0 || (Y->father == X->index && Y->h != X->h + cost(i)))
            {
                Y->father = X->index;
                Insert(Y, X->h + cost(i));
            }
            else
            {
                if (Y->father != X->index && Y->h > X->h + cost(i))
                    Insert(X, X->h);
                else
                {
                    if (Y->father != X->index && X->h > Y->h + cost(i) && Y->state == -1 && Y->h > k_old)
                    {
                        Insert(Y, Y->h);
                    }
                }
            }
        }
    }
    // 获取当前 Openlist 中最小的 K 值
    return Min_state()->k;
}



// 输出最终确定的最短路径
void outPut_Path()
{
    std::ofstream outputFile(OUTPUT_FILE, std::ios::app);

    int index = s_index;
    while (index != e_index)
    {
        isPath[index] = true;
        index = node[index].father;
    }

    outputFile << "## Path" << std::endl;
    outputFile << "```" << std::endl;

    int cnt = 0;
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            if (isPath[cnt])
                outputFile << "·" << ' ';
            else
                outputFile << mp[i][j] << ' ';
            cnt++;
        }
        outputFile << std::endl;
    }
    outputFile << "```" << std::endl;
}

void D_star()
{
    // 设置终点的 h 和 k 的值为 0
    node[e_index].h = 0;
    node[e_index].k = 0;
    // 将终点加入开放列表
    OpenList.insert(&node[e_index]);

    // 重复调用 Process-state
    int cnt =1;
    while (OpenList.size() && node[s_index].state != -1)
    {
        std::cout<<"No." <<cnt++<<": ";
        if(cnt == 2700) break;
        int kmin = process_state();
        std::cout<<kmin<< ' '<<OpenList.size()<<std::endl;

        for (std::set<Node *>::iterator i = OpenList.begin(); i != OpenList.end(); i++)
        {
            Node *n = *i;
            std::cout<< n->index <<" ";
        }
        std::cout<<'\n';
    }
}

// 输出 H 权值图
void outPut_H_graph()
{
    std::ofstream outputFile(OUTPUT_FILE, std::ios::app);
    outputFile << "## H-Graph" << std::endl;
    for (int i = -1; i < n; i++)
    {
        if (i >= 0)
            outputFile << "|" << i;
        else
            outputFile << "|/";
    }

    outputFile << "|" << std::endl;
    for (int i = -1; i < n; i++)
        outputFile << "|-";
    outputFile << "|" << std::endl;
    int cnt = 0;
    for (int i = 0; i < n; i++)
    {
        outputFile << "|" << i << "|";
        for (int j = 0; j < n; j++)
        {
            if (node[cnt].h == INF)
                outputFile << "inf" << '|';
            else
                outputFile << std::fixed << std::setprecision(1) << node[cnt].h << '|';
            cnt++;
        }
        outputFile << std::endl;
    }
    outputFile << std::endl;
}

void outPut()
{
    std::ofstream outputFile(OUTPUT_FILE);
    outputFile << "# OutPuts" << std::endl;
    outputFile << std::endl;
    // outPut_H_graph();
    outPut_Path();
}

int main()
{
    freopen("input2.txt", "r", stdin);
    init();
    // dijkstra(s_index, e_index);
    D_star();
    outPut();
}