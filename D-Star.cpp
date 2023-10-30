#include <iostream>
using namespace std;

const int maxn = 505;

// 定义节点的结构
struct Node
{
    // 当前节点的状态
    // 0 - new
    // 1 - open
    // -1 - close
    int state;

    // 当前节点的坐标值
    int x;
    int y;

    // 当前节点的 h 值
    int h;

    // 当前节点的 k 值
    int k;

    // 当前节点指向的父节点
    int father;
};

// 节点列表
Node node[maxn];


int main()
{
    cout << "done." << endl;
}