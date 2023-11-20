# D*

## 算法特点

动态路径规划（Dynamic A*），适用于动态环境中的路径规划

A*、Dijkstra 等则适用于静态环境中的路径规划

D* 算法是一种增量路径搜索算法，它允许在搜索过程中不断适应环境的变化。它可以在遇到新障碍物或环境变化时，重新计算最短路径而无需从头开始搜索。

因为D\*算法有上述的特性，所以D\*算法可以使用在“无先验地图信息/先验地图信息不多的环境中的导航”的问题，因为只需要在最开始假装整个地图没有任何障碍，起点到终点的路径就是一条直线，然后再在在线运行时不断使用D\*算法重新规划即可。

- Compare to A*

  A* 算法是从起点向目标点进行搜索，而 D* 算法在预搜索阶段会先从目标点向起点进行搜索。

  

### 适用场景

适用于需要实时路径规划的场景，如自动驾驶、机器人导航等。它可以在动态环境中动态地调整路径以避免障碍物。

## 参考资料

- [wikipedia](https://en.wikipedia.org/wiki/D*)
- [算法动态演示](https://www.youtube.com/watch?v=e_7bSKXHvOI)
- [博客-D*算法超详解](https://blog.csdn.net/rezrezre/article/details/131008284)



## 重要定义

- OpenList

​	维护一个需要进行评估的节点列表

- G 

  表示路径搜索的目标点

  

- C(x,y)

  表示从节点 x 移动到节点 y 的代价

  

- t(x) 节点的状态 state

  - new
  - open
  - closed
  - raise  - 成本高于上次出现在 openlist 中
  - lower - 成本低于上次出现在 openlist 中

- h(x)

  表示地图上的点x到达目标点G的代价

- k(x)

  节点 x 最小的 h(x) 值

- b(x)

  用于记录当前节点x的父节点



- 主要函数

  - Process-State()

    > 计算到目标G的最优路径

  - Modify-Cost(x,y,val)

    >用于改变两个节点（state）之间的开销*C（X,Y）* 并将受影响的节点（state）置于Openlist中

  - insert(x,val)

    >修改节点x的状态以及h(x)值和k(x)值

## 算法过程

1. 初始化

   使用 dijkstra算法 从终点开始反向搜索初始地图，得到从地图上所有点到终点的最短距离 h。

   > [为什么初始化搜索阶段不使用搜索效率更高的 A* ?](###1 初始化搜索阶段为什么不使用 A*?)

2. 重规划

   第一步的初始化搜索完成之后，我们可以得到一条从起点到终点的最短路径。此时，机器人就会沿着这一条路径移动，当移动到路径上的某一个点，发现存在障碍物时（初始地图发生变化），就会触发**重规划**操作。

   ### Process State

   1. 在 `openlist` 中找到 `K` 值最小的节点 `X`

   2. 判断 节点 `X` 的 `K` 和 `H` 是否相等

      1. 若 `K < H` 则表示 `X` 受到新增障碍物的影响，处于 `raise` 状态

         遍历 `X` 的邻接节点，看是否可以通过将某个邻接节点作为 `X` 的父结点来是 `X.H` 减小

    3. 若 `K = H` 则表示 `X` 没有受到新增障碍物的影响(或者已经被上一步操作修复，回到了最佳状态)，处于 `lower` 状态

       遍历 `X` 的邻接节点，看看是否有邻接节点 `Y` 需要以 `X` 作为父结点

        1. 如果 `Y.state  = new` , 即 `Y` 还没有访问过

           或者 `Y.b = X && Y.h != X.h+C(X,Y)` 

           或者 `Y.b != X && Y.h > X.h+C(X,Y)` 则:

               1. 将 `X` 作为 `Y` 的父结点;
               2. 更新 `Y.h = X.h+C(X,Y) `
               3. 将 `Y` 加入开放列表

    4. 若 `K < H` 表示仍然处于 `raise` 状态，需要进一步优化

       遍历 X 的邻接节点 Y

       1. 如果 `Y.state = new `

          或者 `Y.b = X && Y.h != X.h+C(X,Y)` 则：

              1. 将 `X` 作为 `Y` 的父结点;
              2. 更新 `Y.h = X.h+C(X,Y) `
              3. 将 `Y` 加入开放列表

       2. 如果`Y.b != X && Y.h > X.h+C(X,Y)` 表示Y 可以通过将父结点改为 `X` 使得 `Y.h`的值更小，但是由于 X 自身仍然处于 `raise` 状态，所以要先将 `X` 再次追加到 Openlist，待下一次循环中 X 回归lower 状态之后再处理。

       3. 如果`Y.b != X && X.h > Y.h+C(X,Y) &&Y.state = closed && Y.h > K` 表示 `X` 不是 `Y` 的父结点，而如果让 `Y` 成为 `X` 的父结点可以使得 `X.h` 更小, 而 `Y` 已经从 `openlist` 中移出。然而当前从 `openlist` 中取出的节点 `K` 值比 `Y.h` 还要小，这就表示已经移出的 `Y` 节点受到了障碍物的影响导致 `H` 值升高，所以要将 `Y` 重新加入 `openlist` 来处理。

          ​	

------



## 论文原文中关于 算法的描述 [ 2.2 Algorithm Description] 

首先说明了 D* 算法主要包含两个函数 ：$Process-State$ 和 $Modify-Cost$.

- Process-State()

  > 计算到目标G的最优路径

- Modify-Cost(x,y,val)

  >用于改变两个节点（state）之间的开销*C（X,Y）* 并将受影响的节点（state）置于Openlist中



算法流程：

1. 首先将所有的节点的 state 值设置为 new，把终点 G 的 H 值设置为 0，然后把终点 G 加入 OpenList。

2. 重复调用 Process-state 函数， 直到机器人当前所处的位置 robot’s state（初始状态下就是起点）X 从Openlist 中移出，表示成功找到了一条从终点到起点的路径，或者程序返回-1表示没有找到路径。

3. 然后机器人会利用父结点指针来遍历这条路经（沿着这条路径移动），直到他达到终点，或者说在半路遇到了新发现的障碍。

4. 若机器人在半路上遇到了新发现的障碍，此时就会调用 Modify-Cost 函数来修正 C(Y,N) - 将当前节点到新发现障碍节点之间的路径权重修改为 INF，同时将受到新的障碍物影响的节点移动到 OpenList。

5. 我们假设机器人在发现新的障碍物时所处的节点是 Y ，接下来重复调用   Process-State 直到 函数返回的 $k_{min} >= Y.h$ ,即当前开放列表中最小的 K 值也比 Y.h 要大或者相等，新障碍物造成的路径成本变化传播到 Y ，使得 $Y.h = O(Y)$ 。

   > 这里的 O（Y）是什么意思没搞懂，应该是使得 Y.h 达到最佳状态的意思吧，这个 O 应该是 optimal

6. 此时，一条新的路径已经被构造出来，机器人将会沿着新的路径继续行走。



process state 中几个函数的解释：

- $GET-MIN$

  - ```c++
    get_min()
    {
      return x from openlist which x.k is smallest
    }
    ```

  - 

- $DELETE(X)$

  - ```c++
    delete(x)
    {
      openlist.erase(x);
      x.t = closed;
    }
    ```

  - 

- $INSERT(X,h_{new})$

  - ```c++
    insert(x,h_new)
    {
      if(x.t == new) x.k = h_new;
      else if(x.t == open) x.k = min(x.k,h_new);
      else if(x.t == closed) x.k = min(x.h,h_new);
      x.h = h_new;
      x.t = open;
      openlist.insert(x);
    }
    ```

    

接着解释了 process-state 函数的具体实现：

1. ![image-20231116125737913](/Users/lihaji/Documents/HajiNote/路径规划/assets/D*.assets/image-20231116125737913.png)

   L1 - L3 ： 将 openlist 中 K 值最小的节点 X 取出。

2. ![image-20231116130432537](/Users/lihaji/Documents/HajiNote/路径规划/assets/D*.assets/image-20231116130432537-0111074.png)

   L8 - L13 ：判断 X 的 K 和 H 值，如果说 X 处于 lower 状态，也就是说 X.h = X.k，那么这个 X 节点到终点的路径成本是最优的。

   然后检查 X 的每一个邻居节点 Y，看看这些邻居节点的路径成本能否降低。

   具体来说：

   ​	如果 Y 节点是一个没有被访问过的节点，那么 Y 的 父结点会变成 X，而且调用 Insert 函数更新 Y 的 H 值并且将其加入开放列表。

   ​	如果 Y 节点的父结点是 X，那么X 的路径成本会传播到 Y，而不管新的成本是否大于或小于旧的成本。

   ​	如果 Y 节点的父结点原来不是 X，但是如果把 X 作为父结点，可以获得更低的路径成本，那么就把父结点更新为 X。

3. ![image-20231117131553414](/Users/lihaji/Documents/HajiNote/路径规划/assets/D*.assets/image-20231117131553414.png)

​	L4 - L7 : 如果一个节点处于 raise 状态的话，那么他当前到终点的路径节点可能不是最优的，在将路径成本变化传播到邻居节点	之前，先遍历其成本最佳的邻居，看看是否可以减少 X.h。

4. ![image-20231118095205959](/Users/lihaji/Documents/HajiNote/路径规划/assets/D*.assets/image-20231118095205959.png)

​	L15 - L18:    路径成本成本更改以与 LOWER 节点相同的方式（2）传播到 NEW 节点和直接后代。

5. ![image-20231118100024467](/Users/lihaji/Documents/HajiNote/路径规划/assets/D*.assets/image-20231118100024467.png)

   L20 - L21：如果X能够降低非子孙的节点的路径成本，则 X 被重新加入Openlist，以便未来的扩展。（因为此时 X 节点仍然处于 Raise 状态，没有达到最佳状态）

6. ![image-20231118103316220](/Users/lihaji/Documents/HajiNote/路径规划/assets/D*.assets/image-20231118103316220.png)

   L23 - L25：如果X的路径成本能够被一个 次优 邻居降低 ， 则将这个 次优 的邻居加入开放列表，更新被 "推迟"，直到这个邻居有一个最优的路径成本。

7. 最后函数会返回 当前开放列表中最小的 K 值。

​	



## Hint

### 1 初始化搜索阶段为什么不使用 A*?

这是D\*的一个核心问题。其实也可以在第一次搜索中加入启发函数，让第一次快一点，但是D\*的核心是保证在全过程中，出现新发现的障碍物时也能很快地找到解，所以第一次搜索的范围其实尽可能的大会好一点。
如果在第一次搜索加入启发函数，确实第一次搜索会快很多，因为搜索范围小了。但是在后面发现新障碍物的时候，因为第一次的搜索范围比较小，很可能这个新障碍物导致的重新搜索，需要重新搜索一些第一次没搜索到的空间，因此这些点依然会被加入到搜索队列中。
一言概之就是说，启发式搜索减少的第一次搜索的点，终归会在后面的搜索中加入到搜索队列中，出来混迟早是要还的，所以实际上并不能很好地提高效率。
这样做可能让整体来说的效率变高，但是方法是减少第一次的搜索范围，而增加后续的搜索范围，而D*因为更多强调实时性，而第一次搜索是可以离线运行，但后续的搜索一定是在线运行的，增加后续的搜索范围会使实时性降低，这反而是不能接受的，因此尽管整体（也即第一次+后续搜索）的效率提升，但后续搜索的效率降低，得不偿失。