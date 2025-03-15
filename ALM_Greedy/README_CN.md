### 算法介绍
ALM - 贪心算法是一种基于路径的算法，专为解决容量受限交通分配问题（CTAP）而设计。即使在大规模网络中，该算法也能以令人满意的精度高效求解CTAP。这种高效性得益于两个特点。其一，它运用[贪心求解器](https://github.com/junxie016/open-TNM/tree/main/Greedy)来处理通过增广拉格朗日乘数法对偶化后的子问题。其二，它并非追求每个子问题的精确解，而是为子问题引入了自适应精度准则。这种方法在精度和效率之间取得了平衡。

### 算法运行方法
- 使用Visual Studio 2019打开“./ALM_Greedy/Source/TNA/TNA.sln”中的解决方案。
- 主函数位于“TNADriver.cpp”文件中。
- 修改“TestALM_Greedy”函数中的参数。
- 使用“Ctrl + F5”组合键编译并执行代码。

### 其他测试问题的运行方法
- 你可以从https://github.com/bstabler/TransportationNetworks 下载更多交通测试网络，并将其放置在“./ALM_Greedy/Network/”目录下。
- 由于bstabler的GitHub上的部分网络没有可行的CTAP解决方案，你应该增加路段容量或减少需求，以确保解集非空。
- 我们提供了CUP算法的实现（[Nie等人，2004年](https://doi.org/10.1016/S0191-2615(03)00010-9)）用于增加路段容量。你可以通过运行“TNADriver.cpp”中的“TestCUP”函数来尝试该算法，同时注释掉“TestALM_Greedy”函数。

### 引用方式
冯利阳、谢军、聂宇（马可）和刘晓波。用于容量受限交通分配的ALM - 贪心算法实现。https://github.com/junxie016/open-TNM/tree/main/ALM_Greedy 。访问时间：年/月/日。
Liyang Feng, Jun Xie, Yu (Marco) Nie and Xiaobo Liu. An implementation of the ALM-Greedy algorithm for capacitated traffic assignment. https://github.com/junxie016/open-TNM/tree/main/ALM_Greedy. Accessed Month, Day, Year.

### 出版物信息
Feng, L., Xie, J., Nie, Y., & Liu, X. (2020). 具有侧约束的交通分配问题的高效算法。《交通研究记录》，2674(4)，129 - 139。 