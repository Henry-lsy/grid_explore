## 策略
目前采用的GridSequence的策略是随机生成n组GridSequence，并不要求这些Sequence是连续的。然后采用动态规划去求解每一组的TSP问题。
后面要加入Sequence 连续的约束。

## 启动方式
roslaunch grid_explorer grid_explorer.launch


## TODO 
1. 增加GridSequence在rviz中可视化的功能。
2. 增加通过rviz进行鼠标选GridSequence的功能。
3. 增加测试用例。
*4. 改进算法，将生成的GridSequence变成相邻连续的。