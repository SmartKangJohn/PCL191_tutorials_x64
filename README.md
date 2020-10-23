# PCL191_tutorials_x64
为了学习PCL，将PCL1.9.1的tutorials建为工程并解决其中的BUG问题 

注： 项目更新中...

## 安装包要求及环境：
1、win10+vs2017+pcl1.9.1_x64;

2、hdf5-1.12.0;

### project 为工程项目
### sources 为工程源码

## 操作说明
使用cmake3.15及以上版本，选择该项目的sources生成到project中即可。
camke会自动链接pcl的相关路径，只有hadf5的路径需要自己配一下。

第一次发开源，文档排版可能乱请见谅！
有问题请留issue，闲暇之余可能回复，哈哈哈哈！


# 工程介绍区
1.build_tree bug未解决

2.cloud_view
介绍了cloud_viewer的可视化操作；

3.cluster_extraction
介绍了聚类分割的操作；
代码中无可视化pcd， 故写了个函数在 一个window中分出多个viewer视窗来分别显示.pcd文件；

4.concatenate_clouds
介绍了点云的 点连接(+) 与 字段连接 的区别(concatenateFields)；

5.concatenate_fields
