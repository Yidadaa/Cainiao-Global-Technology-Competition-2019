# Cainiao-Global-Technology-Competition-2019
The private repo for Cainiao Global Technology Competition 2019 vid HuaHuaGuai.

### 处理流程
现阶段决定选用[VINS Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)作为主要算法，在此基础上进行以下工作：
- [ ] 1. 手机端获取具有时间戳信息的视频和IMU数据
  - [ ] 1.1 录制视频以及IMU数据
  - [ ] 1.2 时间戳对齐（也可以放在服务端做）
- [ ] 2. 将手机获取的数据，发布为ROS Topic节点
  - [ ] 2.1 视频使用opencv发布订阅节点，参考[ROS&OpenCV进行摄像头数据的采集与订阅发布](https://blog.csdn.net/qq_31918961/article/details/80517547)
  - [ ] 2.2 IMU直接发布为订阅节点，参考[	
ros接入IMU数据，打包发布topic](https://blog.csdn.net/xinmei4275/article/details/85040164)
  - [ ] 2.3 配置ROS配置文件，根据节点生成轨迹以及点云图，参考文献待定；
- [ ] 3. 视频逐帧进行语义分割（若速度过慢，采用目标检测代替）
  - [ ] 3.1 使用deeplabv3训练语义分割网络
  - [ ] 3.2 融合点云图与语义分割信息，生成带有语义信息的点云图
- [ ] 4. 估计物体尺寸
  - [ ] 4.1 处理目标物体的点云图，生成最小外接立方体
  - [ ] 4.2 使用计数法调优立方体尺寸
  
#### 额外工作
- [ ] 编写手机app的界面
- [ ] 搭建服务端web架构，处理视频/IMU数据上传工作

### 文档资源
- [2019菜鸟全球科技挑战赛题目](https://tianchi.aliyun.com/competition/entrance/231706/information)
- [2019菜鸟全球科技挑战赛赛题解读](https://tianchi.aliyun.com/course/video?spm=5176.12586971.1001.1.5990489dTpbWni&liveId=41000)
- [深度干货：详解基于视觉+惯性传感器的空间定位方法 | 硬创公开课](https://www.leiphone.com/news/201610/taExbGMOaYfbnnMw.html)

### Paper List
- [CVPR18 - Learning Depth from Monocular Videos using Direct Methods](https://www.ci2cv.net/media/papers/Wang_Learning_Depth_From_CVPR_2018_paper.pdf)[[code]](https://github.com/MightyChaos/LKVOLearner)
- [CVPR18 - Unsupervised Learning of Depth and Ego-Motion from Monocular Video Using 3D Geometric Constraints](https://paperswithcode.com/paper/unsupervised-learning-of-depth-and-ego-motion-2)
- [2018 - Depth Prediction Without the Sensors: Leveraging Structure for Unsupervised
Learning from Monocular Videos](https://arxiv.org/pdf/1811.06152v1.pdf)
