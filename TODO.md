## 0401
-[ ] 通过点云信息校正矿石厚度引起的位置偏差
-[ ] 求体积为什么lidar比mask法大？（应该还是未对齐导致的，mask法会导致矿石边缘被切掉）
## 0331
- [x] 通过高低能分别找矿石的质心来求几何校正系数
-[ ] 仿射变换 (cv::warpAffine)
## 0324
- [ ] 耗时详细比较
- [x] 单块矿石的体积（与排水法求得的体积进行对比）

## 0313
- [x] 指定results的保存路径并且自动创建，而不需要手动创建
- [x] detectByLidar的存图逻辑是啥
- [x] high and low output 几何畸变

## 0306
- [x] lidar crop up and thickness top?
- [x] resize point cloud data?
## 03-03
- [ ] 复杂度分析
- [ ] 图片用异步保存？
- [x] lidar crop units? 
## 02-26
- [x] optimize: detectByLidar
- [x] implement: detect from XRT roi or mask

## 2026-01-30

- [ ] gather output to run_logs

## 2026-01-28

- [x] unit scale 
- [x] thickness to pixel value mapping
