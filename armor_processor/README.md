# armor_processor

- [armor_processor](#armor_processor)
  - [ArmorProcessorNode](#armorprocessornode)
  - [Tracker](#tracker)
  - [KalmanFilter](#kalmanfilter)

## ArmorProcessorNode
装甲板处理节点

装甲板处理节点订阅识别节点发布的装甲板目标及机器人的坐标转换信息，将装甲板目标通过 `tf` 其变换到世界坐标系下，然后将目标送入跟踪器中得到最终目标在世界坐标系下的位置及速度并发布出来。

包含[Tracker](#tracker)

订阅：
- 已识别到的装甲板 `/detector/armors`
- 机器人的坐标转换信息
  - `/tf`
  - `/tf_static`

发布：
- 最终锁定的目标 `/processor/target`

参数：
- 跟踪器参数 tracker
  - 两帧间目标可匹配的最大距离 max_match_distance
  - `DETECTING` 状态进入 `TRACKING` 状态的阈值 tracking_threshold
  - `TRACKING` 状态进入 `NO_FOUND` 状态的阈值 lost_threshold

## Tracker
跟踪器

跟踪器共有四个状态：
- `NO_FOUND` 目标未识别：跟踪器完全丢失目标
- `DETECTING` 目标识别中：短暂识别到目标，需要更多帧识别信息才能进入跟踪状态
- `TRACKING` 目标跟踪中：跟踪器正常跟踪目标中
- `LOST` 目标丢失：跟踪器短暂丢失目标，通过卡尔曼滤波器预测目标

工作流程：

- 初始化：

  跟踪器默认选择离相机光心最近的目标作为跟踪对象，这种方式比较贴近FPS游戏中辅助瞄准的手感。选择目标后初始化卡尔曼滤波器，初始状态设为当前目标位置，速度都设为 0。

- 更新:

  首先由跟踪目标的卡尔曼滤波器得到目标在当前帧的预测位置，然后遍历当前帧中的识别目标对预测位置进行匹配，若未识别到目标或所有目标与预测位置的偏差都过大则认为目标丢失。最后选取位置相差最小的目标作为最佳匹配项，更新卡尔曼滤波器，将更新后的状态作为跟踪器的结果输出。

- 目标在小陀螺状态下的 trick：

  若当前帧的所有目标与预测位置的偏差都不符合条件，但又存在与跟踪目标相同的数字，则直接选取相同数字的目标作为跟踪目标。通过这种方式，卡尔曼滤波器的状态向量中速度不会从 0 开始迭代，而是从上一帧的目标速度继续迭代。考虑到小陀螺状态下敌方机器人出现在视野内的装甲板速度相近，所以这种方式能够较好的持续估计出目标小陀螺状态下的速度。

包含[KalmanFilter](#kalmanfilter)

## KalmanFilter
卡尔曼滤波器
