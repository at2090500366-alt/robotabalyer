# 机器人末端轨迹规划与虚拟仿真实验 (Robot Trajectory Planning & Simulation)



## 🚀 实验一：六自由度机器人空间五角星轨迹规划

本实验通过建立机器人的 D-H 模型，实现了在指定空间平面内精确绘制五角星轨迹的完整流程 

### 1. 算法实现过程
**D-H 参数建模**：根据连杆长度、偏移和扭转角建立机器人连杆参数 。
**顶点坐标计算**：选用平面 $X=0.5$，圆心 $(0.5, 0, 0.2)$，半径 $0.2$，计算得到内切五角星的五个顶点坐标 。
* **笛卡尔空间插值**：
    * 设定五角星轨迹顺序为 `[1 3 5 2 4 1]` 。
    * 确保末端 approach 向量始终垂直于五角星平面（设置 $a = [1; 0; [cite_start]0]$）。
    * 使用 `ctraj` 函数在每两个顶点之间进行 50 步插值 。
* **逆运动学求解**：通过逆运算计算每个传递矩阵对应的关节角度、速度及加速度 。

### 2. 实验结果展示
#### 📸 空间路径与机器人位姿
<img width="413" height="413" alt="image" src="https://github.com/user-attachments/assets/24559e5e-d136-4c7f-85bf-3977e60ff8a5" />




https://github.com/user-attachments/assets/7bb7b827-f2bc-484e-b3ab-12323c1dc3b2







#### 📊 运动学时程曲线 (以 Joint 6 为例)
* **关节曲线**：展示了末端在执行轨迹时关节角的平滑过渡 。
* **速度/加速度曲线**：验证了插值算法下运动的连续性 。
<img width="430" height="236" alt="image" src="https://github.com/user-attachments/assets/fab10693-87ce-42b0-a68c-871c3c508933" />

<img width="430" height="236" alt="image" src="https://github.com/user-attachments/assets/a600089f-d327-4c6e-9514-1c5a93d122a2" />

<img width="430" height="236" alt="image" src="https://github.com/user-attachments/assets/ba51bd58-577d-4970-bb3d-a8f7d68b94e0" />





---

## 🤖 实验二：PUMA560 机器人虚拟仿真 (字符轨迹)

### 1. 平台选用方案
选用 **PUMA560** 机器人主要基于其经典性与结构优势：
* **结构灵活性**：六自由度旋转关节，0.8 米工作半径，±0.1 毫米重复定位精度 。
* **计算优势**：独特的**球形腕设计**（三个腕部关节轴线交于一点）极大简化了逆运动学求解过程 。
* **稳定性**：广阔的关节运动范围有效避免了奇异位形问题 。

### 2. 字符轨迹规划：点阵识别法
* **轮廓刻画**：使用包含 0 和 1 的矩阵刻画字符轮廓 。<img width="413" height="413" alt="image" src="https://github.com/user-attachments/assets/478748ac-fafc-42e9-b899-855bad843b64" />

* **路径优化**：通过识别矩阵中“1”的位置并映射至目标工作平面，采用**最近点优先排序算法**确定连线顺序 。
* **数据求解**：得到末端空间坐标后进行笛卡尔空间插值，并通过逆运算求解各关节数据 。

### 3. 仿真结果
* **工作空间验证**：通过点云图确认了字符轨迹完全处于 PUMA560 的工作包络线内 。<img width="413" height="413" alt="image" src="https://github.com/user-attachments/assets/70873951-2188-40be-855d-5cdec93f4b0d" />

* **字符实现**：成功在工作平面内刻画了“1005ZY.”字样的复杂点阵路径 。


https://github.com/user-attachments/assets/cc469cd1-2fb8-4537-b3ba-014ce047bab9






---

## 🛠 开发环境
* **编程语言**：Matlab 
* **辅助工具**：Robotics Toolbox, RoboAnalyzer, Virtual Robots 




