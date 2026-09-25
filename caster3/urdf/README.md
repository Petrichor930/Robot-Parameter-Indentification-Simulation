<!-- 右上角可以预览markdown文档 -->
# 从 SolidWorks 导出 URDF 注意事项

1. `axis` 和定义的坐标旋转轴方向相同，一般都是 `<0 0 1>`。
2. 必须给 base 选择坐标系，一般是 `frame0`，不然模型初始姿态随机。
3. sdh最好导出 frame6
4. 导出时机械臂的姿态是零位，URDF 限位以此为 0 点。
5. 按照SDH的建系方法，frame0应该是静止不动的，frame1建立在link1的远端绕着z0转动（x0不动，x1绕z0转动得到theta1），urdf:parent连杆->joint关节->child连杆，child连杆跟随关节1转动，
6. 给坐标系正确命名：frame0,frame1,,,frame6;给link正确命名

---

# URDF 快速可视化检查

1. 在线查看器：<https://viewer.robotsfan.com/>

---

# URDF 转换为 .xml

如果你有 URDF 文件，可以直接用 MuJoCo 自带的工具转换。

1. 在 URDF 中添加：

   ```xml
   <mujoco>
       <compiler meshdir="../asserts/" balanceinertia="true" discardvisual="false"/>
   </mujoco>
   ```

2. 修改 STL 路径。

3. 进入 `mujoco/bin`：

   ```bash
   ./compile "your path.urdf" "your path.xml"
   ```

4. 可视化：

   ```bash
   source ~/robot_venv/bin/activate
   python3 -m mujoco.viewer --mjcf caster3/urdf/scene.xml
   ```

# 从 xml/urdf 中提取sdh参数表
1. 直接测量填表

| 连杆 $i$ | $\boldsymbol{\alpha_i}$(°) | $\boldsymbol{a_i}$(mm) | $\boldsymbol{d_i}$(mm) | $\boldsymbol{\theta_i}$ 初始(°) |
| :------: | :------------------------: | :--------------------: | :--------------------: | :-----------------------------: |
| 1        | 90                         | 0                      | 51                     | 0                               |
| 2        | 0                          | 140                    | 0                      | 0                               |
| 3        | -90                        | 55                     | -11.5                  | -90                             |
| 4        | 90                         | 0                      | 129.9                  | 0                               |
| 5        | -90                        | 0                      | 0                      | 0                               |
| 6        | 0                          | 0                      | 130.9                  | 0                               |

2. 使用工具转换，一般提取MDH，和sdh参数数值上是相等的，可以用来 验证手动测量的sdh参数表 或者 反向验证urdf

# 一些在线网站

1. <https://robosimtools.com>

# dh零位，关节电机零点，urdf导出时的零位（建立坐标系的初始姿态）

1. DH零位通常指的是在机器人运动学模型中，各关节角度参数（θ₁, θ₂…）全部为 0 时对应的机器人姿态。
2. 处于urdf导出时的零位，θ（1，2，4，5，6 = 0； 3 = -90）； 关节角度（-1.57；-3.14；-3.14；0；0；0）
