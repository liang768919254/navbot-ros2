# A.3 写模型 · URDF/xacro 建模模板

> 用途：写 URDF 时照着填的**骨架模板**。关键数值留 `___` 自己算，**不是成品**。
> 成品对照：`参考源码/urdf/navbot.xacro`（自己写不动了再看，别一上来就抄）
> 配套主文档：`P3_ROS2自主导航移动机器人_保姆级项目文档.md` 的阶段 A

---

## 一、坐标系约定（写死在纸上，动手前先定死）

| 约定 | 选择 |
|---|---|
| 前进方向 | **+X** |
| 左侧 | **+Y** |
| 上方 | **+Z** |
| 地面 | **z = 0**（所有高度从这里量） |

**两个必分清的 link**（本项目核心考点）：

| | `base_footprint` | `base_link` |
|---|---|---|
| 是什么 | 空 link，**地面投影点** | 底盘**几何中心**（离地约 5cm） |
| 有没有几何体 | 没有（就一个坐标系原点） | 有（box） |
| 谁用 | Nav2 的 AMCL / 代价地图 | 建模、可视化 |

> 为什么分两个：里程计描述「地面上的平面运动 (x,y,θ)」，用地面投影点做基准最干净；用 base_link 会掺进一个永远不变的高度 z。

---

## 二、三件事（整个 URDF 就是这三件事的重复）

| 概念 | 一句话 | 你写它时回答的问题 |
|---|---|---|
| **link** | 一个刚体 | 长什么样？多重？惯性多大？ |
| **joint** | 两个 link 的连接 | 谁连谁？固定还是能转？ |
| **origin** | 子相对父的位姿 | 位置在哪？转没转？ |

### ★ 五条铁律（本次实写踩坑总结，背下来）

1. **link 和 joint 是平级兄弟**，谁也不能嵌套谁——joint 天然横跨两个 link，不可能属于某一个。
2. **命名**：link 用 `_link` 结尾，joint 用 `_joint` 结尾，别混。
3. **origin 里 `xyz` = 位置，`rpy` = 旋转**，两者别串（把球心高度写进 rpy 是最常见的错）。
4. **位置写在哪**：`visual`/`collision` 的 origin = 几何体相对**自己 link 原点**的偏移（对称体就是 `0 0 0`）；`joint` 的 origin = 这个 link 相对**父 link** 的位置。
5. **宏的调用**：要「用」某个宏，在**体内**写 `<xacro:宏名 .../>`，不是塞进 `params`。`params` 里只放「这个宏自己接收的变量」。

---

## 三、link 的三块（能省吗）

| 块 | 给谁用 | 能省吗 |
|---|---|---|
| `<visual>` | RViz 显示（好看） | 能省 |
| `<collision>` | Gazebo 物理碰撞 | 仿真必需 |
| `<inertial>` | 物理引擎算质量/惯性 | 仿真必需，★ 不写会抖/莫名转 |

---

## 四、通用骨架（任何 URDF 都长这样）

```xml
<?xml version="1.0"?>
<robot name="xxx" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- ★ 平级排列，谁都不套谁 -->
  <link name="base_footprint"/>          <!-- 空 link，自闭合，一行就是完整的 -->

  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child  link="base_link"/>
    <origin xyz="0 0 ___" rpy="0 0 0"/>  <!-- ★ z 自己算 -->
  </joint>

  <link name="base_link">
    <visual>
      <geometry><box size="0.30 0.24 0.06"/></geometry>
    </visual>
    <collision>
      <geometry><box size="0.30 0.24 0.06"/></geometry>
    </collision>
    <xacro:box_inertia m="1.2" x="0.30" y="0.24" z="0.06"/>
  </link>

</robot>
```

---

## 五、三个惯性宏（公式封装，可直接用）

### `<inertial>` 通用结构（三个宏都长这样）

```xml
<inertial>
  <origin xyz="0 0 0" rpy="0 0 0"/>   <!-- 质心相对 link 原点，本模型都在原点 -->
  <mass value="..."/>                    <!-- 质量 kg -->
  <inertia ixx="..." ixy="0" ixz="0"
           iyy="..." iyz="0"
           izz="..."/>                   <!-- 6 个分量：对角 3 + 非对角 3 -->
</inertial>
```

> 对称几何体的非对角项（ixy/ixz/iyz）恒为 **0**。

### 公式表（背下来，面试也考）

| 几何体 | ixx | iyy | izz |
|---|---|---|---|
| 长方体 (m,x,y,z) | m(y²+z²)/12 | m(x²+z²)/12 | m(x²+y²)/12 |
| 圆柱 (m,r,h，轴沿Z) | m(3r²+h²)/12 | m(3r²+h²)/12 | m·r²/2 |
| 球 (m,r) | 2mr²/5 | 2mr²/5 | 2mr²/5 |

> ★ 长方体三轴**一般互不相等**（除非正方体）；圆柱绕 x、y 相等；球三向全等。别偷懒把长方体也写 ixx=iyy。

### 三个宏的代码

```xml
<xacro:macro name="box_inertia" params="m x y z">
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="${m}"/>
    <inertia ixx="${m*(y*y+z*z)/12}" ixy="0" ixz="0"
             iyy="${m*(x*x+z*z)/12}" iyz="0"
             izz="${m*(x*x+y*y)/12}"/>
  </inertial>
</xacro:macro>

<xacro:macro name="cyl_inertia" params="m r h">
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="${m}"/>
    <inertia ixx="${m*(3*r*r+h*h)/12}" ixy="0" ixz="0"
             iyy="${m*(3*r*r+h*h)/12}" iyz="0"
             izz="${m*r*r/2}"/>
  </inertial>
</xacro:macro>

<xacro:macro name="sphere_inertia" params="m r">
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="${m}"/>
    <inertia ixx="${2*m*r*r/5}" ixy="0" ixz="0"
             iyy="${2*m*r*r/5}" iyz="0"
             izz="${2*m*r*r/5}"/>
  </inertial>
</xacro:macro>
```

---

## 六、部件模板（关键 origin 留空，自己算）

### 6.1 底盘（base_link）

```xml
<joint name="base_joint" type="fixed">
  <parent link="base_footprint"/>
  <child  link="base_link"/>
  <origin xyz="0 0 ___" rpy="0 0 0"/>
</joint>
```

> 算 z：底面离地 `base_z` + 半厚度 `base_thick/2`。（本项目 = 0.02 + 0.03）

### 6.2 轮子（wheel 宏，左右只差符号）

```xml
<xacro:macro name="wheel" params="prefix side">
  <link name="${prefix}_wheel_link">
    <visual>
      <origin rpy="___ 0 0"/>       <!-- ★ 圆柱默认轴沿 Z，要横躺，绕 X 转多少？ -->
      <geometry><cylinder radius="0.0325" length="0.026"/></geometry>
    </visual>
    <collision>
      <origin rpy="___ 0 0"/>
      <geometry><cylinder radius="0.0325" length="0.026"/></geometry>
    </collision>
    <xacro:cyl_inertia m="0.15" r="0.0325" h="0.026"/>
  </link>

  <joint name="${prefix}_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child  link="${prefix}_wheel_link"/>
    <origin xyz="0 ${side * ___ / 2} ___" rpy="0 0 0"/>  <!-- ★ y、z 自己算 -->
    <axis xyz="0 1 0"/>
    <limit effort="5.0" velocity="20.0"/>
  </joint>
</xacro:macro>

<xacro:wheel prefix="left"  side="1"/>
<xacro:wheel prefix="right" side="-1"/>
```

> 算：半轮距 = 轮距/2；轮心相对 base_link 的 z = 轮半径 − 底盘中心离地。
> （本项目轮距 0.16，轮半径 0.0325，底盘中心离地 0.05）

### 6.3 万向球（caster 宏，fixed，只支撑不驱动）

```xml
<xacro:macro name="caster" params="prefix x_pos">
  <link name="${prefix}_caster_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>   <!-- 球心在 link 原点 -->
      <geometry><sphere radius="0.015"/></geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><sphere radius="0.015"/></geometry>
    </collision>
    <xacro:sphere_inertia m="0.03" r="0.015"/>
  </link>

  <joint name="${prefix}_caster_joint" type="fixed">
    <parent link="base_link"/>
    <child  link="${prefix}_caster_link"/>
    <origin xyz="${x_pos} 0 ___" rpy="0 0 0"/>  <!-- ★ 球心高 z 自己算 -->
  </joint>
</xacro:macro>

<xacro:caster prefix="front" x_pos="0.115"/>
<xacro:caster prefix="rear"  x_pos="-0.115"/>
```

> 算：球心相对 base_link 的 z = 球半径 − 底盘中心离地。（本项目 = 0.015 − 0.05）
> ★ fixed joint **不需要** `<axis>` 和 `<limit>`（那是 continuous/revolute 才有的）。

### 6.4 激光雷达（laser）

```xml
<link name="laser_link">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>   <!-- 圆柱关于自身中心对称 -->
    <geometry><cylinder radius="0.035" length="0.04"/></geometry>
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry><cylinder radius="0.035" length="0.04"/></geometry>
  </collision>
  <xacro:cyl_inertia m="0.10" r="0.035" h="0.04"/>
</link>

<joint name="laser_joint" type="fixed">
  <parent link="base_link"/>
  <child  link="laser_link"/>
  <origin xyz="0.04 0 ___" rpy="0 0 0"/>   <!-- ★ 高 z 自己算 -->
</joint>
```

> 算：底盘顶面 + 半雷达高。（本项目 = 0.03 + 0.02）
> ★★ link 名必须叫 `laser_link`，别叫 lidar——雷达驱动 `rplidar_ros` 默认 `frame_id` 是 `laser`，这里名字对不上会导致 TF 断链（见主文档 §R.6）。

---

## 七、验证命令（★ check_urdf 不吃 stdin）

```bash
# 展开成文件，再喂给 check_urdf（别用管道，会报 BrokenPipeError）
xacro navbot.xacro > /tmp/n.urdf && check_urdf /tmp/n.urdf
```

**目标输出**（6 步全写完应该是这样）：

```
root Link: base_footprint has 1 child(ren)
    child(1):  base_link
        child(1):  front_caster_link
        child(2):  laser_link
        child(3):  left_wheel_link
        child(4):  rear_caster_link
        child(5):  right_wheel_link
```

> `has N child(ren)` 里的 N = 有几个 joint 的 `<child>` 指向它。`has 0` = 没有任何 joint 挂到这个 link 上。

---

## 八、报错对照表

| 报错原文 | 根因 | 改法 |
|---|---|---|
| `Expect URDF xml file to parse` | check_urdf 没给文件参数（用了管道） | `xacro ... > /tmp/n.urdf && check_urdf /tmp/n.urdf` |
| `name 'xxx' is not defined` | 表达式里用了没定义的变量/属性名拼错 | 看 `when evaluating expression` 那一行 |
| `Invalid parameter "prefix"` | 宏参数写了 `$prefix`（ROS1 老写法） | 改成 `params="prefix side"` |
| `xxx is not unique` | 同一个名字定义了两次（复制粘贴忘了改名 / 宏里没拼变量） | 找哪个名字出现两次 |
| `Two root links found` | 有 link 忘了写 joint 连上（孤儿 link） | 看报错点名的 link，给它补 joint |

---

## 九、独立验收题（不看成品能写出来才算会）

> 把车「加长」：底盘从 `0.30×0.24` 改成 `0.40×0.24`，轮距从 `0.16` 改成 `0.20`。
> 要求四处同步改对：底盘 box 尺寸、base_joint 的 origin、轮子的 origin y、惯性张量。
> 判据：`check_urdf` 输出 `base_link has 5 child(ren)`，`xacro` 展开无错。

能不看笔记说出「改轮距时哪几个 origin 的 y 要跟着变、为什么惯性张量的 x²+y² 也要变」，就是真会了。

---

*本文档是主文档 `P3_ROS2自主导航移动机器人_保姆级项目文档.md` 阶段 A 的配套模板，2026-09-29 整理。*
