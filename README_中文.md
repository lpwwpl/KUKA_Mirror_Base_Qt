# KUKA 凹非球面镜真实 BASE 与加工代码生成器（Qt 6 / C++17）

本项目对应“方法一”：轨迹点和姿态保留在镜面局部坐标系 M 中，由 KUKA 控制器通过 `$BASE = BASE_DATA[n]` 自动转换到机器人 WORLD。

适用对象：KUKA KR 70 R2100 等 KSS/KRL 六轴机器人；旋转对称凹非球面镜；五点法建立真实镜面 BASE。

## 1. 软件功能

1. 输入五个机器人 WORLD 实测点：`P0、PX+、PX-、PY+、PY-`。
2. 计算镜面局部 X、Y、Z 三轴在 WORLD 中的单位方向向量。
3. 计算 `R_WM、T_WM` 和 KUKA `BASE_DATA[n]` 的 `X,Y,Z,A,B,C`。
4. 对五点数据进行对称半径、中点横向偏差、矢高和旋转矩阵诊断。
5. 输入凹非球面参数 `D、R、k`。
6. 生成连续螺旋或往复光栅轨迹。
7. 逐点计算凹面开口法向和工具姿态 ABC。
8. 支持两种绕法向定向策略：
   - 固定镜面 X 轴投影：推荐，减少 A6 连续旋转；
   - 工具 X 轴跟随路径切向。
9. 输出：
   - `程序名.src`：KRL 主程序；
   - `程序名.dat`：局部 FRAME 轨迹点；
   - `程序名_trajectory.csv`：局部/WORLD 坐标校验；
   - `程序名_report.txt`：BASE、诊断和轨迹报告。

## 2. 坐标定义

- `P0`：凹面镜光学顶点，也是镜面 BASE 原点。
- `+Z_M`：由顶点指向凹面开口和刀具侧。
- `PX+、PX-`：相同半径处关于 P0 对称。
- `PY+、PY-`：相同半径处关于 P0 对称。
- 五点必须在同一个 TOOL、`$BASE=$WORLD` 条件下测量。

程序计算：

```text
X_M = normalize(PX+ - PX-)
Y_raw = PY+ - PY-
Y_M = normalize(Y_raw - dot(Y_raw, X_M) * X_M)
Z_M = normalize(X_M × Y_M)
R_WM = [X_M  Y_M  Z_M]
T_WM = P0
```

凹面开口方向通过两组对称点中点自动判断；若方向相反，程序同时翻转 Y、Z，以保持右手坐标系。

## 3. Qt 环境

推荐：

- Qt 6.5 或更高版本；
- Qt Creator，或 CMake 3.21+；
- C++17 编译器：MinGW-w64、MSVC 2022、GCC 或 Clang。

### Qt Creator 编译

1. Qt Creator 打开根目录的 `CMakeLists.txt`。
2. 选择已安装的 Qt 6 Desktop Kit。
3. Configure Project。
4. Build / Run。

### Windows + MinGW 命令行

```bat
set QT_DIR=C:\Qt\6.8.3\mingw_64
cmake -S . -B build -G Ninja -DCMAKE_PREFIX_PATH=%QT_DIR%
cmake --build build --config Release
```

### Windows + MSVC

在“x64 Native Tools Command Prompt for VS 2022”中：

```bat
set QT_DIR=C:\Qt\6.8.3\msvc2022_64
cmake -S . -B build -G "Ninja" -DCMAKE_PREFIX_PATH=%QT_DIR%
cmake --build build --config Release
```

### Linux

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH=/path/to/Qt/6.x/gcc_64
cmake --build build -j
```

## 4. 界面使用步骤

### 第一步：五点与 BASE

1. 将机器人设置为测量 TOOL。
2. 设置 `$BASE=$WORLD`。
3. 依次接触并记录 `P0、PX+、PX-、PY+、PY-` 的 WORLD XYZ。
4. 在第一页输入，或导入 CSV。
5. 单击“计算真实 BASE”。
6. 检查：
   - `det(R_WM)` 接近 1；
   - 三个轴点积接近 0；
   - PX 和 PY 对称半径差符合测量公差；
   - X/Y 中点横向误差符合测量公差。
7. 将结果输入 smartPAD 的 `BASE_DATA[n]`。

CSV 格式：

```csv
name,x,y,z
P0,1440,-70,520
PX+,...
PX-,...
PY+,...
PY-,...
```

### 第二步：凹面和轨迹

输入：

- 直径 D；
- 曲率半径 R；
- 二次项系数 k；
- 边缘余量；
- 螺距/光栅间距；
- 离散点间距；
- 法向偏置；
- 接近和退刀距离；
- TOOL +Z 的实际定义；
- 绕法向定向策略；
- KRL 程序名、TOOL_DATA、BASE_DATA、速度和逼近参数。

单击“计算轨迹并生成 KRL”。

### 第三步：输出

在第三页检查：

- SRC 主程序；
- DAT 点数据；
- CSV 中局部坐标和 WORLD 坐标；
- 计算报告和警告。

单击“导出 SRC / DAT / CSV / 报告”。

## 5. 输出 KRL 的重要含义

生成的点均为镜面局部坐标：

```krl
$TOOL = TOOL_DATA[1]
$BASE = BASE_DATA[5]
LIN XP00001
LIN XP00002 C_DIS
```

不得再给轨迹 ABC 手动叠加 `B=-20.8°`，否则会重复旋转。

默认只在 SRC 注释中给出计算出的 `BASE_DATA[n]`。只有勾选“运行时写入 BASE_DATA”时，才生成：

```krl
BASE_DATA[5] = {X ...,Y ...,Z ...,A ...,B ...,C ...}
```

现场更推荐先在 smartPAD 中核对并写入 BASE，而不是由加工程序自动覆盖。

## 6. TOOL +Z 选项

- “TOOL +Z 指向镜面内部/切削方向”：`Z_T = -n_open`。
- “TOOL +Z 指向凹面开口/远离镜面”：`Z_T = +n_open`。

必须根据真实 TOOL 坐标系选择，不能只看法兰外形。

## 7. 安全限制

生成器不计算机器人逆运动学关节构型，也不提供 S/T、碰撞模型或安全 HOME 点。程序上机前必须：

1. 在 KUKA.Sim 或等效离线软件中验证；
2. 检查 A5 是否接近 0°；
3. 检查 A4/A6 是否翻转；
4. 检查关节限位、软限位和镜架碰撞；
5. 手动示教安全接近位；
6. T1、低倍率、空运行；
7. 确认 TOOL 轴与实际主轴轴线一致后才加工。

## 8. 目录

```text
KUKA_Mirror_Base_Qt/
├─ CMakeLists.txt
├─ README_中文.md
├─ src/
│  ├─ main.cpp
│  ├─ mainwindow.h/.cpp
│  ├─ pathpreview.h/.cpp
│  └─ mirrorcore.h/.cpp
├─ sample/
│  ├─ five_points_example.csv
│  ├─ MIRROR01.src
│  ├─ MIRROR01.dat
│  ├─ MIRROR01_trajectory.csv
│  └─ MIRROR01_report.txt
└─ tests/
   ├─ core_test.cpp
   └─ generate_sample.cpp
```

核心数学模块 `mirrorcore.cpp` 不依赖 Qt，可以单独用普通 C++17 编译测试。
