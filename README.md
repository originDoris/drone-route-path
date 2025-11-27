# drone-route-path
# 无人机路径规划算法库

有问题可以加我微信讨论

微信：origindoris

## 项目简介

本项目是一个专业的无人机航线规划算法库，主要用于无人机摄影测量任务的路径规划。项目包含两个核心算法模块：

- **RoutePathPlanner**: 基础路径规划算法，用于在多边形区域内规划单方向航线
- **ObliquePhotography5RouteAlgorithm**: 倾斜摄影5航线算法，用于多方向倾斜摄影任务的智能航线规划

## 核心功能

### 1. 基础路径规划 (RoutePathPlanner)

#### 主要特性
- **多边形区域航线规划**: 在任意多边形区域内生成平行航线
- **智能起始点处理**: 自动处理起始点不在多边形内的情况
- **多重覆盖优化**: 支持多次分块规划，提高区域覆盖率
- **性能优化**: 内置缓存机制和并行处理，提升计算效率
- **摄影测量计算**: 支持GSD、飞行高度、照片覆盖范围等专业计算

#### 核心算法
1. **平行航线生成**: 根据航线方向和间距生成平行航线
2. **航线优化**: 使用TSP算法优化航线顺序，最小化飞行距离
3. **覆盖率分析**: 智能检测未覆盖区域并生成补充航点
4. **几何计算**: 点在多边形判断、线段相交检测等几何算法

### 2. 倾斜摄影5航线算法 (ObliquePhotography5RouteAlgorithm)

#### 主要特性
- **智能航线数量优化**: 根据云台俯仰角自动判断所需航线数量
- **多方向覆盖**: 支持1-5个方向的航线规划
- **边缘扩展**: 自动扩展作业区域确保边缘完全覆盖
- **覆盖率评估**: 提供边缘覆盖率估算和优化建议

#### 智能优化策略
- **云台角度 < 15°**: 单方向垂直摄影
- **云台角度 15°-30°**: 3方向十字交叉摄影
- **云台角度 30°-45°**: 4方向正交摄影
- **云台角度 > 45°**: 完整5方向摄影

## 使用方法

### 1. 基础路径规划

```java
import com.tythin.route.path.RoutePathPlanner;
import com.tythin.route.path.LatLngPoint;

// 定义作业区域多边形
List<LatLngPoint> polygonPoints = Arrays.asList(
    new LatLngPoint(39.9042, 116.4074),
    new LatLngPoint(39.9052, 116.4084),
    new LatLngPoint(39.9032, 116.4094),
    new LatLngPoint(39.9022, 116.4064)
);

// 设置规划参数
LatLngPoint startPoint = new LatLngPoint(39.9042, 116.4074);
double direction = 0;           // 航线方向（度）
double sideOverlapRate = 80;    // 旁路覆盖率（%）
double forwardOverlapRate = 70; // 路线覆盖率（%）
double photoWidth = 100;        // 照片宽度（米）
double photoLength = 80;        // 照片长度（米）
double flightHeight = 120;      // 飞行高度（米）

// 执行路径规划
RoutePathPlanner.RouteResult result = RoutePathPlanner.planRoute(
    polygonPoints, direction, startPoint,
    sideOverlapRate, forwardOverlapRate,
    photoWidth, photoLength, flightHeight
);

// 获取结果
List<LatLngPoint> waypoints = result.waypoints;  // 航点列表
double totalDistance = result.totalDistance;     // 总距离
int totalLines = result.totalLines;              // 航线数量
```

### 2. 多次分块规划（提高覆盖率）

```java
// 使用多次分块规划提高覆盖率
RoutePathPlanner.RouteResult result = RoutePathPlanner.planRouteWithMultipleBlocks(
    polygonPoints, direction, startPoint,
    sideOverlapRate, forwardOverlapRate,
    photoWidth, photoLength, flightHeight,
    10  // 最大分块数量
);
```

### 3. 倾斜摄影5航线规划

```java
import com.tythin.route.path.ObliquePhotography5RouteAlgorithm;
import com.tythin.route.path.ObliquePhotography5RouteAlgorithm.ObliqueParams;

// 设置倾斜摄影参数
ObliqueParams params = new ObliqueParams();
params.mainDirection = 0;           // 主航线方向（度）
params.sideOverlapRate = 80;        // 旁路覆盖率（%）
params.forwardOverlapRate = 70;     // 路线覆盖率（%）
params.photoWidth = 100;            // 照片宽度（米）
params.photoLength = 80;            // 照片长度（米）
params.gimbalPitch = -30;           // 云台俯仰角（度，负值向下）
params.flightHeight = 120;          // 飞行高度（米）
params.polygonPoints = polygonPoints; // 作业区域
params.startPoint = startPoint;     // 起始点

// 执行倾斜摄影航线规划
ObliquePhotography5RouteAlgorithm.ObliqueRouteResult result = 
    ObliquePhotography5RouteAlgorithm.planObliqueRoutes(params);

// 获取结果
List<RouteDirection> routes = result.routes;           // 各方向航线
double totalDistance = result.totalDistance;           // 总距离
int routeCount = result.totalRouteCount;              // 实际航线数量
boolean isOptimized = result.isOptimized;             // 是否优化
String reason = result.optimizationReason;            // 优化原因
double edgeCoverageRate = result.edgeCoverageRate;    // 边缘覆盖率
```

### 4. 摄影测量计算工具

```java
// 计算地面采样距离(GSD)
double gsd = RoutePathPlanner.calculateGSD(
    23.5,    // 传感器宽度（毫米）
    120,     // 飞行高度（米）
    24,      // 焦距（毫米）
    4000     // 图像宽度（像素）
);

// 计算飞行高度
double height = RoutePathPlanner.calculateFlightHeight(
    0.03,    // GSD（米）
    24,      // 焦距（毫米）
    4000,    // 图像宽度（像素）
    23.5     // 传感器宽度（毫米）
);

// 计算照片覆盖范围
double photoWidth = RoutePathPlanner.calculatePhotoWidth(
    23.5,    // 传感器宽度（毫米）
    120,     // 飞行高度（米）
    24       // 焦距（毫米）
);
```

## 算法原理

### 1. 基础路径规划算法

#### 核心步骤
1. **参数验证与预处理**
    - 验证多边形有效性（至少3个顶点）
    - 计算航线间距和航点间距
    - 处理起始点位置

2. **航线生成**
    - 根据航线方向生成平行线
    - 计算航线与多边形的交点
    - 生成航线内的航点序列

3. **路径优化**
    - 使用贪心算法优化航线顺序
    - 最小化航线间的连接距离
    - 验证航点间距合理性

4. **覆盖率优化**
    - 网格采样检测未覆盖区域
    - 生成补充航点
    - 多次迭代提高覆盖率

#### 关键算法
- **射线法**: 判断点是否在多边形内
- **线段相交**: 计算航线与多边形边界的交点
- **TSP优化**: 最小化航线间的飞行距离
- **自适应网格**: 根据区域面积动态调整采样密度

### 2. 倾斜摄影算法

#### 智能优化策略
倾斜摄影算法根据云台俯仰角度智能判断所需的航线数量：

- **垂直摄影模式** (角度 < 15°): 云台角度不足以产生有效的倾斜效果，采用单方向垂直摄影
- **十字交叉模式** (15° ≤ 角度 < 30°): 采用3个方向（0°、90°、180°）的十字交叉摄影
- **正交模式** (30° ≤ 角度 < 45°): 采用4个方向（0°、90°、180°、270°）的正交摄影
- **完整5方向模式** (角度 ≥ 45°): 采用标准的5方向摄影（0°、72°、144°、216°、288°）

#### 边缘扩展算法
1. **计算扩展距离**: 根据云台角度和飞行高度计算需要扩展的距离
2. **多边形扩展**: 使用法向量方法向外扩展多边形边界
3. **覆盖验证**: 确保扩展后的航线能完全覆盖原始区域

## 性能特性

### 优化机制
- **多级缓存**: 距离计算、几何判断、边界计算等结果缓存
- **并行处理**: 使用Java 8 Stream并行处理大量计算
- **智能采样**: 根据区域面积自适应调整网格采样密度
- **内存管理**: 自动清理缓存防止内存溢出

### 性能监控
```java
// 获取平均规划时间
double avgTime = RoutePathPlanner.getAveragePlanningTime();

// 重置性能统计
RoutePathPlanner.resetPerformanceStats();
```

## 技术特点

1. **高精度计算**: 使用球面几何算法，考虑地球曲率影响
2. **智能优化**: 多种优化策略，平衡计算效率和结果质量
3. **容错处理**: 完善的异常处理和边界情况处理
4. **可扩展性**: 模块化设计，易于扩展新的算法
5. **专业性**: 针对无人机摄影测量领域的专业算法

## 依赖要求

- Java 8+
- Lombok（用于数据类注解）

## 注意事项

1. **坐标系统**: 使用WGS84地理坐标系（经纬度）
2. **角度单位**: 所有角度参数使用度（°）为单位
3. **距离单位**: 所有距离参数使用米（m）为单位
4. **云台角度**: 俯仰角使用负值表示向下
5. **多边形方向**: 算法会自动处理多边形的顺时针/逆时针方向

## 应用场景

- 无人机航空摄影测量
- 倾斜摄影三维建模
- 农业植保作业规划
- 地形测绘任务规划
- 基础设施巡检路径规划
  
## 关于规划的航点过于密集的问题
```java
// 建议使用该方法进行规划
// 5航线算法是调用该方法的，会过滤掉冗余航点
com.origindoris.drone.RoutePathPlanner.planRouteWithMultipleBlocks 

// 如果使用 该方法需要手动打开
// 新增：航点优化 - 合并同一直线上的冗余航点
// List<LatLngPoint> finalOptimizedRoute = optimizeWaypoints(validatedRoute);
com.origindoris.drone.RoutePathPlanner.planRoute
```




## 关于规划的航点过于密集的问题
```java
// 建议使用该方法进行规划
// 5航线算法是调用该方法的，会过滤掉冗余航点
com.origindoris.drone.RoutePathPlanner.planRouteWithMultipleBlocks 

// 如果使用 该方法需要手动打开
// 新增：航点优化 - 合并同一直线上的冗余航点
// List<LatLngPoint> finalOptimizedRoute = optimizeWaypoints(validatedRoute);
com.origindoris.drone.RoutePathPlanner.planRoute
```

本算法库为无人机自动化作业提供了专业、高效、可靠的路径规划解决方案。


## Star History

[![Star History Chart](https://api.star-history.com/svg?repos=originDoris/drone-route-path&type=date&legend=top-left)](https://www.star-history.com/#originDoris/drone-route-path&type=date&legend=top-left)
