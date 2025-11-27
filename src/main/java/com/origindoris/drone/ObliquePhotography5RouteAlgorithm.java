package com.origindoris.drone;

import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.NoArgsConstructor;

import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.stream.Collectors;

/**
 * 倾斜摄影5航线算法类
 * 用于规划倾斜摄影任务的多方向航线，根据云台俯仰角度智能判断所需航线数量
 * @author xhz
 */
public class ObliquePhotography5RouteAlgorithm {

    /**
     * 倾斜摄影航线规划结果
     */
    @Data
    @AllArgsConstructor
    @NoArgsConstructor
    public static class ObliqueRouteResult {
        public List<RouteDirection> routes;  // 各方向航线列表
        public double totalDistance;        // 总距离（米）
        public int totalRouteCount;         // 实际航线数量
        public boolean isOptimized;         // 是否为优化后的航线（少于5条）
        public String optimizationReason;   // 优化原因说明
        public List<LatLngPoint> expandedPolygon; // 扩展后的作业区域
        public double expansionDistance;    // 扩展距离（米）
        public double edgeCoverageRate;     // 边缘覆盖率估算
    }

    /**
     * 单个方向的航线
     */
    @Data
    @AllArgsConstructor
    @NoArgsConstructor
    public static class RouteDirection {
        public double direction;             // 航线方向（度）
        public double gimbalPitch;          // 云台俯仰角（度）
        public List<LatLngPoint> waypoints; // 航点列表
        public double distance;             // 该方向航线总距离
        public String description;          // 航线描述
    }

    /**
     * 倾斜摄影参数
     */
    @Data
    @AllArgsConstructor
    @NoArgsConstructor
    public static class ObliqueParams {
        public double mainDirection;        // 主航线方向（0-360度）
        public double sideOverlapRate;      // 旁路覆盖率（0-100%）
        public double forwardOverlapRate;   // 路线覆盖率（0-100%）
        public double photoWidth;          // 照片宽度（米）
        public double photoLength;         // 照片长度（米）
        public double gimbalPitch;         // 云台俯仰角（度，负值表示向下）
        public double flightHeight;        // 飞行高度（米）
        public List<LatLngPoint> polygonPoints; // 作业区域多边形顶点
        public LatLngPoint startPoint;      // 起始点
    }

    private static final double EARTH_RADIUS = 6371000; // 地球半径（米）
    private static final double[] STANDARD_5_DIRECTIONS = {0, 72, 144, 216, 288}; // 标准5方向（度）
    private static final double MIN_EFFECTIVE_PITCH = -15.0; // 最小有效俯仰角
    private static final double MAX_EFFECTIVE_PITCH = -60.0; // 最大有效俯仰角
    
    // 性能优化：缓存和预计算
    private static final Map<String, Double> DISTANCE_CACHE = new ConcurrentHashMap<>();
    private static final Map<String, Double> COVERAGE_RADIUS_CACHE = new ConcurrentHashMap<>();
    private static final double[] COS_TABLE = new double[361];
    private static final double[] SIN_TABLE = new double[361];
    
    // 对象池优化
    private static final ThreadLocal<Queue<LatLngPoint>> POINT_POOL = 
        ThreadLocal.withInitial(() -> new ArrayDeque<>());
    
    static {
        // 预计算三角函数表
        for (int i = 0; i <= 360; i++) {
            double rad = Math.toRadians(i);
            COS_TABLE[i] = Math.cos(rad);
            SIN_TABLE[i] = Math.sin(rad);
        }
    }
    
    /**
     * 空间网格索引类，用于优化邻近查询
     */
    private static class SpatialGrid {
        private final double cellSize;
        private final Map<String, List<LatLngPoint>> grid;
        
        public SpatialGrid(double cellSize) {
            this.cellSize = cellSize;
            this.grid = new HashMap<>();
        }
        
        public void addPoint(LatLngPoint point) {
            String key = getCellKey(point);
            grid.computeIfAbsent(key, k -> new ArrayList<>()).add(point);
        }
        
        private String getCellKey(LatLngPoint point) {
            int x = (int) Math.floor(point.longitude / cellSize);
            int y = (int) Math.floor(point.latitude / cellSize);
            return x + "," + y;
        }
    }

    // 性能优化辅助方法
    
    /**
     * 快速三角函数计算
     */
    private static double fastCos(double degrees) {
        int index = (int) Math.round(degrees) % 360;
        if (index < 0) {
            index += 360;
        }
        return COS_TABLE[index];
    }

    /**
     * 对象池管理
     */
    private static LatLngPoint borrowPoint(double lat, double lng) {
        Queue<LatLngPoint> pool = POINT_POOL.get();
        LatLngPoint point = pool.poll();
        if (point == null) {
            point = new LatLngPoint();
        }
        point.latitude = lat;
        point.longitude = lng;
        return point;
    }

     private static void returnPoint(LatLngPoint point) {
         if (POINT_POOL.get().size() < 100) { // 限制池大小
             POINT_POOL.get().offer(point);
         }
     }
    
    /**
     * 缓存的距离计算
     */
    private static double calculateDistanceCached(LatLngPoint point1, LatLngPoint point2) {
        // 创建缓存键（考虑精度）
        String key = String.format("%.6f,%.6f-%.6f,%.6f", 
            point1.latitude, point1.longitude, point2.latitude, point2.longitude);
        
        return DISTANCE_CACHE.computeIfAbsent(key, k -> calculateDistance(point1, point2));
    }
    
    /**
     * 缓存的覆盖半径计算
     */
    private static double calculateEffectiveCoverageRadiusCached(ObliqueParams params) {
        String key = String.format("%.2f,%.2f,%.2f,%.2f,%.2f", 
            params.photoWidth, params.photoLength, params.gimbalPitch, 
            params.sideOverlapRate, params.forwardOverlapRate);
        
        return COVERAGE_RADIUS_CACHE.computeIfAbsent(key, k -> calculateEffectiveCoverageRadius(params));
    }
    
    /**
     * 快速距离计算（近距离使用平面近似）
     */
    private static double calculateDistanceFast(LatLngPoint point1, LatLngPoint point2) {
        double deltaLat = point2.latitude - point1.latitude;
        double deltaLng = point2.longitude - point1.longitude;
        
        // 对于小距离，使用平面近似计算（误差<1%）
        if (Math.abs(deltaLat) < 0.01 && Math.abs(deltaLng) < 0.01) {
            double avgLat = (point1.latitude + point2.latitude) / 2;
            double latDistance = deltaLat * 111000;
            double lngDistance = deltaLng * 111000 * Math.cos(Math.toRadians(avgLat));
            return Math.sqrt(latDistance * latDistance + lngDistance * lngDistance);
        }
        
        // 大距离仍使用精确的Haversine公式
        return calculateDistance(point1, point2);
    }
    
    /**
     * 边界框检查优化
     */
    private static boolean isPointInBoundingBox(LatLngPoint point, List<LatLngPoint> polygon) {
        double minLat = Double.MAX_VALUE, maxLat = Double.MIN_VALUE;
        double minLng = Double.MAX_VALUE, maxLng = Double.MIN_VALUE;
        
        for (LatLngPoint p : polygon) {
            minLat = Math.min(minLat, p.latitude);
            maxLat = Math.max(maxLat, p.latitude);
            minLng = Math.min(minLng, p.longitude);
            maxLng = Math.max(maxLng, p.longitude);
        }
        
        return point.latitude >= minLat && point.latitude <= maxLat &&
               point.longitude >= minLng && point.longitude <= maxLng;
    }
    
    /**
     * 边界框附近检查
     */
    private static boolean isPointNearBoundingBox(LatLngPoint point, List<LatLngPoint> polygon, double threshold) {
        double minLat = Double.MAX_VALUE, maxLat = Double.MIN_VALUE;
        double minLng = Double.MAX_VALUE, maxLng = Double.MIN_VALUE;
        
        for (LatLngPoint p : polygon) {
            minLat = Math.min(minLat, p.latitude);
            maxLat = Math.max(maxLat, p.latitude);
            minLng = Math.min(minLng, p.longitude);
            maxLng = Math.max(maxLng, p.longitude);
        }
        
        // 将阈值转换为经纬度
        double latThreshold = threshold / 111000.0;
        double avgLat = (minLat + maxLat) / 2;
        double lngThreshold = threshold / (111000.0 * Math.cos(Math.toRadians(avgLat)));
        
        return point.latitude >= (minLat - latThreshold) && point.latitude <= (maxLat + latThreshold) &&
               point.longitude >= (minLng - lngThreshold) && point.longitude <= (maxLng + lngThreshold);
    }
    
    /**
     * 规划倾斜摄影5航线
     * @param params 倾斜摄影参数
     * @return 倾斜摄影航线规划结果
     */
    public static ObliqueRouteResult planObliqueRoutes(ObliqueParams params) {
        // 参数验证
        validateParams(params);

        // 计算扩展区域以确保边缘覆盖
        List<LatLngPoint> expandedPolygon = expandPolygonForEdgeCoverage(params);

        // 分析云台俯仰角，判断是否需要完整的5航线
        RouteOptimizationResult optimization = analyzeRouteOptimization(params);

        List<RouteDirection> routes = new ArrayList<>();
        double totalDistance = 0;

        // 根据优化结果生成航线
        for (int i = 0; i < optimization.directions.length; i++) {
            double direction = optimization.directions[i];
            double gimbalPitch = optimization.gimbalPitches[i];
            String description = optimization.descriptions[i];

            // 使用扩展后的多边形进行航线规划
            RoutePathPlanner.RouteResult routeResult = RoutePathPlanner.planRouteWithMultipleBlocks(
                    expandedPolygon,
                    direction,
                    params.startPoint,
                    params.sideOverlapRate,
                    params.forwardOverlapRate,
                    params.photoWidth,
                    params.photoLength,
                    params.flightHeight,
                    10,false);

            if (!routeResult.waypoints.isEmpty()) {
                // 确保航点能够完全覆盖原始多边形区域
                List<LatLngPoint> effectiveWaypoints = ensureCompletePolygonCoverage(
                        routeResult.waypoints, params.polygonPoints, params);
                if (!effectiveWaypoints.isEmpty()) {
                    // 重新计算距离
                    double effectiveDistance = calculateRouteDistance(effectiveWaypoints);

                    effectiveWaypoints = RoutePathPlanner.optimizeWaypoints(effectiveWaypoints);
                    RouteDirection route = new RouteDirection(
                            direction,
                            gimbalPitch,
                            effectiveWaypoints,
                            effectiveDistance,
                            description
                    );
                    routes.add(route);
                    totalDistance += effectiveDistance;
                }
            }
        }

        // 计算边缘覆盖率
        double edgeCoverageRate = calculateEdgeCoverageRate(routes, params.polygonPoints, params);
        
        // 计算扩展距离
        double expansionDistance = calculateExpansionDistance(params);
        
        ObliqueRouteResult result = new ObliqueRouteResult(
            routes,
            totalDistance,
            routes.size(),
            optimization.isOptimized,
            optimization.reason,
            expandedPolygon,
            expansionDistance,
            edgeCoverageRate
        );
        
        return result;
    }

    /**
     * 分析航线优化策略
     */
    private static RouteOptimizationResult analyzeRouteOptimization(ObliqueParams params) {
        double pitch = Math.abs(params.gimbalPitch);
        
        // 云台俯仰角度不够大，使用垂直摄影
        if (pitch < Math.abs(MIN_EFFECTIVE_PITCH)) {
            return new RouteOptimizationResult(
                new double[]{params.mainDirection},
                new double[]{params.gimbalPitch},
                new String[]{"垂直摄影航线（云台角度不足）"},
                true,
                "云台俯仰角度小于" + Math.abs(MIN_EFFECTIVE_PITCH) + "度，采用单方向垂直摄影"
            );
        }

        // 云台角度适中，使用3航线十字交叉
        if (pitch >= Math.abs(MIN_EFFECTIVE_PITCH) && pitch < 30.0) {
            double[] directions = {
                params.mainDirection,
                (params.mainDirection + 90) % 360,
                (params.mainDirection + 180) % 360
            };
            double[] pitches = {params.gimbalPitch, params.gimbalPitch, params.gimbalPitch};
            String[] descriptions = {
                "主方向倾斜摄影航线",
                "垂直方向倾斜摄影航线",
                "反向倾斜摄影航线"
            };
            
            return new RouteOptimizationResult(
                directions, pitches, descriptions, true,
                "云台角度适中（" + pitch + "度），采用3方向十字交叉摄影提高效率"
            );
        }

        // 云台角度较大，使用4航线
        if (pitch >= 30.0 && pitch < 45.0) {
            double[] directions = {
                params.mainDirection,
                (params.mainDirection + 90) % 360,
                (params.mainDirection + 180) % 360,
                (params.mainDirection + 270) % 360
            };
            double[] pitches = {params.gimbalPitch, params.gimbalPitch, params.gimbalPitch, params.gimbalPitch};
            String[] descriptions = {
                "主方向倾斜摄影航线",
                "东向倾斜摄影航线",
                "南向倾斜摄影航线",
                "西向倾斜摄影航线"
            };
            
            return new RouteOptimizationResult(
                directions, pitches, descriptions, true,
                "云台角度较大（" + pitch + "度），采用4方向正交摄影保证覆盖质量"
            );
        }

        // 云台角度很大，需要完整的5航线
        double[] directions = new double[5];
        double[] pitches = new double[5];
        String[] descriptions = new String[5];
        
        for (int i = 0; i < 5; i++) {
            directions[i] = (params.mainDirection + STANDARD_5_DIRECTIONS[i]) % 360;
            pitches[i] = params.gimbalPitch;
            descriptions[i] = "第" + (i + 1) + "方向倾斜摄影航线（" + String.format("%.0f", directions[i]) + "度）";
        }

        return new RouteOptimizationResult(
            directions, pitches, descriptions, false,
            "云台角度很大（" + pitch + "度），需要完整5方向摄影确保无死角覆盖"
        );
    }

    /**
     * 航线优化结果
     */
    private static class RouteOptimizationResult {
        public double[] directions;
        public double[] gimbalPitches;
        public String[] descriptions;
        public boolean isOptimized;
        public String reason;

        public RouteOptimizationResult(double[] directions, double[] gimbalPitches, String[] descriptions,
                                     boolean isOptimized, String reason) {
            this.directions = directions;
            this.gimbalPitches = gimbalPitches;
            this.descriptions = descriptions;
            this.isOptimized = isOptimized;
            this.reason = reason;
        }
    }

    /**
     * 参数验证
     */
    private static void validateParams(ObliqueParams params) {
        if (params.polygonPoints == null || params.polygonPoints.size() < 3) {
            throw new RuntimeException("作业区域多边形至少需要3个顶点");
        }
        if (params.sideOverlapRate < 0 || params.sideOverlapRate > 100) {
            throw new RuntimeException("旁路覆盖率必须在0-100%之间");
        }
        if (params.forwardOverlapRate < 0 || params.forwardOverlapRate > 100) {
            throw new RuntimeException("路线覆盖率必须在0-100%之间");
        }
        if (params.photoWidth <= 0 || params.photoLength <= 0) {
            throw new RuntimeException("照片尺寸必须大于0");
        }
        if (params.flightHeight <= 0) {
            throw new RuntimeException("飞行高度必须大于0");
        }
        if (params.gimbalPitch > 0) {
            throw new RuntimeException("云台俯仰角应为负值（向下）");
        }
    }

    /**
     * 扩展多边形以确保边缘覆盖
     * 根据倾斜摄影参数计算需要扩展的距离
     */
    private static List<LatLngPoint> expandPolygonForEdgeCoverage(ObliqueParams params) {
        // 使用统一的扩展距离计算函数
        double expansionDistance = calculateExpansionDistance(params);
        return expandPolygon(params.polygonPoints, expansionDistance);
    }
    
    /**
     * 向外扩展多边形（优化版本 - 减少对象创建）
     */
    private static List<LatLngPoint> expandPolygon(List<LatLngPoint> originalPolygon, double expansionDistance) {
        if (originalPolygon.size() < 3) {
            return new ArrayList<>(originalPolygon);
        }
        
        // 预分配容量，避免动态扩容
        List<LatLngPoint> expandedPolygon = new ArrayList<>(originalPolygon.size());
        int n = originalPolygon.size();
        
        // 确保多边形是逆时针方向（复用现有对象）
        List<LatLngPoint> polygon = ensureCounterClockwise(originalPolygon);
        
        // 复用数组对象，避免重复创建
        double[] outwardNormal = new double[2];
        double[] offset = new double[2];
        
        for (int i = 0; i < n; i++) {
            LatLngPoint current = polygon.get(i);
            LatLngPoint prev = polygon.get((i - 1 + n) % n);
            LatLngPoint next = polygon.get((i + 1) % n);
            
            // 计算当前点的向外法向量（复用数组）
            calculateOutwardNormalFixed(prev, current, next, outwardNormal);
            
            // 将扩展距离转换为经纬度偏移（复用数组）
            convertDistanceToLatLngOffset(current, expansionDistance, outwardNormal, offset);
            
            LatLngPoint expandedPoint = new LatLngPoint(
                current.latitude + offset[0],
                current.longitude + offset[1]
            );
            
            expandedPolygon.add(expandedPoint);
        }
        
        return expandedPolygon;
    }
    
    /**
     * 确保多边形是逆时针方向
     */
    private static List<LatLngPoint> ensureCounterClockwise(List<LatLngPoint> polygon) {
        if (polygon.size() < 3) {
            return new ArrayList<>(polygon);
        }
        
        // 计算多边形的有向面积
        double signedArea = 0.0;
        int n = polygon.size();
        
        for (int i = 0; i < n; i++) {
            LatLngPoint p1 = polygon.get(i);
            LatLngPoint p2 = polygon.get((i + 1) % n);
            signedArea += (p2.longitude - p1.longitude) * (p2.latitude + p1.latitude);
        }
        
        // 如果有向面积为正，说明是顺时针，需要反转
        if (signedArea > 0) {
            List<LatLngPoint> reversed = new ArrayList<>(polygon);
            Collections.reverse(reversed);
            return reversed;
        }
        
        return new ArrayList<>(polygon);
    }
    
    /**
     * 计算修复后的向外法向量（优化版本 - 使用输出参数避免数组创建）
     */
    private static void calculateOutwardNormalFixed(LatLngPoint prev, LatLngPoint current, LatLngPoint next, double[] result) {
        // 计算两个相邻边的向量
        double edge1Lat = current.latitude - prev.latitude;
        double edge1Lng = current.longitude - prev.longitude;
        double edge2Lat = next.latitude - current.latitude;
        double edge2Lng = next.longitude - current.longitude;
        
        // 计算边的法向量（向右旋转90度）
        double normal1Lat = -edge1Lng;
        double normal1Lng = edge1Lat;
        double normal2Lat = -edge2Lng;
        double normal2Lng = edge2Lat;
        
        // 计算角平分线方向
        double bisectorLat = normal1Lat + normal2Lat;
        double bisectorLng = normal1Lng + normal2Lng;
        
        // 归一化
        double length = Math.sqrt(bisectorLat * bisectorLat + bisectorLng * bisectorLng);
        if (length > 1e-10) {
            result[0] = bisectorLat / length;
            result[1] = bisectorLng / length;
        } else {
            // 如果角平分线长度为0（180度角），使用垂直于第一条边的向量
            length = Math.sqrt(normal1Lat * normal1Lat + normal1Lng * normal1Lng);
            if (length > 1e-10) {
                result[0] = normal1Lat / length;
                result[1] = normal1Lng / length;
            } else {
                result[0] = 0;
                result[1] = 0;
            }
        }
    }
    
    /**
     * 将距离转换为经纬度偏移（优化版本 - 使用输出参数避免数组创建）
     */
    private static void convertDistanceToLatLngOffset(LatLngPoint point, double distance, double[] direction, double[] result) {
        // 纬度偏移（1度纬度约等于111000米）
        result[0] = (distance * direction[0]) / 111000.0;
        
        // 经度偏移（考虑纬度的影响）
        double cosLat = Math.cos(Math.toRadians(point.latitude));
        result[1] = (distance * direction[1]) / (111000.0 * cosLat);
    }
    
    /**
     * 确保航点能够完全覆盖原始多边形区域（优化版本 - 使用缓存和更高效的集合）
     * 不仅要覆盖边缘，还要确保整个多边形内部区域都能被拍摄到
     */
    private static List<LatLngPoint> ensureCompletePolygonCoverage(List<LatLngPoint> waypoints, 
                                                                   List<LatLngPoint> originalPolygon, 
                                                                   ObliqueParams params) {
        // 使用LinkedHashSet保持顺序并避免重复
        Set<LatLngPoint> effectiveWaypoints = new LinkedHashSet<>();
        
        // 计算有效覆盖半径（使用缓存）
        double coverageRadius = calculateEffectiveCoverageRadiusCached(params);
        
        // 构建空间索引
        SpatialGrid waypointGrid = new SpatialGrid(coverageRadius);
        for (LatLngPoint waypoint : waypoints) {
            waypointGrid.addPoint(waypoint);
        }
        
        // 1. 首先保留所有能够覆盖原始多边形的航点
        for (LatLngPoint waypoint : waypoints) {
            if (canWaypointCoverPolygonAreaFast(waypoint, originalPolygon, coverageRadius)) {
                effectiveWaypoints.add(waypoint);
            }
        }
        
        // 2. 检查原始多边形的覆盖完整性
        List<LatLngPoint> uncoveredAreas = findUncoveredAreas(originalPolygon, new ArrayList<>(effectiveWaypoints), coverageRadius);
        
        // 3. 如果有未覆盖区域，从原始航点中补充
        if (!uncoveredAreas.isEmpty()) {
            for (LatLngPoint waypoint : waypoints) {
                if (!effectiveWaypoints.contains(waypoint)) {
                    // 检查这个航点是否能覆盖未覆盖的区域
                    boolean coversUncoveredArea = false;
                    for (LatLngPoint uncoveredPoint : uncoveredAreas) {
                        if (calculateDistanceFast(waypoint, uncoveredPoint) <= coverageRadius) {
                            coversUncoveredArea = true;
                            break;
                        }
                    }
                    if (coversUncoveredArea) {
                        effectiveWaypoints.add(waypoint);
                    }
                }
            }
        }
        
        return new ArrayList<>(effectiveWaypoints);
    }
    
    /**
     * 计算倾斜摄影的有效覆盖半径
     */
    private static double calculateEffectiveCoverageRadius(ObliqueParams params) {
        // 基础覆盖半径
        double baseRadius = Math.max(params.photoWidth, params.photoLength) * 0.5;
        
        // 考虑倾斜角度的影响 - 使用快速三角函数
        double tiltFactor = fastCos(Math.abs(params.gimbalPitch)); // 倾斜角度越大，有效覆盖面积越小
        
        // 考虑重叠率的影响
        double overlapFactor = Math.min(params.sideOverlapRate, params.forwardOverlapRate) / 100.0;
        double effectiveRadius = baseRadius * tiltFactor * (1 - overlapFactor * 0.3);
        
        // 确保最小覆盖半径
        return Math.max(effectiveRadius, baseRadius * 0.4);
    }
    
    /**
     * 检查航点是否能够覆盖多边形区域（优化版本 - 早期退出）
     */
    private static boolean canWaypointCoverPolygonAreaFast(LatLngPoint waypoint, List<LatLngPoint> polygon, double coverageRadius) {
        // 先检查是否在多边形内部（最快的情况）
        if (isPointInPolygonOptimized(waypoint, polygon)) {
            return true;
        }
        
        // 快速边界框检查
        if (!isPointNearBoundingBox(waypoint, polygon, coverageRadius)) {
            return false;
        }
        
        // 详细的边界距离检查
        return isPointNearPolygon(waypoint, polygon, coverageRadius);
    }
    
    /**
     * 找出原始多边形中未被覆盖的区域（优化版本 - 使用并行流）
     */
    private static List<LatLngPoint> findUncoveredAreas(List<LatLngPoint> originalPolygon, 
                                                        List<LatLngPoint> waypoints, 
                                                        double coverageRadius) {
        // 在多边形内部进行网格采样，检查覆盖情况
        List<LatLngPoint> samplePoints = samplePolygonInterior(originalPolygon, coverageRadius * 0.5);
        
        // 使用并行流处理大量采样点
        return samplePoints.parallelStream()
            .filter(samplePoint -> {
                return waypoints.stream().noneMatch(waypoint -> 
                    calculateDistanceFast(samplePoint, waypoint) <= coverageRadius);
            })
            .collect(Collectors.toList());
    }
    
    /**
     * 在多边形内部进行采样
     */
    private static List<LatLngPoint> samplePolygonInterior(List<LatLngPoint> polygon, double sampleDistance) {
        List<LatLngPoint> samplePoints = new ArrayList<>();
        
        // 计算多边形边界框
        double minLat = polygon.stream().mapToDouble(p -> p.latitude).min().orElse(0);
        double maxLat = polygon.stream().mapToDouble(p -> p.latitude).max().orElse(0);
        double minLng = polygon.stream().mapToDouble(p -> p.longitude).min().orElse(0);
        double maxLng = polygon.stream().mapToDouble(p -> p.longitude).max().orElse(0);
        
        // 转换采样距离为经纬度增量
        double latStep = sampleDistance / 111000.0;
        double avgLat = (minLat + maxLat) / 2;
        double lngStep = sampleDistance / (111000.0 * Math.cos(Math.toRadians(avgLat)));
        
        // 在边界框内进行网格采样
        for (double lat = minLat; lat <= maxLat; lat += latStep) {
            for (double lng = minLng; lng <= maxLng; lng += lngStep) {
                LatLngPoint samplePoint = new LatLngPoint(lat, lng);
                if (isPointInPolygon(samplePoint, polygon)) {
                    samplePoints.add(samplePoint);
                }
            }
        }
        
        // 同时在多边形边界上采样
        samplePoints.addAll(samplePolygonEdge(polygon, sampleDistance));
        
        return samplePoints;
    }
    
    /**
     * 检查点是否接近多边形（优化版本 - 使用快速距离计算）
     */
    private static boolean isPointNearPolygon(LatLngPoint point, List<LatLngPoint> polygon, double threshold) {
        // 检查点是否在多边形内
        if (isPointInPolygon(point, polygon)) {
            return true;
        }
        
        // 检查点到多边形边界的最短距离
        for (int i = 0; i < polygon.size(); i++) {
            LatLngPoint p1 = polygon.get(i);
            LatLngPoint p2 = polygon.get((i + 1) % polygon.size());
            
            if (distanceToLineSegmentFast(point, p1, p2) <= threshold) {
                return true;
            }
        }
        
        return false;
    }
    
    /**
     * 计算点到线段的距离（优化版本 - 使用快速距离计算和对象池）
     */
    private static double distanceToLineSegmentFast(LatLngPoint point, LatLngPoint lineStart, LatLngPoint lineEnd) {
        double A = point.latitude - lineStart.latitude;
        double B = point.longitude - lineStart.longitude;
        double C = lineEnd.latitude - lineStart.latitude;
        double D = lineEnd.longitude - lineStart.longitude;
        
        double dot = A * C + B * D;
        double lenSq = C * C + D * D;
        
        if (lenSq == 0) {
            return calculateDistanceFast(point, lineStart);
        }
        
        double param = dot / lenSq;
        
        if (param < 0) {
            return calculateDistanceFast(point, lineStart);
        } else if (param > 1) {
            return calculateDistanceFast(point, lineEnd);
        } else {
            LatLngPoint projection = borrowPoint(
                lineStart.latitude + param * C,
                lineStart.longitude + param * D
            );
            
            double distance = calculateDistanceFast(point, projection);
            returnPoint(projection);
            return distance;
        }
    }
    
    /**
     * 计算点到线段的距离（保持向后兼容）
     */
    private static double distanceToLineSegment(LatLngPoint point, LatLngPoint lineStart, LatLngPoint lineEnd) {
        return distanceToLineSegmentFast(point, lineStart, lineEnd);
    }
    
    /**
     * 检查点是否在多边形内部（射线法 - 优化版本）
     */
    private static boolean isPointInPolygonOptimized(LatLngPoint point, List<LatLngPoint> polygon) {
        // 先进行边界框检查，快速排除明显不在多边形内的点
        if (!isPointInBoundingBox(point, polygon)) {
            return false;
        }
        
        // 原有的射线法逻辑
        int intersections = 0;
        int n = polygon.size();
        
        for (int i = 0; i < n; i++) {
            LatLngPoint p1 = polygon.get(i);
            LatLngPoint p2 = polygon.get((i + 1) % n);
            
            if (((p1.latitude > point.latitude) != (p2.latitude > point.latitude)) &&
                (point.longitude < (p2.longitude - p1.longitude) * (point.latitude - p1.latitude) / (p2.latitude - p1.latitude) + p1.longitude)) {
                intersections++;
            }
        }
        
        return (intersections % 2) == 1;
    }
    
    /**
     * 检查点是否在多边形内部（保持向后兼容）
     */
    private static boolean isPointInPolygon(LatLngPoint point, List<LatLngPoint> polygon) {
        return isPointInPolygonOptimized(point, polygon);
    }
    
    /**
     * 计算航线总距离（优化版本 - 使用缓存）
     */
    private static double calculateRouteDistance(List<LatLngPoint> waypoints) {
        if (waypoints.size() < 2) {
            return 0;
        }
        
        double totalDistance = 0;
        for (int i = 1; i < waypoints.size(); i++) {
            totalDistance += calculateDistanceCached(waypoints.get(i - 1), waypoints.get(i));
        }
        
        return totalDistance;
    }
    
    /**
     * 计算两点间距离（Haversine公式）
     */
    private static double calculateDistance(LatLngPoint point1, LatLngPoint point2) {
        double lat1Rad = Math.toRadians(point1.latitude);
        double lat2Rad = Math.toRadians(point2.latitude);
        double deltaLatRad = Math.toRadians(point2.latitude - point1.latitude);
        double deltaLngRad = Math.toRadians(point2.longitude - point1.longitude);

        double a = Math.sin(deltaLatRad / 2) * Math.sin(deltaLatRad / 2) +
                Math.cos(lat1Rad) * Math.cos(lat2Rad) *
                Math.sin(deltaLngRad / 2) * Math.sin(deltaLngRad / 2);
        double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

        return EARTH_RADIUS * c;
    }

    /**
     * 获取推荐的倾斜摄影参数
     * @param targetHeight 目标飞行高度（米）
     * @param cameraFOV 相机视场角（度）
     * @return 推荐的云台俯仰角
     */
    public static double getRecommendedGimbalPitch(double targetHeight, double cameraFOV) {
        // 根据飞行高度和相机参数推荐合适的云台俯仰角
        if (targetHeight < 50) {
            return -20.0; // 低空飞行，较小俯仰角
        } else if (targetHeight < 100) {
            return -30.0; // 中等高度，中等俯仰角
        } else if (targetHeight < 200) {
            return -45.0; // 较高飞行，较大俯仰角
        } else {
            return -60.0; // 高空飞行，大俯仰角
        }
    }

    /**
     * 获取扩展后的作业区域信息
     * @param params 倾斜摄影参数
     * @return 扩展区域信息
     */
    public static ExpandedAreaInfo getExpandedAreaInfo(ObliqueParams params) {
        validateParams(params);
        
        List<LatLngPoint> expandedPolygon = expandPolygonForEdgeCoverage(params);
        double expansionDistance = calculateExpansionDistance(params);
        double originalArea = calculatePolygonArea(params.polygonPoints);
        double expandedArea = calculatePolygonArea(expandedPolygon);
        
        return new ExpandedAreaInfo(
            expandedPolygon,
            expansionDistance,
            originalArea,
            expandedArea,
            (expandedArea - originalArea) / originalArea * 100
        );
    }
    
    /**
     * 扩展区域信息
     */
    @Data
    @AllArgsConstructor
    @NoArgsConstructor
    public static class ExpandedAreaInfo {
        public List<LatLngPoint> expandedPolygon; // 扩展后的多边形
        public double expansionDistance;         // 扩展距离（米）
        public double originalArea;              // 原始面积（平方米）
        public double expandedArea;              // 扩展后面积（平方米）
        public double areaIncreasePercentage;    // 面积增加百分比
    }
    
    /**
     * 计算扩展距离（修复版本）
     */
    private static double calculateExpansionDistance(ObliqueParams params) {
        double gimbalPitchRad = Math.toRadians(Math.abs(params.gimbalPitch));
        
        // 照片的地面覆盖尺寸
        double maxGroundCoverage = Math.max(params.photoWidth, params.photoLength);
        
        // 计算倾斜摄影的实际地面投影偏移
        // 这里使用更精确的计算方法
        double tiltOffset = 0;
        if (Math.abs(params.gimbalPitch) > 5.0) { // 只有在明显倾斜时才考虑偏移
            tiltOffset = params.flightHeight * Math.tan(gimbalPitchRad) * 0.5; // 减小偏移影响
        }
        
        // 考虑重叠率影响
        double minOverlapRate = Math.min(params.sideOverlapRate, params.forwardOverlapRate) / 100.0;
        double overlapFactor = 1.0 - minOverlapRate * 0.1; // 减小重叠率的影响
        
        // 基础扩展距离：主要基于照片覆盖范围
        double baseExpansion = maxGroundCoverage * 0.6; // 减小基础扩展系数
        double expansionDistance = (baseExpansion + tiltOffset) * overlapFactor;
        
        // 设置更合理的扩展距离范围
        double minExpansion = maxGroundCoverage * 0.3; // 最小扩展距离
        double maxExpansion = maxGroundCoverage * 0.8 + tiltOffset; // 最大扩展距离
        
        return Math.max(minExpansion, Math.min(expansionDistance, maxExpansion));
    }
    
    /**
     * 计算边缘覆盖率
     */
    private static double calculateEdgeCoverageRate(List<RouteDirection> routes, List<LatLngPoint> originalPolygon, ObliqueParams params) {
        if (routes.isEmpty()) {
            return 0.0;
        }
        
        // 收集所有航点
        List<LatLngPoint> allWaypoints = new ArrayList<>();
        for (RouteDirection route : routes) {
            allWaypoints.addAll(route.waypoints);
        }
        
        // 计算覆盖半径
        double coverageRadius = Math.max(params.photoWidth, params.photoLength) * 0.6;
        
        // 在多边形边界上采样点
        List<LatLngPoint> edgeSamplePoints = samplePolygonEdge(originalPolygon, 10.0); // 每10米采样一个点
        
        int coveredPoints = 0;
        for (LatLngPoint edgePoint : edgeSamplePoints) {
            boolean covered = false;
            for (LatLngPoint waypoint : allWaypoints) {
                if (calculateDistance(edgePoint, waypoint) <= coverageRadius) {
                    covered = true;
                    break;
                }
            }
            if (covered) {
                coveredPoints++;
            }
        }
        
        return edgeSamplePoints.isEmpty() ? 100.0 : (double) coveredPoints / edgeSamplePoints.size() * 100.0;
    }
    
    /**
     * 在多边形边界上采样点
     */
    private static List<LatLngPoint> samplePolygonEdge(List<LatLngPoint> polygon, double sampleDistance) {
        List<LatLngPoint> samplePoints = new ArrayList<>();
        
        for (int i = 0; i < polygon.size(); i++) {
            LatLngPoint p1 = polygon.get(i);
            LatLngPoint p2 = polygon.get((i + 1) % polygon.size());
            
            double edgeLength = calculateDistance(p1, p2);
            int sampleCount = Math.max(1, (int) Math.ceil(edgeLength / sampleDistance));
            
            for (int j = 0; j < sampleCount; j++) {
                double ratio = (double) j / sampleCount;
                LatLngPoint samplePoint = new LatLngPoint(
                    p1.latitude + (p2.latitude - p1.latitude) * ratio,
                    p1.longitude + (p2.longitude - p1.longitude) * ratio
                );
                samplePoints.add(samplePoint);
            }
        }
        
        return samplePoints;
    }
    
    /**
     * 计算多边形面积（平方米）
     */
    private static double calculatePolygonArea(List<LatLngPoint> polygon) {
        if (polygon.size() < 3) {
            return 0;
        }
        
        double area = 0;
        int n = polygon.size();
        
        for (int i = 0; i < n; i++) {
            LatLngPoint p1 = polygon.get(i);
            LatLngPoint p2 = polygon.get((i + 1) % n);
            
            // 使用球面三角形面积计算（简化版）
            double lat1 = Math.toRadians(p1.latitude);
            double lat2 = Math.toRadians(p2.latitude);
            double lng1 = Math.toRadians(p1.longitude);
            double lng2 = Math.toRadians(p2.longitude);
            
            area += (lng2 - lng1) * (2 + Math.sin(lat1) + Math.sin(lat2));
        }
        
        area = Math.abs(area) * EARTH_RADIUS * EARTH_RADIUS / 2.0;
        return area;
    }

    /**
     * 估算倾斜摄影作业时间
     * @param result 航线规划结果
     * @param cruiseSpeed 巡航速度（米/秒）
     * @param photoInterval 拍照间隔（秒）
     * @return 预估作业时间（分钟）
     */
    public static double estimateWorkTime(ObliqueRouteResult result, double cruiseSpeed, double photoInterval) {
        if (result.routes.isEmpty()) {
            return 0;
        }

        double totalFlightTime = result.totalDistance / cruiseSpeed; // 飞行时间（秒）
        
        // 估算总拍照点数
        int totalPhotoPoints = 0;
        for (RouteDirection route : result.routes) {
            totalPhotoPoints += route.waypoints.size();
        }
        
        double totalPhotoTime = totalPhotoPoints * photoInterval; // 拍照时间（秒）
        double routeChangeTime = (result.totalRouteCount - 1) * 60; // 航线切换时间（秒）
        
        return (totalFlightTime + totalPhotoTime + routeChangeTime) / 60.0; // 转换为分钟
    }
}
