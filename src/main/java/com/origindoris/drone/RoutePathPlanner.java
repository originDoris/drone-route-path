package com.origindoris.drone;


import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.stream.Collectors;
import java.util.stream.IntStream;



/**
 * 路线规划算法类
 * 用于在多边形区域内规划无人机航线，确保航线和航点都在多边形内部
 */
public class RoutePathPlanner {

    /**
     * 路线规划结果
     */
    public static class RouteResult {
        public List<LatLngPoint> waypoints;  // 航点列表
        public double totalDistance;        // 总距离（米）
        public int totalLines;              // 总航线数
        
        public RouteResult() {}
        
        public RouteResult(List<LatLngPoint> waypoints, double totalDistance, int totalLines) {
            this.waypoints = waypoints;
            this.totalDistance = totalDistance;
            this.totalLines = totalLines;
        }
    }

    private static final double EARTH_RADIUS = 6371000; // 地球半径（米）
    
    // 性能优化：距离计算缓存
    private static final Map<String, Double> distanceCache = new ConcurrentHashMap<>();
    private static final int MAX_CACHE_SIZE = 10000;
    
    // 性能优化：预计算的三角函数值缓存
    private static final Map<String, Double> trigCache = new ConcurrentHashMap<>();
    
    // 新增性能优化缓存
    private static final Map<String, Boolean> pointInPolygonCache = new ConcurrentHashMap<>();
    private static final Map<String, List<LatLngPoint>> intersectionCache = new ConcurrentHashMap<>();
    private static final Map<String, LatLngPoint[]> boundsCache = new ConcurrentHashMap<>();
    
    // 缓存清理阈值
    private static final int CACHE_SIZE_THRESHOLD = 10000;
    private static final long CACHE_CLEANUP_INTERVAL = 300000; // 5分钟
    private static long lastCacheCleanup = System.currentTimeMillis();
    
    // 性能监控
    private static long totalPlanningTime = 0;
    private static int planningCount = 0;
    
    /**
     * 获取平均规划时间（毫秒）
     */
    public static double getAveragePlanningTime() {
        return planningCount > 0 ? (double) totalPlanningTime / planningCount : 0;
    }
    
    /**
     * 重置性能统计
     */
    public static void resetPerformanceStats() {
        totalPlanningTime = 0;
        planningCount = 0;
        distanceCache.clear();
        trigCache.clear();
        pointInPolygonCache.clear();
        intersectionCache.clear();
        boundsCache.clear();
        lastCacheCleanup = System.currentTimeMillis();
    }
    
    /**
     * 自动清理缓存以防止内存溢出
     */
    private static void cleanupCachesIfNeeded() {
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastCacheCleanup > CACHE_CLEANUP_INTERVAL ||
            distanceCache.size() > CACHE_SIZE_THRESHOLD ||
            pointInPolygonCache.size() > CACHE_SIZE_THRESHOLD) {
            
            // 清理最老的缓存项（简单策略：清理一半）
            if (distanceCache.size() > CACHE_SIZE_THRESHOLD / 2) {
                distanceCache.clear();
            }
            if (pointInPolygonCache.size() > CACHE_SIZE_THRESHOLD / 2) {
                pointInPolygonCache.clear();
            }
            if (intersectionCache.size() > CACHE_SIZE_THRESHOLD / 2) {
                intersectionCache.clear();
            }
            if (boundsCache.size() > CACHE_SIZE_THRESHOLD / 2) {
                boundsCache.clear();
            }
            
            lastCacheCleanup = currentTime;
        }
    }

    /**
     * 规划路线
     * @param polygonPoints 多边形顶点坐标列表
     * @param direction 路线方向（0-360度）
     * @param startPoint 起始点坐标
     * @param sideOverlapRate 旁路覆盖率（0-100%）
     * @param forwardOverlapRate 路线覆盖率（0-100%）
     * @param photoWidth 照片宽度（米）
     * @param photoLength 照片长度（米）
     * @param flightHeight 飞行高度（米）
     * @return 路线规划结果
     */
    public static RouteResult planRoute(List<LatLngPoint> polygonPoints,
                                        double direction,
                                        LatLngPoint startPoint,
                                        double sideOverlapRate,
                                        double forwardOverlapRate,
                                        double photoWidth,
                                        double photoLength,
                                        double flightHeight) {

        // 性能监控：记录开始时间
        long startTime = System.currentTimeMillis();
        
        try {
            if (polygonPoints == null || polygonPoints.size() < 3) {
                throw new RuntimeException("多边形至少需要3个顶点");
            }

            // 计算实际航线间距和航点间距
            double lineSpacing = photoWidth * (1 - sideOverlapRate / 100.0);
            double pointSpacing = photoLength * (1 - forwardOverlapRate / 100.0);

            // 将方向转换为弧度
            double directionRad = Math.toRadians(direction);

            // 计算垂直于航线方向的向量（用于航线间距）
            double perpDirRad = directionRad + Math.PI / 2;

            // 如果起始点不在多边形内，找到最近的多边形内部点
            LatLngPoint actualStartPoint = startPoint;
            if (!isPointInPolygon(startPoint, polygonPoints)) {

                // 方法1：找到多边形重心
                double centerLat = 0, centerLng = 0;
                for (LatLngPoint point : polygonPoints) {
                    centerLat += point.latitude;
                    centerLng += point.longitude;
                }
                centerLat /= polygonPoints.size();
                centerLng /= polygonPoints.size();
                LatLngPoint centroid = new LatLngPoint(centerLat, centerLng);

                // 验证重心是否在多边形内
                if (isPointInPolygon(centroid, polygonPoints)) {
                    actualStartPoint = centroid;
                } else {
                    // 方法2：在多边形边界上找最近点，然后向内偏移
                    LatLngPoint closestBoundaryPoint = findClosestPointOnPolygon(startPoint, polygonPoints);
                    // 向多边形重心方向偏移一小段距离
                    double offsetRatio = 0.01; // 偏移1%
                    actualStartPoint = interpolatePoint(closestBoundaryPoint, centroid, offsetRatio);

                    // 如果偏移后的点仍不在多边形内，直接使用重心
                    if (!isPointInPolygon(actualStartPoint, polygonPoints)) {
                        actualStartPoint = centroid;
                    }
                }
            }

            // 找到起始航线位置
            LatLngPoint entryPoint = findPolygonEntry(polygonPoints, actualStartPoint, directionRad);
            if (entryPoint == null) {
                // 如果起始点在多边形外，找到最近的进入点
                entryPoint = findNearestEntryPoint(polygonPoints, actualStartPoint, directionRad);
            }

            // 生成平行航线
            List<List<LatLngPoint>> lines = generateParallelLines(
                    polygonPoints, entryPoint, directionRad, perpDirRad, lineSpacing, pointSpacing);

            // 优化航线顺序以最小化总距离
            List<LatLngPoint> optimizedRoute = optimizeRouteOrder(lines, actualStartPoint, polygonPoints);

            // 智能航点间距验证：只在航点密度过高时进行严格验证
            List<LatLngPoint> validatedRoute = optimizedRoute;

            // 计算航点密度（航点数/多边形面积）
            double polygonArea = calculatePolygonArea(polygonPoints);
            double waypointDensity = optimizedRoute.size() / polygonArea; // 航点数/平方米
            double expectedDensity = 1.0 / (lineSpacing * pointSpacing); // 期望密度

            // 只有当航点密度明显超过期望密度时才进行严格验证
            if (waypointDensity > expectedDensity * 1.5) {
                validatedRoute = validateWaypointSpacing(optimizedRoute, new ArrayList<>(), pointSpacing);
            }
            
            // 新增：航点优化 - 合并同一直线上的冗余航点
//            List<LatLngPoint> finalOptimizedRoute = optimizeWaypoints(validatedRoute);

            // 计算总距离
            double totalDistance = calculateTotalDistance(validatedRoute);

            return new RouteResult(validatedRoute, totalDistance, lines.size());
        
        } finally {
            // 性能监控：记录结束时间
            long endTime = System.currentTimeMillis();
            synchronized (RoutePathPlanner.class) {
                totalPlanningTime += (endTime - startTime);
                planningCount++;
            }
        }
    }

    /**
     * 多次分块规划路线，允许回头路径来提高覆盖率
     * @param polygonPoints 多边形顶点列表
     * @param direction 航线方向（度）
     * @param startPoint 起始点
     * @param sideOverlapRate 旁路覆盖率（%）
     * @param forwardOverlapRate 路线覆盖率（%）
     * @param photoWidth 照片宽度（米）
     * @param photoLength 照片长度（米）
     * @param flightHeight 飞行高度（米）
     * @param maxBlocks 最大分块数量
     * @return 多次规划合并后的路线结果
     */
    public static RouteResult planRouteWithMultipleBlocks(List<LatLngPoint> polygonPoints,
                                                          double direction,
                                                          LatLngPoint startPoint,
                                                          double sideOverlapRate,
                                                          double forwardOverlapRate,
                                                          double photoWidth,
                                                          double photoLength,
                                                          double flightHeight,
                                                          int maxBlocks,
                                                          boolean optimize) {


        List<LatLngPoint> allWaypoints = new ArrayList<>();
        List<List<LatLngPoint>> allRouteSegments = new ArrayList<>();
        double totalDistance = 0;
        int totalLines = 0;

        // 第一次规划 - 主要覆盖
        RouteResult firstResult = planRoute(polygonPoints, direction, startPoint,
                sideOverlapRate, forwardOverlapRate,
                photoWidth, photoLength, flightHeight);

        if (!firstResult.waypoints.isEmpty()) {
            allWaypoints.addAll(firstResult.waypoints);
            allRouteSegments.add(new ArrayList<>(firstResult.waypoints));
            totalDistance += firstResult.totalDistance;
            totalLines += firstResult.totalLines;
        }

        // 分析未覆盖区域并进行补充规划
        for (int blockIndex = 2; blockIndex <= maxBlocks; blockIndex++) {

            // 寻找未覆盖的区域
            List<LatLngPoint> uncoveredRegion = findUncoveredRegion(polygonPoints, allWaypoints,
                    photoWidth, photoLength);

            if (uncoveredRegion.isEmpty()) {
                break;
            }


            // 为未覆盖区域规划补充路线
            // 直接在未覆盖点周围生成补充航点
            List<LatLngPoint> supplementaryWaypoints = generateSupplementaryWaypoints(
                    uncoveredRegion, polygonPoints, direction, sideOverlapRate, forwardOverlapRate,
                    photoWidth, photoLength, flightHeight, allWaypoints);

            RouteResult bestBlockResult = null;
            if (!supplementaryWaypoints.isEmpty()) {
                bestBlockResult = new RouteResult(supplementaryWaypoints, 0, supplementaryWaypoints.size() - 1);
            }

            if (bestBlockResult != null && !bestBlockResult.waypoints.isEmpty()) {

                // 清理新规划的航点，确保都在多边形内
                List<LatLngPoint> cleanedNewWaypoints = cleanupInvalidConnections(bestBlockResult.waypoints, polygonPoints);

                if (!cleanedNewWaypoints.isEmpty()) {
                    // 连接到现有路线
                    List<LatLngPoint> connectedSegment = connectToExistingRoute(
                            allWaypoints, cleanedNewWaypoints, polygonPoints);

                    // 再次清理连接后的航点
                    List<LatLngPoint> finalCleanedSegment = cleanupInvalidConnections(connectedSegment, polygonPoints);

                    if (!finalCleanedSegment.isEmpty()) {
                        allWaypoints.addAll(finalCleanedSegment);
                        allRouteSegments.add(finalCleanedSegment);
                        totalDistance += bestBlockResult.totalDistance;
                        totalDistance += calculateConnectionDistance(allWaypoints, finalCleanedSegment);
                        totalLines += bestBlockResult.totalLines;
                    }
                }
            } else {
                break;
            }
        }

        // 最终清理：确保所有航点和航线都在多边形内
        List<LatLngPoint> finalCleanedWaypoints = cleanupInvalidConnections(allWaypoints, polygonPoints);

        if (optimize) {
            // 新增：航点优化 - 合并同一直线上的冗余航点
            finalCleanedWaypoints = optimizeWaypoints(finalCleanedWaypoints);
        }

        return new RouteResult(finalCleanedWaypoints, totalDistance, totalLines);
    }

    /**
     * 寻找未覆盖的区域
     */
    private static List<LatLngPoint> findUncoveredRegion(List<LatLngPoint> originalPolygon,
                                                         List<LatLngPoint> existingWaypoints,
                                                         double photoWidth,
                                                         double photoLength) {

        // 改进版本：基于更精细的网格采样来识别未覆盖区域
        List<LatLngPoint> uncoveredPoints = new ArrayList<>();

        // 获取多边形边界
        LatLngPoint[] bounds = getBounds(originalPolygon);
        LatLngPoint minBound = bounds[0];
        LatLngPoint maxBound = bounds[1];

        // 智能自适应网格采样：根据多边形面积动态调整
        double polygonArea = calculatePolygonArea(originalPolygon);
        double baseGridSize = Math.min(photoWidth, photoLength);
        
        // 根据面积自适应调整网格大小
        double adaptiveGridSize;
        if (polygonArea < 10000) { // 小于1公顷
            adaptiveGridSize = baseGridSize * 0.2; // 更精细采样
        } else if (polygonArea < 100000) { // 小于10公顷
            adaptiveGridSize = baseGridSize * 0.3;
        } else {
            adaptiveGridSize = baseGridSize * 0.5; // 大面积使用粗采样
        }
        
        double latStep = adaptiveGridSize / 111000.0;
        double lngStep = adaptiveGridSize / (111000.0 * Math.cos(Math.toRadians((minBound.latitude + maxBound.latitude) / 2)));
        
        // 动态限制采样点数量
        int maxSamples = Math.min(1500, Math.max(200, (int)(polygonArea / 100))); // 根据面积动态调整
        double estimatedSamples = ((maxBound.latitude - minBound.latitude) / latStep) * 
                                 ((maxBound.longitude - minBound.longitude) / lngStep);
        
        if (estimatedSamples > maxSamples) {
            double scaleFactor = Math.sqrt(estimatedSamples / maxSamples);
            latStep *= scaleFactor;
            lngStep *= scaleFactor;
        }
        int totalSamples = 0;
        int polygonInteriorSamples = 0;
        int uncoveredSamples = 0;

        // 性能优化：使用并行流处理网格点
        List<LatLngPoint> gridPoints = new ArrayList<>();
        for (double lat = minBound.latitude; lat <= maxBound.latitude; lat += latStep) {
            for (double lng = minBound.longitude; lng <= maxBound.longitude; lng += lngStep) {
                gridPoints.add(new LatLngPoint(lat, lng));
            }
        }
        
        totalSamples = gridPoints.size();
        double coverageRadius = Math.max(photoWidth, photoLength) * 0.25;
        
        // 并行处理网格点
        List<LatLngPoint> validGridPoints = gridPoints.parallelStream()
                .filter(gridPoint -> isPointInPolygon(gridPoint, originalPolygon))
                .collect(Collectors.toList());
        
        polygonInteriorSamples = validGridPoints.size();
        
        uncoveredPoints = validGridPoints.parallelStream()
                .filter(gridPoint -> {
                    // 检查网格点是否被现有航点覆盖
                    return existingWaypoints.stream()
                            .noneMatch(waypoint -> calculateDistance(gridPoint, waypoint) <= coverageRadius);
                })
                .collect(Collectors.toList());
        
        uncoveredSamples = uncoveredPoints.size();

        double coverageRate = polygonInteriorSamples > 0 ?
                (1.0 - (double)uncoveredSamples/polygonInteriorSamples) * 100 : 100.0;

        // 如果未覆盖点太少或覆盖率已经很高，返回空列表
        if (uncoveredSamples < 2 || coverageRate > 98.0) {
            return new ArrayList<>();
        }

        // 直接返回未覆盖点，用于生成补充航点
        return uncoveredPoints.size() > 10 ? uncoveredPoints.subList(0, Math.min(50, uncoveredPoints.size())) : uncoveredPoints;
    }

    /**
     * 计算点集的重心
     */
    private static LatLngPoint calculateCentroid(List<LatLngPoint> points) {
        double sumLat = 0, sumLng = 0;
        for (LatLngPoint point : points) {
            sumLat += point.latitude;
            sumLng += point.longitude;
        }
        return new LatLngPoint(sumLat / points.size(), sumLng / points.size());
    }

    /**
     * 计算地面采样距离(GSD)
     * 使用GSD公式：GSD = (传感器宽度 × 飞行高度) / (焦距 × 图像宽度)
     * 
     * @param sensorWidth 传感器宽度（毫米）
     * @param height 飞行高度（米）
     * @param focalLength 焦距（毫米）
     * @param imageWidth 图像宽度（像素）
     * @return 地面采样距离GSD（米）
     */
    public static double calculateGSD(double sensorWidth, double height, double focalLength, int imageWidth) {
        if (sensorWidth <= 0 || height <= 0 || focalLength <= 0 || imageWidth <= 0) {
            throw new IllegalArgumentException("传感器宽度、飞行高度、焦距和图像宽度必须大于0");
        }
        
        // GSD = (传感器宽度 × 飞行高度) / (焦距 × 图像宽度)
        return (sensorWidth * height) / (focalLength * imageWidth);
    }
    /**
     * 根据地面采样距离(GSD)计算飞行高度
     * 使用反向GSD公式：高度 = (GSD × 焦距 × 图像宽度) / 传感器宽度
     *
     * @param gsd 地面采样距离（米）
     * @param sensorWidth 传感器宽度（毫米）
     * @param focalLength 焦距（毫米）
     * @param imageWidth 图像宽度（像素）
     * @return 飞行高度（米）
     */
    public static double calculateHeightFromGSD(double gsd, double sensorWidth, double focalLength, int imageWidth) {
        if (gsd <= 0 || sensorWidth <= 0 || focalLength <= 0 || imageWidth <= 0) {
            throw new IllegalArgumentException("GSD、传感器宽度、焦距和图像宽度必须大于0");
        }

        // 高度 = (GSD × 焦距 × 图像宽度) / 传感器宽度
        return (gsd * focalLength * imageWidth) / sensorWidth;
    }
    
    /**
     * 计算飞行高度
     * 使用GSD公式：高度 = (GSD × 焦距 × 图像宽度) / 传感器宽度
     * 
     * @param gsd 地面采样距离（米）
     * @param focalLength 焦距（毫米）
     * @param imageWidth 图像宽度（像素）
     * @param sensorWidth 传感器宽度（毫米）
     * @return 飞行高度（米）
     */
    public static double calculateFlightHeight(double gsd, double focalLength, int imageWidth, double sensorWidth) {
        if (sensorWidth <= 0 || focalLength <= 0 || imageWidth <= 0) {
            throw new IllegalArgumentException("传感器宽度、焦距和图像宽度必须大于0");
        }
        
        // 高度 = (GSD × 焦距 × 图像宽度) / 传感器宽度
        return (gsd * focalLength * imageWidth) / sensorWidth;
    }
    
    /**
     * 计算单张照片覆盖宽度
     * 公式：单张照片覆盖宽度 = (传感器宽度 × 高度) / 焦距
     * 
     * @param sensorWidth 传感器宽度（毫米）
     * @param height 飞行高度（米）
     * @param focalLength 焦距（毫米）
     * @return 单张照片覆盖宽度（米）
     */
    public static double calculatePhotoWidth(double sensorWidth, double height, double focalLength) {
        if (sensorWidth <= 0 || height <= 0 || focalLength <= 0) {
            throw new IllegalArgumentException("传感器宽度、飞行高度和焦距必须大于0");
        }
        
        // 单张照片覆盖宽度 = (传感器宽度 × 高度) / 焦距
        return (sensorWidth * height) / focalLength;
    }
    
    /**
     * 计算单张照片覆盖长度
     * 公式：单张照片覆盖长度 = (传感器高度 × 高度) / 焦距
     * 
     * @param sensorHeight 传感器高度（毫米）
     * @param height 飞行高度（米）
     * @param focalLength 焦距（毫米）
     * @return 单张照片覆盖长度（米）
     */
    public static double calculatePhotoLength(double sensorHeight, double height, double focalLength) {
        if (sensorHeight <= 0 || height <= 0 || focalLength <= 0) {
            throw new IllegalArgumentException("传感器高度、飞行高度和焦距必须大于0");
        }
        
        // 单张照片覆盖长度 = (传感器高度 × 高度) / 焦距
        return (sensorHeight * height) / focalLength;
    }

    /**
     * 改进的连接到现有路线方法 - 增加交叉检测
     */
    private static List<LatLngPoint> connectToExistingRoute(List<LatLngPoint> existingRoute,
                                                            List<LatLngPoint> newSegment,
                                                            List<LatLngPoint> polygon) {
        if (existingRoute.isEmpty() || newSegment.isEmpty()) {
            return newSegment;
        }

        List<LatLngPoint> connectedSegment = new ArrayList<>();
        LatLngPoint lastExistingPoint = existingRoute.get(existingRoute.size() - 1);
        LatLngPoint firstNewPoint = newSegment.get(0);

        // 检查直接连接是否安全且不产生交叉
        if (isLineInPolygon(lastExistingPoint, firstNewPoint, polygon) &&
                !hasLineIntersection(lastExistingPoint, firstNewPoint, existingRoute)) {
            connectedSegment.addAll(newSegment);
        } else {
            // 寻找安全的桥接路径，避免交叉
            List<LatLngPoint> bridgePath = findNonIntersectingBridgePath(
                    lastExistingPoint, firstNewPoint, polygon, existingRoute);

            if (bridgePath != null && !bridgePath.isEmpty()) {
                connectedSegment.addAll(bridgePath);
                connectedSegment.addAll(newSegment.subList(1, newSegment.size()));
            } else {
                // 如果无法安全连接，创建独立的航线段
                connectedSegment.addAll(newSegment);
            }
        }

        return connectedSegment;
    }

    /**
     * 计算连接距离
     */
    private static double calculateConnectionDistance(List<LatLngPoint> existingRoute,
                                                      List<LatLngPoint> newSegment) {
        if (existingRoute.size() < 2 || newSegment.isEmpty()) {
            return 0;
        }

        LatLngPoint lastExisting = existingRoute.get(existingRoute.size() - newSegment.size() - 1);
        LatLngPoint firstNew = newSegment.get(0);

        return calculateDistance(lastExisting, firstNew);
    }



    /**
     * 获取多边形边界（带缓存）
     */
    private static LatLngPoint[] getBounds(List<LatLngPoint> polygon) {
        // 生成缓存键
        String cacheKey = String.valueOf(polygon.hashCode());
        
        // 检查缓存
        LatLngPoint[] cached = boundsCache.get(cacheKey);
        if (cached != null) {
            return cached;
        }
        
        double minLat = Double.MAX_VALUE, maxLat = Double.MIN_VALUE;
        double minLng = Double.MAX_VALUE, maxLng = Double.MIN_VALUE;

        for (LatLngPoint point : polygon) {
            minLat = Math.min(minLat, point.latitude);
            maxLat = Math.max(maxLat, point.latitude);
            minLng = Math.min(minLng, point.longitude);
            maxLng = Math.max(maxLng, point.longitude);
        }

        LatLngPoint[] result = new LatLngPoint[]{new LatLngPoint(minLat, minLng), new LatLngPoint(maxLat, maxLng)};
        
        // 存储到缓存
        boundsCache.put(cacheKey, result);
        
        return result;
    }

    /**
     * 判断点是否在多边形内部（改进的射线法，带缓存）
     */
    private static boolean isPointInPolygon(LatLngPoint point, List<LatLngPoint> polygon) {
        if (polygon.size() < 3) {
            return false;
        }
        
        // 缓存清理
        cleanupCachesIfNeeded();
        
        // 生成缓存键
        String cacheKey = String.format("%.8f,%.8f,%d", 
            point.latitude, point.longitude, polygon.hashCode());
        
        // 检查缓存
        Boolean cached = pointInPolygonCache.get(cacheKey);
        if (cached != null) {
            return cached;
        }

        int intersections = 0;
        int n = polygon.size();

        for (int i = 0; i < n; i++) {
            LatLngPoint p1 = polygon.get(i);
            LatLngPoint p2 = polygon.get((i + 1) % n);

            // 跳过水平边
            if (Math.abs(p1.latitude - p2.latitude) < 1e-10) {
                continue;
            }

            // 确保p1.latitude < p2.latitude
            if (p1.latitude > p2.latitude) {
                LatLngPoint temp = p1;
                p1 = p2;
                p2 = temp;
            }

            // 检查点是否在边的纵向范围内
            if (point.latitude < p1.latitude || point.latitude >= p2.latitude) {
                continue;
            }

            // 计算射线与边的交点的横坐标
            double intersectionX = p1.longitude +
                    (point.latitude - p1.latitude) * (p2.longitude - p1.longitude) / (p2.latitude - p1.latitude);

            // 如果交点在射线右侧，计数加1
            if (intersectionX > point.longitude) {
                intersections++;
            }
        }

        boolean result = (intersections % 2) == 1;
        
        // 存储到缓存
        pointInPolygonCache.put(cacheKey, result);
        
        return result;
    }

    /**
     * 判断线段是否完全在多边形内部（优化版本）
     */
    private static boolean isLineInPolygon(LatLngPoint start, LatLngPoint end, List<LatLngPoint> polygon) {
        // 检查起点和终点
        if (!isPointInPolygon(start, polygon) || !isPointInPolygon(end, polygon)) {
            return false;
        }

        // 优化：减少采样点数量，提高性能
        double lineLength = calculateDistance(start, end);
        int sampleCount = Math.max(3, Math.min(15, (int)(lineLength / 50))); // 动态采样点数量，最多15个
        
        for (int i = 1; i < sampleCount; i++) {
            double ratio = (double) i / sampleCount;
            LatLngPoint samplePoint = interpolatePoint(start, end, ratio);
            if (!isPointInPolygon(samplePoint, polygon)) {
                return false;
            }
        }

        // 检查线段与多边形边界的相交
        int n = polygon.size();
        for (int i = 0; i < n; i++) {
            LatLngPoint p1 = polygon.get(i);
            LatLngPoint p2 = polygon.get((i + 1) % n);

            if (doLinesIntersect(start, end, p1, p2)) {
                return false;
            }
        }

        return true;
    }

    /**
     * 严格判断线段是否在多边形内（高性能优化版本）
     */
    private static boolean isLineStrictlyInPolygon(LatLngPoint start, LatLngPoint end, List<LatLngPoint> polygon) {
        // 首先检查端点
        if (!isPointInPolygon(start, polygon) || !isPointInPolygon(end, polygon)) {
            return false;
        }

        // 大幅减少采样点数量，基于线段长度动态调整
        double lineLength = calculateDistance(start, end);
        int sampleCount = Math.max(2, Math.min(8, (int)(lineLength / 100))); // 最多8个采样点
        
        for (int i = 1; i < sampleCount; i++) {
            double ratio = (double) i / sampleCount;
            LatLngPoint samplePoint = interpolatePoint(start, end, ratio);
            if (!isPointInPolygon(samplePoint, polygon)) {
                return false;
            }
        }

        // 简化边界相交检查
        for (int i = 0; i < polygon.size(); i++) {
            LatLngPoint p1 = polygon.get(i);
            LatLngPoint p2 = polygon.get((i + 1) % polygon.size());

            if (doLinesIntersect(start, end, p1, p2)) {
                return false;
            }
        }

        return true;
    }

    /**
     * 检查线段是否与现有路线产生交叉
     */
    private static boolean hasLineIntersection(LatLngPoint start, LatLngPoint end,
                                               List<LatLngPoint> existingRoute) {
        if (existingRoute.size() < 2) {
            return false;
        }

        for (int i = 0; i < existingRoute.size() - 1; i++) {
            LatLngPoint segStart = existingRoute.get(i);
            LatLngPoint segEnd = existingRoute.get(i + 1);

            if (doLinesIntersect(start, end, segStart, segEnd)) {
                return true;
            }
        }
        return false;
    }

    /**
     * 检查两条线段是否相交
     */
    private static boolean doLinesIntersect(LatLngPoint p1, LatLngPoint q1,
                                            LatLngPoint p2, LatLngPoint q2) {
        // 使用向量叉积判断线段相交
        double d1 = crossProduct(p2, q2, p1);
        double d2 = crossProduct(p2, q2, q1);
        double d3 = crossProduct(p1, q1, p2);
        double d4 = crossProduct(p1, q1, q2);

        if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
                ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) {
            return true;
        }

        // 检查共线情况
        if (d1 == 0 && isPointOnSegment(p2, p1, q2)) {
            return true;
        }
        if (d2 == 0 && isPointOnSegment(p2, q1, q2)) {
            return true;
        }
        if (d3 == 0 && isPointOnSegment(p1, p2, q1)) {
            return true;
        }
        if (d4 == 0 && isPointOnSegment(p1, q2, q1)) {
            return true;
        }

        return false;
    }

    /**
     * 计算向量叉积
     */
    private static double crossProduct(LatLngPoint o, LatLngPoint a, LatLngPoint b) {
        return (a.latitude - o.latitude) * (b.longitude - o.longitude) -
                (a.longitude - o.longitude) * (b.latitude - o.latitude);
    }

    /**
     * 检查点是否在线段上
     */
    private static boolean isPointOnSegment(LatLngPoint p, LatLngPoint q, LatLngPoint r) {
        return q.latitude <= Math.max(p.latitude, r.latitude) &&
                q.latitude >= Math.min(p.latitude, r.latitude) &&
                q.longitude <= Math.max(p.longitude, r.longitude) &&
                q.longitude >= Math.min(p.longitude, r.longitude);
    }

    /**
     * 寻找不产生交叉的桥接路径
     */
    private static List<LatLngPoint> findNonIntersectingBridgePath(LatLngPoint start, LatLngPoint end,
                                                                   List<LatLngPoint> polygon,
                                                                   List<LatLngPoint> existingRoute) {
        List<LatLngPoint> bridgePath = new ArrayList<>();

        // 计算多边形重心作为候选中转点
        LatLngPoint centroid = calculateCentroid(polygon);

        // 检查通过重心的路径是否安全且不交叉
        if (isPointInPolygon(centroid, polygon) &&
                isLineInPolygon(start, centroid, polygon) &&
                isLineInPolygon(centroid, end, polygon) &&
                !hasLineIntersection(start, centroid, existingRoute) &&
                !hasLineIntersection(centroid, end, existingRoute)) {
            bridgePath.add(centroid);
            return bridgePath;
        }

        // 尝试其他安全的中转点
        List<LatLngPoint> candidates = generateSafeBridgeCandidates(start, end, polygon, centroid);

        for (LatLngPoint candidate : candidates) {
            if (isPointInPolygon(candidate, polygon) &&
                    isLineInPolygon(start, candidate, polygon) &&
                    isLineInPolygon(candidate, end, polygon) &&
                    !hasLineIntersection(start, candidate, existingRoute) &&
                    !hasLineIntersection(candidate, end, existingRoute)) {
                bridgePath.add(candidate);
                return bridgePath;
            }
        }

        return bridgePath; // 返回空列表表示无法找到安全路径
    }

    /**
     * 生成安全的桥接候选点
     */
    private static List<LatLngPoint> generateSafeBridgeCandidates(LatLngPoint start, LatLngPoint end,
                                                                  List<LatLngPoint> polygon,
                                                                  LatLngPoint centroid) {
        List<LatLngPoint> candidates = new ArrayList<>();

        // 在起点和终点之间生成候选点，并向重心方向偏移
        for (int i = 1; i <= 4; i++) {
            double ratio = i / 5.0;
            LatLngPoint midPoint = interpolatePoint(start, end, ratio);

            // 向重心方向偏移不同程度
            for (double offset : new double[]{0.1, 0.2, 0.3}) {
                LatLngPoint candidate = interpolatePoint(midPoint, centroid, offset);
                candidates.add(candidate);
            }
        }

        return candidates;
    }



    /**
     * 找到多边形的进入点
     */
    private static LatLngPoint findPolygonEntry(List<LatLngPoint> polygon, LatLngPoint startPoint, double directionRad) {
        if (isPointInPolygon(startPoint, polygon)) {
            return startPoint;
        }
        return null;
    }

    /**
     * 找到最近的进入点
     */
    private static LatLngPoint findNearestEntryPoint(List<LatLngPoint> polygon, LatLngPoint startPoint, double directionRad) {
        LatLngPoint nearestPoint = null;
        double minDistance = Double.MAX_VALUE;

        // 在多边形边界上找最近点
        int n = polygon.size();
        for (int i = 0; i < n; i++) {
            LatLngPoint p1 = polygon.get(i);
            LatLngPoint p2 = polygon.get((i + 1) % n);

            LatLngPoint closestPoint = getClosestPointOnSegment(startPoint, p1, p2);
            double distance = calculateDistance(startPoint, closestPoint);

            if (distance < minDistance) {
                minDistance = distance;
                nearestPoint = closestPoint;
            }
        }

        return nearestPoint;
    }

    /**
     * 获取点到线段的最近点
     */
    private static LatLngPoint getClosestPointOnSegment(LatLngPoint point, LatLngPoint segStart, LatLngPoint segEnd) {
        double dx = segEnd.longitude - segStart.longitude;
        double dy = segEnd.latitude - segStart.latitude;

        if (dx == 0 && dy == 0) {
            return segStart;
        }

        double t = ((point.longitude - segStart.longitude) * dx + (point.latitude - segStart.latitude) * dy) / (dx * dx + dy * dy);
        t = Math.max(0, Math.min(1, t));

        return new LatLngPoint(
                segStart.latitude + t * dy,
                segStart.longitude + t * dx
        );
    }

    /**
     * 生成平行航线（改进版本）
     */
    private static List<List<LatLngPoint>> generateParallelLines(List<LatLngPoint> polygon,
                                                                 LatLngPoint entryPoint,
                                                                 double directionRad,
                                                                 double perpDirRad,
                                                                 double lineSpacing,
                                                                 double pointSpacing) {
        List<List<LatLngPoint>> lines = new ArrayList<>();

        // 获取多边形边界
        LatLngPoint[] bounds = getBounds(polygon);
        double maxDimension = Math.max(
                calculateDistance(new LatLngPoint(bounds[0].latitude, bounds[0].longitude),
                        new LatLngPoint(bounds[1].latitude, bounds[0].longitude)),
                calculateDistance(new LatLngPoint(bounds[0].latitude, bounds[0].longitude),
                        new LatLngPoint(bounds[0].latitude, bounds[1].longitude))
        );

        // 计算多边形重心作为参考点
        double centerLat = 0, centerLng = 0;
        for (LatLngPoint point : polygon) {
            centerLat += point.latitude;
            centerLng += point.longitude;
        }
        centerLat /= polygon.size();
        centerLng /= polygon.size();
        LatLngPoint center = new LatLngPoint(centerLat, centerLng);

        // 使用重心作为基准点，而不是entryPoint
        LatLngPoint basePoint = isPointInPolygon(entryPoint, polygon) ? entryPoint : center;
        
        // 性能优化：动态计算最大航线数，避免过度生成
        int maxLines = Math.min(100, Math.max(20, (int)(maxDimension / lineSpacing) + 10)); // 动态调整最大航线数
        
        // 多线程并行生成航线
        List<Integer> lineIndices = IntStream.rangeClosed(-maxLines/2, maxLines/2)
                .boxed().collect(Collectors.toList());
        
        List<List<LatLngPoint>> parallelLines = lineIndices.parallelStream()
                .map(i -> {
                    double offsetDistance = i * lineSpacing;
                    
                    // 计算航线起点和终点，进一步扩大范围
                    LatLngPoint lineStart = offsetPoint(basePoint, perpDirRad, offsetDistance);
                    lineStart = offsetPoint(lineStart, directionRad, -maxDimension * 2.0);
                    
                    LatLngPoint lineEnd = offsetPoint(lineStart, directionRad, 4 * maxDimension);
                    
                    // 生成该航线上的航点
                    return generateLinePoints(polygon, lineStart, lineEnd, directionRad, pointSpacing);
                })
                .filter(linePoints -> !linePoints.isEmpty())
                .collect(Collectors.toList());
        
        lines.addAll(parallelLines);
        int validLineCount = parallelLines.size();


        // 如果航线数量太少，尝试不同的方向生成补充航线
        if (validLineCount < 3) {

            // 尝试垂直方向的航线
            double perpDirection = directionRad + Math.PI / 2;
            double perpPerpDirection = perpDirection + Math.PI / 2;

            for (int i = -20; i <= 20; i++) {
                double offsetDistance = i * lineSpacing * 0.5; // 使用更小的间距

                LatLngPoint lineStart = offsetPoint(basePoint, perpPerpDirection, offsetDistance);
                lineStart = offsetPoint(lineStart, perpDirection, -maxDimension * 2.0);

                LatLngPoint lineEnd = offsetPoint(lineStart, perpDirection, 4 * maxDimension);

                List<LatLngPoint> linePoints = generateLinePoints(polygon, lineStart, lineEnd, perpDirection, pointSpacing);

                if (!linePoints.isEmpty()) {
                    lines.add(linePoints);
                    validLineCount++;
                }
            }

        }

        return lines;
    }

    /**
     * 在指定方向上偏移点
     */
    private static LatLngPoint offsetPoint(LatLngPoint point, double bearingRad, double distanceM) {
        double lat1 = Math.toRadians(point.latitude);
        double lng1 = Math.toRadians(point.longitude);

        double lat2 = Math.asin(Math.sin(lat1) * Math.cos(distanceM / EARTH_RADIUS) +
                Math.cos(lat1) * Math.sin(distanceM / EARTH_RADIUS) * Math.cos(bearingRad));

        double lng2 = lng1 + Math.atan2(Math.sin(bearingRad) * Math.sin(distanceM / EARTH_RADIUS) * Math.cos(lat1),
                Math.cos(distanceM / EARTH_RADIUS) - Math.sin(lat1) * Math.sin(lat2));

        return new LatLngPoint(Math.toDegrees(lat2), Math.toDegrees(lng2));
    }

    /**
     * 生成航线上的航点（严格版本）
     */
    private static List<LatLngPoint> generateLinePoints(List<LatLngPoint> polygon,
                                                        LatLngPoint lineStart,
                                                        LatLngPoint lineEnd,
                                                        double directionRad,
                                                        double pointSpacing) {
        List<LatLngPoint> points = new ArrayList<>();

        // 找到航线与多边形的交点
        List<LatLngPoint> intersections = findLinePolygonIntersections(lineStart, lineEnd, polygon);

        if (intersections.size() < 2) {
            return points; // 航线不与多边形相交
        }

        // 去重并按照航线方向排序交点
        Set<String> uniquePoints = new HashSet<>();
        List<LatLngPoint> filteredIntersections = new ArrayList<>();

        for (LatLngPoint point : intersections) {
            String key = String.format("%.8f,%.8f", point.latitude, point.longitude);
            if (!uniquePoints.contains(key)) {
                uniquePoints.add(key);
                filteredIntersections.add(point);
            }
        }

        filteredIntersections.sort((p1, p2) -> {
            double dist1 = calculateDistance(lineStart, p1);
            double dist2 = calculateDistance(lineStart, p2);
            return Double.compare(dist1, dist2);
        });

        // 在每对交点之间生成航点，使用更严格的验证
        for (int i = 0; i < filteredIntersections.size() - 1; i += 2) {
            LatLngPoint segStart = filteredIntersections.get(i);
            LatLngPoint segEnd = filteredIntersections.get(i + 1);

            // 增加收缩比例，确保航线完全安全
            double shrinkRatio = 0.02; // 收缩2%，确保安全性
            LatLngPoint adjustedStart = interpolatePoint(segStart, segEnd, shrinkRatio);
            LatLngPoint adjustedEnd = interpolatePoint(segEnd, segStart, shrinkRatio);

            // 使用严格的线段验证，确保无无效航线
            if (!isLineStrictlyInPolygon(adjustedStart, adjustedEnd, polygon)) {
                continue; // 跳过无效线段
            }

            double segDistance = calculateDistance(adjustedStart, adjustedEnd);
            int numPoints = Math.max(2, (int) Math.ceil(segDistance / pointSpacing) + 1);

            List<LatLngPoint> segmentPoints = new ArrayList<>();
            LatLngPoint lastValidPoint = null;

            for (int j = 0; j < numPoints; j++) {
                double ratio = (double) j / (numPoints - 1);
                LatLngPoint candidate = interpolatePoint(adjustedStart, adjustedEnd, ratio);

                if (isPointInPolygon(candidate, polygon)) {
                    // 如果是第一个点，直接添加
                    if (lastValidPoint == null) {
                        segmentPoints.add(candidate);
                        lastValidPoint = candidate;
                    } else {
                        // 严格验证与上一个点的连线
                        if (isLineStrictlyInPolygon(lastValidPoint, candidate, polygon)) {
                            segmentPoints.add(candidate);
                            lastValidPoint = candidate;
                        } else {
                            // 如果连线不安全，尝试找到安全的中间点
                            LatLngPoint safePoint = findSafeIntermediatePoint(lastValidPoint, candidate, polygon);
                            if (safePoint != null && isLineStrictlyInPolygon(lastValidPoint, safePoint, polygon)) {
                                segmentPoints.add(safePoint);
                                lastValidPoint = safePoint;
                                // 再次尝试添加原候选点
                                if (isLineStrictlyInPolygon(lastValidPoint, candidate, polygon)) {
                                    segmentPoints.add(candidate);
                                    lastValidPoint = candidate;
                                }
                            }
                        }
                    }
                }
            }

            // 最终验证整个线段的连续性（使用严格验证）
            boolean validSegment = true;
            for (int k = 1; k < segmentPoints.size(); k++) {
                if (!isLineStrictlyInPolygon(segmentPoints.get(k-1), segmentPoints.get(k), polygon)) {
                    validSegment = false;
                    break;
                }
            }

            if (validSegment && segmentPoints.size() >= 2) {
                points.addAll(segmentPoints);
            }
        }

        return points;
    }

    /**
     * 在两点之间找到安全的中间点
     */
    private static LatLngPoint findSafeIntermediatePoint(LatLngPoint start, LatLngPoint end, List<LatLngPoint> polygon) {
        // 尝试在中点位置找到安全点
        LatLngPoint midPoint = interpolatePoint(start, end, 0.5);

        if (isPointInPolygon(midPoint, polygon)) {
            // 尝试向多边形重心方向微调
            double centerLat = 0, centerLng = 0;
            for (LatLngPoint point : polygon) {
                centerLat += point.latitude;
                centerLng += point.longitude;
            }
            centerLat /= polygon.size();
            centerLng /= polygon.size();
            LatLngPoint centroid = new LatLngPoint(centerLat, centerLng);

            // 向重心方向偏移一小段距离
            LatLngPoint adjustedPoint = interpolatePoint(midPoint, centroid, 0.1);

            if (isPointInPolygon(adjustedPoint, polygon)) {
                return adjustedPoint;
            }

            return midPoint;
        }

        return null;
    }

    /**
     * 找到直线与多边形的交点（带缓存）
     */
    private static List<LatLngPoint> findLinePolygonIntersections(LatLngPoint lineStart, LatLngPoint lineEnd, List<LatLngPoint> polygon) {
        // 生成缓存键
        String cacheKey = String.format("%.6f,%.6f,%.6f,%.6f,%d", 
            lineStart.latitude, lineStart.longitude, lineEnd.latitude, lineEnd.longitude, polygon.hashCode());
        
        // 检查缓存
        List<LatLngPoint> cached = intersectionCache.get(cacheKey);
        if (cached != null) {
            return new ArrayList<>(cached); // 返回副本避免修改缓存
        }
        
        List<LatLngPoint> intersections = new ArrayList<>();
        int n = polygon.size();

        for (int i = 0; i < n; i++) {
            LatLngPoint p1 = polygon.get(i);
            LatLngPoint p2 = polygon.get((i + 1) % n);

            LatLngPoint intersection = getLineIntersection(lineStart, lineEnd, p1, p2);
            if (intersection != null) {
                intersections.add(intersection);
            }
        }
        
        // 存储到缓存
        intersectionCache.put(cacheKey, new ArrayList<>(intersections));

        return intersections;
    }

    /**
     * 计算两条直线的交点
     */
    private static LatLngPoint getLineIntersection(LatLngPoint p1, LatLngPoint p2, LatLngPoint p3, LatLngPoint p4) {
        double x1 = p1.longitude, y1 = p1.latitude;
        double x2 = p2.longitude, y2 = p2.latitude;
        double x3 = p3.longitude, y3 = p3.latitude;
        double x4 = p4.longitude, y4 = p4.latitude;

        double denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (Math.abs(denom) < 1e-10) {
            return null; // 平行线
        }

        double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom;
        double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom;

        if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
            double x = x1 + t * (x2 - x1);
            double y = y1 + t * (y2 - y1);
            return new LatLngPoint(y, x);
        }

        return null;
    }

    /**
     * 插值计算两点之间的点
     */
    private static LatLngPoint interpolatePoint(LatLngPoint start, LatLngPoint end, double ratio) {
        double lat = start.latitude + ratio * (end.latitude - start.latitude);
        double lng = start.longitude + ratio * (end.longitude - start.longitude);
        return new LatLngPoint(lat, lng);
    }

    /**
     * 优化航线顺序以最小化总距离（蛇形路径优化版本）
     */
    private static List<LatLngPoint> optimizeRouteOrder(List<List<LatLngPoint>> lines, LatLngPoint startPoint, List<LatLngPoint> polygon) {
        List<LatLngPoint> result = new ArrayList<>();

        if (lines.isEmpty()) {
            return result;
        }

        // 过滤掉空的航线
        List<List<LatLngPoint>> validLines = new ArrayList<>();
        for (List<LatLngPoint> line : lines) {
            if (!line.isEmpty()) {
                validLines.add(line);
            }
        }

        if (validLines.isEmpty()) {
            return result;
        }

        // 实现蛇形路径规划：按航线的空间顺序排列，然后交替反向
        return generateSnakePathRoute(validLines, startPoint, polygon);
    }

    /**
     * 生成蛇形路径航线（核心优化方法）
     */
    private static List<LatLngPoint> generateSnakePathRoute(List<List<LatLngPoint>> validLines, LatLngPoint startPoint, List<LatLngPoint> polygon) {
        List<LatLngPoint> result = new ArrayList<>();

        if (validLines.isEmpty()) {
            return result;
        }

        // 第一步：按航线的空间位置排序（基于航线中点的位置）
        List<LineWithIndex> linesWithIndex = new ArrayList<>();
        for (int i = 0; i < validLines.size(); i++) {
            List<LatLngPoint> line = validLines.get(i);
            LatLngPoint midPoint = calculateLineMidPoint(line);
            linesWithIndex.add(new LineWithIndex(line, i, midPoint));
        }

        // 根据航线方向确定排序方式
        sortLinesByPosition(linesWithIndex, startPoint);

        // 第二步：实现蛇形连接
        boolean reverseDirection = false;

        for (int i = 0; i < linesWithIndex.size(); i++) {
            List<LatLngPoint> currentLine = linesWithIndex.get(i).line;

            // 根据当前方向决定航线的飞行方向
            List<LatLngPoint> orderedLine;
            if (reverseDirection) {
                // 反向飞行：从航线末端到起点
                orderedLine = new ArrayList<>();
                for (int j = currentLine.size() - 1; j >= 0; j--) {
                    orderedLine.add(currentLine.get(j));
                }
            } else {
                // 正向飞行：从航线起点到末端
                orderedLine = new ArrayList<>(currentLine);
            }

            // 添加连接路径（如果需要）
            if (!result.isEmpty()) {
                LatLngPoint lastPoint = result.get(result.size() - 1);
                LatLngPoint nextPoint = orderedLine.get(0);

                // 检查是否需要桥接路径
                if (!isLineStrictlyInPolygon(lastPoint, nextPoint, polygon)) {
                    List<LatLngPoint> bridgePath = findPerfectSafeBridgePath(lastPoint, nextPoint, polygon);
                    if (!bridgePath.isEmpty()) {
                        result.addAll(bridgePath);
                    }
                }
            }

            // 添加当前航线的航点
            result.addAll(orderedLine);


            // 切换方向，实现蛇形路径
            reverseDirection = !reverseDirection;
        }

        // 最终清理：移除所有无效的连接
        return cleanupInvalidConnections(result, polygon);
    }

    /**
     * 航线与索引的包装类
     */
    private static class LineWithIndex {
        List<LatLngPoint> line;
        int originalIndex;
        LatLngPoint midPoint;

        LineWithIndex(List<LatLngPoint> line, int originalIndex, LatLngPoint midPoint) {
            this.line = line;
            this.originalIndex = originalIndex;
            this.midPoint = midPoint;
        }
    }

    /**
     * 计算航线的中点
     */
    private static LatLngPoint calculateLineMidPoint(List<LatLngPoint> line) {
        if (line.isEmpty()) {
            return new LatLngPoint(0, 0);
        }
        if (line.size() == 1) {
            return line.get(0);
        }

        int midIndex = line.size() / 2;
        return line.get(midIndex);
    }

    /**
     * 按位置对航线进行排序
     */
    private static void sortLinesByPosition(List<LineWithIndex> linesWithIndex, LatLngPoint startPoint) {
        if (linesWithIndex.size() <= 1) {
            return;
        }

        // 计算主要飞行方向（基于前两条航线的位置关系）
        LatLngPoint first = linesWithIndex.get(0).midPoint;
        LatLngPoint second = linesWithIndex.get(1).midPoint;

        double deltaLat = Math.abs(second.latitude - first.latitude);
        double deltaLng = Math.abs(second.longitude - first.longitude);

        // 根据主要变化方向进行排序
        if (deltaLat > deltaLng) {
            // 主要是南北方向的航线，按纬度排序
            linesWithIndex.sort((a, b) -> Double.compare(a.midPoint.latitude, b.midPoint.latitude));
        } else {
            // 主要是东西方向的航线，按经度排序
            linesWithIndex.sort((a, b) -> Double.compare(a.midPoint.longitude, b.midPoint.longitude));
        }

        // 确保从离起始点最近的航线开始
        if (!linesWithIndex.isEmpty()) {
            double distToFirst = calculateDistance(startPoint, linesWithIndex.get(0).midPoint);
            double distToLast = calculateDistance(startPoint, linesWithIndex.get(linesWithIndex.size() - 1).midPoint);

            // 如果起始点离最后一条航线更近，则反转整个顺序
            if (distToLast < distToFirst) {
                Collections.reverse(linesWithIndex);
            }
        }
    }

    /**
     * 清理无效连接，确保所有航线段都是安全的
     */
    private static List<LatLngPoint> cleanupInvalidConnections(List<LatLngPoint> waypoints, List<LatLngPoint> polygon) {
        if (waypoints.size() <= 1) {
            return waypoints;
        }

        List<LatLngPoint> cleanedWaypoints = new ArrayList<>();
        cleanedWaypoints.add(waypoints.get(0)); // 添加第一个点

        for (int i = 1; i < waypoints.size(); i++) {
            LatLngPoint currentPoint = waypoints.get(i);
            LatLngPoint lastValidPoint = cleanedWaypoints.get(cleanedWaypoints.size() - 1);

            // 检查连接是否安全
            if (isLineStrictlyInPolygon(lastValidPoint, currentPoint, polygon)) {
                cleanedWaypoints.add(currentPoint);
            } else {
                // 尝试找到安全的桥接路径
                List<LatLngPoint> bridgePath = findPerfectSafeBridgePath(lastValidPoint, currentPoint, polygon);
                if (!bridgePath.isEmpty()) {
                    // 验证桥接路径的每一段都是安全的
                    boolean bridgeValid = true;
                    LatLngPoint bridgeStart = lastValidPoint;

                    for (LatLngPoint bridgePoint : bridgePath) {
                        if (!isLineStrictlyInPolygon(bridgeStart, bridgePoint, polygon)) {
                            bridgeValid = false;
                            break;
                        }
                        bridgeStart = bridgePoint;
                    }

                    // 检查最后一段桥接到目标点
                    if (bridgeValid && isLineStrictlyInPolygon(bridgeStart, currentPoint, polygon)) {
                        cleanedWaypoints.addAll(bridgePath);
                        cleanedWaypoints.add(currentPoint);
                    }

                }
            }
        }
        return cleanedWaypoints;
    }


    /**
     * 找到完美安全的桥接路径（更严格的版本）
     */
    private static List<LatLngPoint> findPerfectSafeBridgePath(LatLngPoint start, LatLngPoint end, List<LatLngPoint> polygon) {
        List<LatLngPoint> bridgePath = new ArrayList<>();

        // 如果直接连接是完全安全的，返回空列表
        if (isLineStrictlyInPolygon(start, end, polygon)) {
            return bridgePath;
        }

        // 计算多边形重心
        double centerLat = 0, centerLng = 0;
        for (LatLngPoint point : polygon) {
            centerLat += point.latitude;
            centerLng += point.longitude;
        }
        centerLat /= polygon.size();
        centerLng /= polygon.size();
        LatLngPoint centroid = new LatLngPoint(centerLat, centerLng);

        // 方法1：通过重心连接
        if (isPointInPolygon(centroid, polygon) &&
                isLineStrictlyInPolygon(start, centroid, polygon) &&
                isLineStrictlyInPolygon(centroid, end, polygon)) {
            bridgePath.add(centroid);
            return bridgePath;
        }

        // 方法2：寻找多边形内部的安全中转点（使用更保守的策略）
        List<LatLngPoint> safeCandidates = findSafeIntermediatePoints(start, end, polygon, centroid);

        // 对候选点进行额外的安全边距处理
        List<LatLngPoint> ultraSafeCandidates = new ArrayList<>();
        for (LatLngPoint candidate : safeCandidates) {
            // 向重心方向进一步收缩，增加安全边距
            LatLngPoint saferPoint = interpolatePoint(candidate, centroid, 0.1);
            if (isPointInPolygon(saferPoint, polygon)) {
                ultraSafeCandidates.add(saferPoint);
            }
        }

        // 尝试单点桥接（使用超安全候选点）
        for (LatLngPoint candidate : ultraSafeCandidates) {
            if (isLineStrictlyInPolygon(start, candidate, polygon) &&
                    isLineStrictlyInPolygon(candidate, end, polygon)) {
                bridgePath.add(candidate);
                return bridgePath;
            }
        }

        // 方法3：尝试两点桥接（使用超安全候选点）
        for (int i = 0; i < ultraSafeCandidates.size() && i < 3; i++) { // 限制尝试次数
            for (int j = i + 1; j < ultraSafeCandidates.size() && j < 3; j++) {
                LatLngPoint mid1 = ultraSafeCandidates.get(i);
                LatLngPoint mid2 = ultraSafeCandidates.get(j);

                if (isLineStrictlyInPolygon(start, mid1, polygon) &&
                        isLineStrictlyInPolygon(mid1, mid2, polygon) &&
                        isLineStrictlyInPolygon(mid2, end, polygon)) {
                    bridgePath.add(mid1);
                    bridgePath.add(mid2);
                    return bridgePath;
                }
            }
        }

        return bridgePath; // 无法找到完全安全的路径
    }

    /**
     * 寻找安全的中间点候选
     */
    private static List<LatLngPoint> findSafeIntermediatePoints(LatLngPoint start, LatLngPoint end, List<LatLngPoint> polygon, LatLngPoint centroid) {
        List<LatLngPoint> candidates = new ArrayList<>();

        // 在起点和终点之间生成候选点
        for (int i = 1; i <= 9; i++) {
            double ratio = i / 10.0;
            LatLngPoint directPoint = interpolatePoint(start, end, ratio);

            // 向重心方向偏移不同距离
            for (double offset : new double[]{0.1, 0.2, 0.3, 0.5}) {
                LatLngPoint offsetPoint = interpolatePoint(directPoint, centroid, offset);
                if (isPointInPolygon(offsetPoint, polygon)) {
                    candidates.add(offsetPoint);
                }
            }
        }

        // 在重心周围生成候选点
        double baseDistance = calculateDistance(start, end) * 0.1;
        for (int angle = 0; angle < 360; angle += 30) {
            double angleRad = Math.toRadians(angle);
            for (double dist : new double[]{baseDistance * 0.5, baseDistance, baseDistance * 1.5}) {
                LatLngPoint candidate = offsetPoint(centroid, angleRad, dist);
                if (isPointInPolygon(candidate, polygon)) {
                    candidates.add(candidate);
                }
            }
        }

        return candidates;
    }


    /**
     * 计算两点之间的距离（米）- 优化版本，带缓存
     */
    private static double calculateDistance(LatLngPoint p1, LatLngPoint p2) {
        // 生成缓存键
        String cacheKey = String.format("%.6f,%.6f-%.6f,%.6f", 
            p1.latitude, p1.longitude, p2.latitude, p2.longitude);
        
        // 检查缓存
        Double cachedDistance = distanceCache.get(cacheKey);
        if (cachedDistance != null) {
            return cachedDistance;
        }
        
        // 对于非常近的点，使用简化计算
        double deltaLat = Math.abs(p2.latitude - p1.latitude);
        double deltaLng = Math.abs(p2.longitude - p1.longitude);
        
        if (deltaLat < 0.0001 && deltaLng < 0.0001) {
            // 使用平面距离近似计算（适用于很近的点）
            double latAvg = Math.toRadians((p1.latitude + p2.latitude) / 2);
            double x = Math.toRadians(deltaLng) * Math.cos(latAvg);
            double y = Math.toRadians(deltaLat);
            double distance = Math.sqrt(x * x + y * y) * EARTH_RADIUS;
            
            // 缓存结果
            cacheDistance(cacheKey, distance);
            return distance;
        }
        
        // 使用Haversine公式计算精确距离
        double lat1Rad = Math.toRadians(p1.latitude);
        double lat2Rad = Math.toRadians(p2.latitude);
        double deltaLatRad = Math.toRadians(p2.latitude - p1.latitude);
        double deltaLngRad = Math.toRadians(p2.longitude - p1.longitude);

        double a = Math.sin(deltaLatRad / 2) * Math.sin(deltaLatRad / 2) +
                Math.cos(lat1Rad) * Math.cos(lat2Rad) *
                        Math.sin(deltaLngRad / 2) * Math.sin(deltaLngRad / 2);

        double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
        double distance = EARTH_RADIUS * c;
        
        // 缓存结果
        cacheDistance(cacheKey, distance);
        return distance;
    }
    
    /**
     * 缓存距离计算结果（优化版本）
     */
    private static void cacheDistance(String key, double distance) {
        // 使用更高效的缓存清理策略
        if (distanceCache.size() >= MAX_CACHE_SIZE) {
            // 清理缓存，保留最近使用的
            distanceCache.clear();
        }
        distanceCache.put(key, distance);
    }

    /**
     * 计算路径总距离
     */
    private static double calculateTotalDistance(List<LatLngPoint> waypoints) {
        double totalDistance = 0;
        for (int i = 1; i < waypoints.size(); i++) {
            totalDistance += calculateDistance(waypoints.get(i - 1), waypoints.get(i));
        }
        return totalDistance;
    }

    /**
     * 将航点列表转换为JSON字符串
     * @param waypoints 航点列表
     * @return JSON字符串
     */
    public static String toJson(List<LatLngPoint> waypoints){
        StringBuilder json = new StringBuilder();
        json.append("[");
        for (int i = 0; i < waypoints.size(); i++) {
            if (i > 0) {
                json.append(",");
            }
            LatLngPoint point = waypoints.get(i);
            json.append(String.format("{\"latitude\":%.8f,\"longitude\":%.8f}",
                    point.latitude, point.longitude));
        }
        json.append("]");
        return json.toString();
    }

    /**
     * 找到多边形边界上距离给定点最近的点
     */
    private static LatLngPoint findClosestPointOnPolygon(LatLngPoint point, List<LatLngPoint> polygon) {
        LatLngPoint closestPoint = null;
        double minDistance = Double.MAX_VALUE;

        for (int i = 0; i < polygon.size(); i++) {
            LatLngPoint p1 = polygon.get(i);
            LatLngPoint p2 = polygon.get((i + 1) % polygon.size());

            // 找到线段上距离point最近的点
            LatLngPoint closestOnSegment = findClosestPointOnSegment(point, p1, p2);
            double distance = calculateDistance(point, closestOnSegment);

            if (distance < minDistance) {
                minDistance = distance;
                closestPoint = closestOnSegment;
            }
        }

        return closestPoint;
    }

    /**
     * 找到线段上距离给定点最近的点
     */
    private static LatLngPoint findClosestPointOnSegment(LatLngPoint point, LatLngPoint segStart, LatLngPoint segEnd) {
        double dx = segEnd.longitude - segStart.longitude;
        double dy = segEnd.latitude - segStart.latitude;

        if (dx == 0 && dy == 0) {
            return segStart; // 线段退化为点
        }

        double t = ((point.longitude - segStart.longitude) * dx + (point.latitude - segStart.latitude) * dy) / (dx * dx + dy * dy);

        // 限制t在[0,1]范围内
        t = Math.max(0, Math.min(1, t));

        return new LatLngPoint(
                segStart.latitude + t * dy,
                segStart.longitude + t * dx
        );
    }

    /**
     * 为未覆盖区域生成智能优化的补充航线
     */
    private static List<LatLngPoint> generateSupplementaryWaypoints(List<LatLngPoint> uncoveredPoints,
                                                                    List<LatLngPoint> polygonPoints,
                                                                    double direction,
                                                                    double sideOverlapRate,
                                                                    double forwardOverlapRate,
                                                                    double photoWidth,
                                                                    double photoLength,
                                                                    double flightHeight,
                                                                    List<LatLngPoint> existingWaypoints) {
        if (uncoveredPoints.isEmpty()) {
            return new ArrayList<>();
        }

        // 1. 分析未覆盖区域特征
        RegionAnalysis analysis = analyzeUncoveredRegion(uncoveredPoints, photoWidth, photoLength);
        // 2. 智能方向优化
        double optimizedDirection = optimizeSupplementaryDirection(analysis, direction);
        // 3. 自适应参数调整
        AdaptiveParams adaptiveParams = calculateAdaptiveParams(analysis, sideOverlapRate, forwardOverlapRate, photoWidth, photoLength);
        // 4. 生成精确边界
        List<LatLngPoint> optimizedBoundary = generateOptimizedBoundary(uncoveredPoints, polygonPoints, analysis);

        try {
            // 5. 多策略补充航线生成
            List<LatLngPoint> bestWaypoints = generateMultiStrategyWaypoints(
                    uncoveredPoints, optimizedBoundary, polygonPoints,
                    optimizedDirection, adaptiveParams, flightHeight, existingWaypoints);

            // 6. 重叠检测与优化
            List<LatLngPoint> optimizedWaypoints = removeOverlappingWaypoints(bestWaypoints, uncoveredPoints, adaptiveParams);

            return optimizedWaypoints;

        } catch (Exception e) {
            return generateEnhancedFallbackWaypoints(uncoveredPoints, polygonPoints, adaptiveParams);
        }
    }

    /**
     * 区域分析结果
     */
    private static class RegionAnalysis {
        double area;           // 区域面积（平方米）
        double aspectRatio;    // 长宽比
        double density;        // 点密度
        boolean isScattered;   // 是否为散点分布
        LatLngPoint centroid;  // 重心
        double[] principalAxis; // 主轴方向
    }

    /**
     * 自适应参数
     */
    private static class AdaptiveParams {
        double sideOverlap;
        double forwardOverlap;
        double lineSpacing;
        double pointSpacing;
        double coverageRadius;
    }

    /**
     * 分析未覆盖区域特征
     */
    private static RegionAnalysis analyzeUncoveredRegion(List<LatLngPoint> uncoveredPoints, double photoWidth, double photoLength) {
        RegionAnalysis analysis = new RegionAnalysis();

        // 计算边界框
        LatLngPoint[] bounds = getBounds(uncoveredPoints);
        double latRange = bounds[1].latitude - bounds[0].latitude;
        double lngRange = bounds[1].longitude - bounds[0].longitude;

        // 转换为米制单位进行计算
        double latRangeMeters = latRange * 111000.0;
        double lngRangeMeters = lngRange * 111000.0 * Math.cos(Math.toRadians((bounds[0].latitude + bounds[1].latitude) / 2));

        analysis.area = latRangeMeters * lngRangeMeters;
        analysis.aspectRatio = Math.max(latRangeMeters, lngRangeMeters) / Math.min(latRangeMeters, lngRangeMeters);
        analysis.density = uncoveredPoints.size() / analysis.area * 1000000; // 点/平方公里
        analysis.centroid = calculateCentroid(uncoveredPoints);

        // 判断是否为散点分布
        double avgDistance = calculateAverageDistance(uncoveredPoints);
        double expectedDistance = Math.sqrt(analysis.area / uncoveredPoints.size());
        analysis.isScattered = avgDistance > expectedDistance * 1.5;

        // 计算主轴方向（简化版PCA）
        analysis.principalAxis = calculatePrincipalAxis(uncoveredPoints);

        return analysis;
    }

    /**
     * 优化补充航线方向
     */
    private static double optimizeSupplementaryDirection(RegionAnalysis analysis, double originalDirection) {
        // 直接使用原始方向，确保与主航线方向一致
        return originalDirection;
    }

    /**
     * 计算自适应参数
     */
    private static AdaptiveParams calculateAdaptiveParams(RegionAnalysis analysis, double baseSideOverlap,
                                                          double baseForwardOverlap, double photoWidth, double photoLength) {
        AdaptiveParams params = new AdaptiveParams();

        // 直接使用传入的原始参数，确保与主航线完全一致
        params.sideOverlap = baseSideOverlap;
        params.forwardOverlap = baseForwardOverlap;

        // 使用传入的原始参数计算间距，确保与主航线一致
        params.lineSpacing = photoWidth * (1 - params.sideOverlap / 100.0);
        params.pointSpacing = photoLength * (1 - params.forwardOverlap / 100.0);

        // 计算覆盖半径
        params.coverageRadius = Math.min(photoWidth, photoLength) * 0.6;

        return params;
    }

    /**
     * 生成优化边界
     */
    private static List<LatLngPoint> generateOptimizedBoundary(List<LatLngPoint> uncoveredPoints,
                                                               List<LatLngPoint> polygonPoints, RegionAnalysis analysis) {
        // 如果是散点分布，为每个点生成小的局部边界
        if (analysis.isScattered) {
            return generateScatteredPointsBoundary(uncoveredPoints, analysis);
        }

        // 否则生成凸包边界
        List<LatLngPoint> convexHull = calculateConvexHull(uncoveredPoints);

        // 扩展边界以确保覆盖
        return expandBoundary(convexHull, analysis.area);
    }

    /**
     * 多策略航点生成（使用与主航线完全一致的参数）
     */
    private static List<LatLngPoint> generateMultiStrategyWaypoints(List<LatLngPoint> uncoveredPoints,
                                                                    List<LatLngPoint> boundary,
                                                                    List<LatLngPoint> polygonPoints,
                                                                    double direction,
                                                                    AdaptiveParams params,
                                                                    double flightHeight,
                                                                    List<LatLngPoint> existingWaypoints) {

        // 直接使用严格的局部密集扫描策略，确保参数一致性
        List<LatLngPoint> localWaypoints = generateLocalDenseWaypoints(uncoveredPoints, polygonPoints, params, direction, flightHeight, existingWaypoints);
        return new ArrayList<>(localWaypoints);
    }

    /**
     * 改进的重叠航点移除方法 - 更严格的去重逻辑
     */
    private static List<LatLngPoint> removeOverlappingWaypoints(List<LatLngPoint> waypoints,
                                                                List<LatLngPoint> uncoveredPoints,
                                                                AdaptiveParams params) {
        List<LatLngPoint> optimizedWaypoints = new ArrayList<>();
        // 使用更严格的最小间距：从 coverageRadius * 0.5 改为 pointSpacing * 0.8
        double minDistance = Math.max(params.pointSpacing * 0.8, params.coverageRadius * 0.3);

        for (LatLngPoint waypoint : waypoints) {
            boolean tooClose = false;
            for (LatLngPoint existing : optimizedWaypoints) {
                if (calculateDistance(waypoint, existing) < minDistance) {
                    tooClose = true;
                    break;
                }
            }

            if (!tooClose) {
                // 检查是否真正需要这个航点
                boolean coversUncovered = false;
                for (LatLngPoint uncovered : uncoveredPoints) {
                    if (calculateDistance(waypoint, uncovered) <= params.coverageRadius) {
                        coversUncovered = true;
                        break;
                    }
                }

                if (coversUncovered) {
                    optimizedWaypoints.add(waypoint);
                }
            }
        }

        return optimizedWaypoints;
    }

    /**
     * 增强回退策略
     */
    private static List<LatLngPoint> generateEnhancedFallbackWaypoints(List<LatLngPoint> uncoveredPoints,
                                                                       List<LatLngPoint> polygonPoints,
                                                                       AdaptiveParams params) {
        List<LatLngPoint> fallbackWaypoints = new ArrayList<>();

        // 使用主航线参数生成精确的回退航点
        for (LatLngPoint uncovered : uncoveredPoints) {
            // 使用精确的主航线间距参数
            double stepLat = params.pointSpacing / 111000.0; // 精确转换为纬度步长
            double stepLng = params.lineSpacing / (111000.0 * Math.cos(Math.toRadians(uncovered.latitude))); // 精确转换为经度步长

            // 减小搜索半径，提高精度
            double maxRadius = Math.max(params.lineSpacing, params.pointSpacing) * 1.0;
            int maxSteps = (int)(maxRadius / Math.min(params.lineSpacing, params.pointSpacing)) + 1;

            // 生成精确对齐的网格点
            for (int i = -maxSteps; i <= maxSteps; i++) {
                for (int j = -maxSteps; j <= maxSteps; j++) {
                    LatLngPoint candidate = new LatLngPoint(
                            uncovered.latitude + i * stepLat,
                            uncovered.longitude + j * stepLng
                    );

                    double distance = calculateDistance(candidate, uncovered);
                    if (distance <= maxRadius && isPointInPolygon(candidate, polygonPoints)) {

                        // 使用更严格的距离检查，确保精确间距
                        boolean tooClose = false;
                        double minDistance = Math.min(params.lineSpacing, params.pointSpacing) * 0.4;

                        for (LatLngPoint existing : fallbackWaypoints) {
                            if (calculateDistance(candidate, existing) < minDistance) {
                                tooClose = true;
                                break;
                            }
                        }

                        if (!tooClose) {
                            fallbackWaypoints.add(candidate);
                        }
                    }
                }
            }
        }
        return fallbackWaypoints;
    }

    // 辅助方法实现
    private static double calculateAverageDistance(List<LatLngPoint> points) {
        if (points.size() < 2) {
            return 0;
        }

        double totalDistance = 0;
        int count = 0;

        for (int i = 0; i < points.size(); i++) {
            for (int j = i + 1; j < points.size(); j++) {
                totalDistance += calculateDistance(points.get(i), points.get(j));
                count++;
            }
        }

        return count > 0 ? totalDistance / count : 0;
    }

    private static double[] calculatePrincipalAxis(List<LatLngPoint> points) {
        // 简化的主成分分析
        LatLngPoint centroid = calculateCentroid(points);

        double sumXX = 0, sumYY = 0, sumXY = 0;
        for (LatLngPoint point : points) {
            double x = point.longitude - centroid.longitude;
            double y = point.latitude - centroid.latitude;
            sumXX += x * x;
            sumYY += y * y;
            sumXY += x * y;
        }

        // 计算特征向量（主轴方向）
        double trace = sumXX + sumYY;
        double det = sumXX * sumYY - sumXY * sumXY;
        double lambda1 = (trace + Math.sqrt(trace * trace - 4 * det)) / 2;

        if (Math.abs(sumXY) > 1e-10) {
            return new double[]{lambda1 - sumYY, sumXY};
        } else {
            return new double[]{1, 0}; // 默认水平方向
        }
    }

    private static List<LatLngPoint> calculateConvexHull(List<LatLngPoint> points) {
        // 简化的凸包算法（Graham扫描法）
        if (points.size() < 3) {
            return new ArrayList<>(points);
        }

        // 找到最下方的点
        LatLngPoint bottom = points.get(0);
        for (LatLngPoint point : points) {
            if (point.latitude < bottom.latitude ||
                    (point.latitude == bottom.latitude && point.longitude < bottom.longitude)) {
                bottom = point;
            }
        }

        // 简化实现：返回边界框的四个角点
        LatLngPoint[] bounds = getBounds(points);
        List<LatLngPoint> hull = new ArrayList<>();
        hull.add(bounds[0]);
        hull.add(new LatLngPoint(bounds[0].latitude, bounds[1].longitude));
        hull.add(bounds[1]);
        hull.add(new LatLngPoint(bounds[1].latitude, bounds[0].longitude));

        return hull;
    }

    private static List<LatLngPoint> expandBoundary(List<LatLngPoint> boundary, double area) {
        LatLngPoint centroid = calculateCentroid(boundary);
        double expansionFactor = Math.max(1.1, Math.min(1.5, 1.0 + 1000.0 / area)); // 根据面积调整扩展系数

        List<LatLngPoint> expanded = new ArrayList<>();
        for (LatLngPoint point : boundary) {
            double deltaLat = (point.latitude - centroid.latitude) * expansionFactor;
            double deltaLng = (point.longitude - centroid.longitude) * expansionFactor;
            expanded.add(new LatLngPoint(
                    centroid.latitude + deltaLat,
                    centroid.longitude + deltaLng
            ));
        }

        return expanded;
    }

    private static List<LatLngPoint> generateScatteredPointsBoundary(List<LatLngPoint> uncoveredPoints, RegionAnalysis analysis) {
        // 为散点分布生成多个小的局部边界
        List<LatLngPoint> boundary = new ArrayList<>();
        double radius = Math.sqrt(analysis.area / uncoveredPoints.size()) * 0.5;

        for (LatLngPoint point : uncoveredPoints) {
            // 为每个点生成小的正方形边界
            double latOffset = radius / 111000.0;
            double lngOffset = radius / (111000.0 * Math.cos(Math.toRadians(point.latitude)));

            boundary.add(new LatLngPoint(point.latitude - latOffset, point.longitude - lngOffset));
            boundary.add(new LatLngPoint(point.latitude - latOffset, point.longitude + lngOffset));
            boundary.add(new LatLngPoint(point.latitude + latOffset, point.longitude + lngOffset));
            boundary.add(new LatLngPoint(point.latitude + latOffset, point.longitude - lngOffset));
        }

        return boundary;
    }

    /**
     * 改进的局部密集航点生成 - 增强全局去重
     */
    private static List<LatLngPoint> generateLocalDenseWaypoints(List<LatLngPoint> uncoveredPoints,
                                                                 List<LatLngPoint> polygonPoints,
                                                                 AdaptiveParams params,
                                                                 double direction,
                                                                 double flightHeight,
                                                                 List<LatLngPoint> existingWaypoints) {
        List<LatLngPoint> localWaypoints = new ArrayList<>();
        double clusterRadius = Math.max(params.lineSpacing * 2.0, params.pointSpacing * 4.0);
        List<List<LatLngPoint>> clusters = clusterUncoveredPointsOptimized(uncoveredPoints, clusterRadius, params.lineSpacing);

        // 全局航点管理：包含主航线和所有已生成的补充航点
        Set<String> globalWaypointKeys = new HashSet<>();
        List<LatLngPoint> globalWaypoints = new ArrayList<>(existingWaypoints);

        // 为现有航点建立索引
        for (LatLngPoint waypoint : existingWaypoints) {
            String key = String.format("%.6f,%.6f", waypoint.latitude, waypoint.longitude);
            globalWaypointKeys.add(key);
        }

        for (int i = 0; i < clusters.size(); i++) {
            List<LatLngPoint> cluster = clusters.get(i);

            // 生成候选航点
            List<LatLngPoint> clusterWaypoints = generateConsistentSupplementaryLines(
                    cluster, polygonPoints, params, direction, flightHeight, globalWaypoints);

            // 严格的全局去重检查
            List<LatLngPoint> deduplicatedWaypoints = new ArrayList<>();
            double strictMinDistance = params.pointSpacing * 0.75; // 更严格的最小间距

            for (LatLngPoint candidate : clusterWaypoints) {
                String candidateKey = String.format("%.6f,%.6f", candidate.latitude, candidate.longitude);

                // 检查是否已存在相同位置的航点
                if (globalWaypointKeys.contains(candidateKey)) {
                    continue;
                }

                // 检查与所有现有航点的距离
                boolean tooClose = false;
                for (LatLngPoint existing : globalWaypoints) {
                    if (calculateDistance(candidate, existing) < strictMinDistance) {
                        tooClose = true;
                        break;
                    }
                }

                if (!tooClose) {
                    deduplicatedWaypoints.add(candidate);
                    globalWaypoints.add(candidate);
                    globalWaypointKeys.add(candidateKey);
                }
            }

            localWaypoints.addAll(deduplicatedWaypoints);
        }

        return localWaypoints;
    }

    /**
     * 优化的未覆盖点聚类（高性能版本，减少嵌套循环）
     */
    private static List<List<LatLngPoint>> clusterUncoveredPointsOptimized(List<LatLngPoint> uncoveredPoints,
                                                                           double clusterRadius,
                                                                           double lineSpacing) {
        List<List<LatLngPoint>> clusters = new ArrayList<>();
        
        if (uncoveredPoints.isEmpty()) {
            return clusters;
        }
        
        // 使用空间网格优化邻近点查找
        Map<String, List<Integer>> spatialGrid = new HashMap<>();
        double gridSize = clusterRadius;
        
        // 将点分配到网格中
        for (int i = 0; i < uncoveredPoints.size(); i++) {
            LatLngPoint point = uncoveredPoints.get(i);
            String gridKey = getGridKey(point, gridSize);
            spatialGrid.computeIfAbsent(gridKey, k -> new ArrayList<>()).add(i);
        }
        
        boolean[] visited = new boolean[uncoveredPoints.size()];

        for (int i = 0; i < uncoveredPoints.size(); i++) {
            if (visited[i]) {
                continue;
            }

            List<LatLngPoint> cluster = new ArrayList<>();
            Queue<Integer> queue = new LinkedList<>();
            queue.offer(i);
            visited[i] = true;

            while (!queue.isEmpty()) {
                int current = queue.poll();
                cluster.add(uncoveredPoints.get(current));
                
                LatLngPoint currentPoint = uncoveredPoints.get(current);
                
                // 只检查邻近网格中的点
                Set<Integer> nearbyPoints = getNearbyPointsFromGrid(currentPoint, spatialGrid, gridSize);
                
                for (int j : nearbyPoints) {
                    if (!visited[j]) {
                        double distance = calculateDistance(currentPoint, uncoveredPoints.get(j));
                        if (distance <= clusterRadius) {
                            visited[j] = true;
                            queue.offer(j);
                        }
                    }
                }
            }

            if (!cluster.isEmpty()) {
                clusters.add(cluster);
            }
        }

        return clusters;
    }
    
    /**
     * 获取点的网格键
     */
    private static String getGridKey(LatLngPoint point, double gridSize) {
        double latGrid = Math.floor(point.latitude * 111000.0 / gridSize);
        double lngGrid = Math.floor(point.longitude * 111000.0 / gridSize);
        return latGrid + "," + lngGrid;
    }
    
    /**
     * 从空间网格中获取邻近点
     */
    private static Set<Integer> getNearbyPointsFromGrid(LatLngPoint point, Map<String, List<Integer>> spatialGrid, double gridSize) {
        Set<Integer> nearbyPoints = new HashSet<>();
        
        double latGrid = Math.floor(point.latitude * 111000.0 / gridSize);
        double lngGrid = Math.floor(point.longitude * 111000.0 / gridSize);
        
        // 检查当前网格和8个邻近网格
        for (int dLat = -1; dLat <= 1; dLat++) {
            for (int dLng = -1; dLng <= 1; dLng++) {
                String gridKey = (latGrid + dLat) + "," + (lngGrid + dLng);
                List<Integer> gridPoints = spatialGrid.get(gridKey);
                if (gridPoints != null) {
                    nearbyPoints.addAll(gridPoints);
                }
            }
        }
        
        return nearbyPoints;
    }



    /**
     * 计算聚类的跨度
     */
    private static double getClusterSpan(List<LatLngPoint> cluster) {
        if (cluster.size() < 2) {
            return 0;
        }

        LatLngPoint[] bounds = getBounds(cluster);
        double latSpan = calculateDistance(
                new LatLngPoint(bounds[0].latitude, bounds[0].longitude),
                new LatLngPoint(bounds[1].latitude, bounds[0].longitude)
        );
        double lngSpan = calculateDistance(
                new LatLngPoint(bounds[0].latitude, bounds[0].longitude),
                new LatLngPoint(bounds[0].latitude, bounds[1].longitude)
        );

        return Math.max(latSpan, lngSpan);
    }



    /**
     * 全局航点间距验证方法
     */
    private static List<LatLngPoint> validateWaypointSpacing(List<LatLngPoint> waypoints,
                                                             List<LatLngPoint> existingWaypoints,
                                                             double pointSpacing) {
        List<LatLngPoint> validatedWaypoints = new ArrayList<>();
        List<LatLngPoint> allExistingPoints = new ArrayList<>(existingWaypoints);

        // 调整间距要求：保持合理的最小间距，但不过于严格
        double minDistance = pointSpacing * 0.75; // 最小间距：75%的标准间距（放宽要求）
        int rejectedTooClose = 0;
        int acceptedCount = 0;

        for (LatLngPoint waypoint : waypoints) {
            boolean validSpacing = true;

            // 只检查过近的航点，不检查过远的航点（避免大片区域被拒绝）
            if (!allExistingPoints.isEmpty()) {
                for (LatLngPoint existing : allExistingPoints) {
                    double distance = calculateDistance(waypoint, existing);

                    if (distance < minDistance) {
                        validSpacing = false;
                        rejectedTooClose++;
                        break;
                    }
                }
            }

            if (validSpacing) {
                validatedWaypoints.add(waypoint);
                allExistingPoints.add(waypoint); // 添加到现有点列表中，用于后续验证
                acceptedCount++;
            }
        }

        // 如果拒绝率过高，给出警告
        double rejectionRate = waypoints.size() > 0 ? (double)rejectedTooClose / waypoints.size() : 0;
        if (rejectionRate > 0.3) {
            System.out.println("警告: 航点拒绝率较高 (" + String.format("%.1f%%", rejectionRate * 100) + ")，可能需要调整间距参数");
        }
        return validatedWaypoints;
    }

    /**
     * 生成与主航线完全一致的智能补充航线
     */
    private static List<LatLngPoint> generateConsistentSupplementaryLines(List<LatLngPoint> clusterPoints,
                                                                          List<LatLngPoint> polygonPoints,
                                                                          AdaptiveParams params,
                                                                          double direction,
                                                                          double flightHeight,
                                                                          List<LatLngPoint> existingWaypoints) {
        List<LatLngPoint> supplementaryWaypoints = new ArrayList<>();

        if (clusterPoints.isEmpty()) {
            return supplementaryWaypoints;
        }

        // 计算聚类的精确边界和中心
        LatLngPoint clusterCenter = calculateCentroid(clusterPoints);
        double clusterSpan = getClusterSpan(clusterPoints);

        // 根据聚类大小选择不同的生成策略
        if (clusterPoints.size() == 1 || clusterSpan < params.lineSpacing * 0.8) {
            // 小聚类或单点：生成单个航点，严格验证间距
            LatLngPoint candidatePoint = null;

            if (isPointInPolygon(clusterCenter, polygonPoints)) {
                candidatePoint = clusterCenter;
            } else {
                // 如果中心不在多边形内，找最近的有效点
                for (LatLngPoint point : clusterPoints) {
                    if (isPointInPolygon(point, polygonPoints)) {
                        candidatePoint = point;
                        break;
                    }
                }
            }

            // 严格验证候选点与现有航点的距离
            if (candidatePoint != null) {
                List<LatLngPoint> candidateList = new ArrayList<>();
                candidateList.add(candidatePoint);
                List<LatLngPoint> validatedCandidates = validateWaypointSpacing(candidateList, existingWaypoints, params.pointSpacing);

                if (!validatedCandidates.isEmpty()) {
                    supplementaryWaypoints.addAll(validatedCandidates);
                }
            }
        } else {
            // 大聚类：使用平行线策略
            double directionRad = Math.toRadians(direction);
            double perpDirRad = directionRad + Math.PI / 2;

            List<List<LatLngPoint>> supplementaryLines = generateAdaptiveParallelLines(polygonPoints, clusterCenter,
                    directionRad, perpDirRad,
                    params.lineSpacing, params.pointSpacing,
                    clusterPoints, params.coverageRadius);

            if (!supplementaryLines.isEmpty()) {
                List<LatLngPoint> connectedWaypoints = connectLinesWithStrictValidation(supplementaryLines, polygonPoints);

                // 收集覆盖未覆盖点的航点
                List<LatLngPoint> candidateWaypoints = new ArrayList<>();
                for (LatLngPoint waypoint : connectedWaypoints) {
                    boolean coversUncovered = false;
                    for (LatLngPoint uncovered : clusterPoints) {
                        if (calculateDistance(waypoint, uncovered) <= params.coverageRadius) {
                            coversUncovered = true;
                            break;
                        }
                    }

                    if (coversUncovered) {
                        candidateWaypoints.add(waypoint);
                    }
                }

                // 应用全局间距验证
                List<LatLngPoint> validatedWaypoints = validateWaypointSpacing(candidateWaypoints, existingWaypoints, params.pointSpacing);
                supplementaryWaypoints.addAll(validatedWaypoints);

            } else {
                // 平行线策略失败，回退到网格策略
                List<LatLngPoint> gridWaypoints = generateGridWaypoints(clusterPoints, polygonPoints, params);
                List<LatLngPoint> validatedGridWaypoints = validateWaypointSpacing(gridWaypoints, existingWaypoints, params.pointSpacing);
                supplementaryWaypoints.addAll(validatedGridWaypoints);

            }
        }

        return supplementaryWaypoints;
    }

    /**
     * 生成自适应平行线（更灵活的补充航线策略）
     */
    private static List<List<LatLngPoint>> generateAdaptiveParallelLines(List<LatLngPoint> polygon,
                                                                         LatLngPoint basePoint,
                                                                         double directionRad,
                                                                         double perpDirRad,
                                                                         double lineSpacing,
                                                                         double pointSpacing,
                                                                         List<LatLngPoint> targetPoints,
                                                                         double coverageRadius) {
        List<List<LatLngPoint>> lines = new ArrayList<>();

        // 计算目标区域的范围
        LatLngPoint[] bounds = getBounds(targetPoints);
        double maxDimension = Math.max(
                calculateDistance(new LatLngPoint(bounds[0].latitude, bounds[0].longitude),
                        new LatLngPoint(bounds[1].latitude, bounds[0].longitude)),
                calculateDistance(new LatLngPoint(bounds[0].latitude, bounds[0].longitude),
                        new LatLngPoint(bounds[0].latitude, bounds[1].longitude))
        );

        // 使用更宽松的航线数量计算
        int maxLines = Math.max(3, (int)(maxDimension / lineSpacing) + 4);
        int validLineCount = 0;


        // 生成自适应平行航线
        for (int i = -maxLines/2; i <= maxLines/2; i++) {
            double offsetDistance = i * lineSpacing;

            // 计算航线起点和终点
            LatLngPoint lineStart = offsetPoint(basePoint, perpDirRad, offsetDistance);
            lineStart = offsetPoint(lineStart, directionRad, -maxDimension * 1.5);
            LatLngPoint lineEnd = offsetPoint(lineStart, directionRad, 3 * maxDimension);

            // 使用更宽松的航点生成逻辑
            List<LatLngPoint> linePoints = generateAdaptiveLinePoints(polygon, lineStart, lineEnd, directionRad, pointSpacing);

            // 检查航线是否覆盖目标点
            boolean coversTarget = false;
            for (LatLngPoint linePoint : linePoints) {
                for (LatLngPoint target : targetPoints) {
                    if (calculateDistance(linePoint, target) <= coverageRadius) {
                        coversTarget = true;
                        break;
                    }
                }
                if (coversTarget) {
                    break;
                }
            }

            if (!linePoints.isEmpty() && coversTarget) {
                lines.add(linePoints);
                validLineCount++;
            }
        }

        return lines;
    }

    /**
     * 生成网格航点（回退策略）- 严格间距控制
     */
    private static List<LatLngPoint> generateGridWaypoints(List<LatLngPoint> clusterPoints,
                                                           List<LatLngPoint> polygonPoints,
                                                           AdaptiveParams params) {
        List<LatLngPoint> gridWaypoints = new ArrayList<>();

        // 计算聚类边界
        LatLngPoint[] bounds = getBounds(clusterPoints);

        // 使用更精确的网格间距计算，确保严格符合pointSpacing
        double latStep = params.pointSpacing / 111000.0; // 转换为度
        double lngStep = params.pointSpacing / (111000.0 * Math.cos(Math.toRadians((bounds[0].latitude + bounds[1].latitude) / 2)));

        List<LatLngPoint> candidatePoints = new ArrayList<>();

        // 在边界内生成候选网格点
        for (double lat = bounds[0].latitude; lat <= bounds[1].latitude; lat += latStep) {
            for (double lng = bounds[0].longitude; lng <= bounds[1].longitude; lng += lngStep) {
                LatLngPoint gridPoint = new LatLngPoint(lat, lng);

                // 检查网格点是否在多边形内且覆盖未覆盖点
                if (isPointInPolygon(gridPoint, polygonPoints)) {
                    boolean coversCluster = false;
                    for (LatLngPoint clusterPoint : clusterPoints) {
                        if (calculateDistance(gridPoint, clusterPoint) <= params.coverageRadius) {
                            coversCluster = true;
                            break;
                        }
                    }

                    if (coversCluster) {
                        candidatePoints.add(gridPoint);
                    }
                }
            }
        }

        // 严格过滤，确保航点间距符合要求
        for (LatLngPoint candidate : candidatePoints) {
            boolean validSpacing = true;

            // 检查与已有航点的距离
            for (LatLngPoint existing : gridWaypoints) {
                double distance = calculateDistance(candidate, existing);
                // 严格要求：距离应该大于等于pointSpacing的85%，避免过近
                if (distance < params.pointSpacing * 0.85) {
                    validSpacing = false;
                    break;
                }
            }

            if (validSpacing) {
                gridWaypoints.add(candidate);
            }
        }

        return gridWaypoints;
    }

    /**
     * 生成自适应航线点（严格间距控制）
     */
    private static List<LatLngPoint> generateAdaptiveLinePoints(List<LatLngPoint> polygon,
                                                                LatLngPoint lineStart,
                                                                LatLngPoint lineEnd,
                                                                double directionRad,
                                                                double pointSpacing) {
        List<LatLngPoint> points = new ArrayList<>();

        // 找到航线与多边形的交点
        List<LatLngPoint> intersections = findLinePolygonIntersections(lineStart, lineEnd, polygon);

        if (intersections.size() < 2) {
            return points; // 航线不与多边形相交
        }

        // 去重并排序交点
        Set<String> uniquePoints = new HashSet<>();
        List<LatLngPoint> filteredIntersections = new ArrayList<>();

        for (LatLngPoint point : intersections) {
            String key = String.format("%.8f,%.8f", point.latitude, point.longitude);
            if (!uniquePoints.contains(key)) {
                uniquePoints.add(key);
                filteredIntersections.add(point);
            }
        }

        filteredIntersections.sort((p1, p2) -> {
            double dist1 = calculateDistance(lineStart, p1);
            double dist2 = calculateDistance(lineStart, p2);
            return Double.compare(dist1, dist2);
        });

        // 在每对交点之间生成航点，严格控制间距
        for (int i = 0; i < filteredIntersections.size() - 1; i += 2) {
            LatLngPoint segStart = filteredIntersections.get(i);
            LatLngPoint segEnd = filteredIntersections.get(i + 1);

            // 使用较小的收缩比例
            double shrinkRatio = 0.01; // 收缩1%
            LatLngPoint adjustedStart = interpolatePoint(segStart, segEnd, shrinkRatio);
            LatLngPoint adjustedEnd = interpolatePoint(segEnd, segStart, shrinkRatio);

            double segDistance = calculateDistance(adjustedStart, adjustedEnd);

            // 严格按照pointSpacing计算航点数量
            if (segDistance < pointSpacing * 0.5) {
                // 线段太短，只生成中点
                LatLngPoint midPoint = interpolatePoint(adjustedStart, adjustedEnd, 0.5);
                if (isPointInPolygon(midPoint, polygon)) {
                    points.add(midPoint);
                }
            } else {
                // 精确计算航点数量，确保间距符合要求
                int numPoints = Math.max(2, (int) Math.round(segDistance / pointSpacing) + 1);

                // 重新计算实际间距，确保均匀分布


                LatLngPoint lastValidPoint = null;

                for (int j = 0; j < numPoints; j++) {
                    double ratio = (double) j / (numPoints - 1);
                    LatLngPoint candidate = interpolatePoint(adjustedStart, adjustedEnd, ratio);

                    if (isPointInPolygon(candidate, polygon)) {
                        // 验证与上一个点的距离
                        if (lastValidPoint == null) {
                            points.add(candidate);
                            lastValidPoint = candidate;
                        } else {
                            double distanceToLast = calculateDistance(lastValidPoint, candidate);
                            // 确保距离在合理范围内（pointSpacing的80%-120%）
                            if (distanceToLast >= pointSpacing * 0.8 && distanceToLast <= pointSpacing * 1.2) {
                                points.add(candidate);
                                lastValidPoint = candidate;
                            } else if (distanceToLast > pointSpacing * 1.2) {
                                // 距离太远，需要插入中间点
                                int insertCount = (int) Math.round(distanceToLast / pointSpacing) - 1;
                                for (int k = 1; k <= insertCount; k++) {
                                    double insertRatio = (double) k / (insertCount + 1);
                                    LatLngPoint insertPoint = interpolatePoint(lastValidPoint, candidate, insertRatio);
                                    if (isPointInPolygon(insertPoint, polygon)) {
                                        points.add(insertPoint);
                                        lastValidPoint = insertPoint;
                                    }
                                }
                                points.add(candidate);
                                lastValidPoint = candidate;
                            }
                        }
                    }
                }
            }
        }

        return points;
    }



    /**
     * 使用严格验证连接航线
     */
    private static List<LatLngPoint> connectLinesWithStrictValidation(List<List<LatLngPoint>> lines, List<LatLngPoint> polygon) {
        List<LatLngPoint> connectedWaypoints = new ArrayList<>();

        if (lines.isEmpty()) {
            return connectedWaypoints;
        }

        // 选择中间的航线作为起始航线
        int startIndex = lines.size() / 2;
        List<LatLngPoint> currentLine = lines.get(startIndex);
        connectedWaypoints.addAll(currentLine);


        Set<Integer> usedLines = new HashSet<>();
        usedLines.add(startIndex);

        // 严格连接其他航线
        while (usedLines.size() < lines.size()) {
            boolean foundConnection = false;
            for (int i = 0; i < lines.size(); i++) {
                if (usedLines.contains(i)) {
                    continue;
                }

                List<LatLngPoint> candidateLine = lines.get(i);
                LatLngPoint lastPoint = connectedWaypoints.get(connectedWaypoints.size() - 1);

                // 尝试正向连接
                if (!candidateLine.isEmpty()) {
                    LatLngPoint firstPoint = candidateLine.get(0);
                    if (isLineStrictlyInPolygon(lastPoint, firstPoint, polygon)) {
                        connectedWaypoints.addAll(candidateLine);
                        usedLines.add(i);
                        foundConnection = true;
                        break;
                    }

                    // 尝试反向连接
                    LatLngPoint lastPointOfLine = candidateLine.get(candidateLine.size() - 1);
                    if (isLineStrictlyInPolygon(lastPoint, lastPointOfLine, polygon)) {
                        List<LatLngPoint> reversedLine = new ArrayList<>(candidateLine);
                        Collections.reverse(reversedLine);
                        connectedWaypoints.addAll(reversedLine);
                        usedLines.add(i);
                        foundConnection = true;
                        break;
                    }
                }
            }

            if (!foundConnection) {
                break;
            }
        }

        return connectedWaypoints;
    }

    /**
     * 计算多边形面积（平方米）
     * 使用鞋带公式（Shoelace formula）
     */
    private static double calculatePolygonArea(List<LatLngPoint> polygonPoints) {
        if (polygonPoints.size() < 3) {
            return 0.0;
        }

        double area = 0.0;
        int n = polygonPoints.size();

        // 鞋带公式计算面积
        for (int i = 0; i < n; i++) {
            int j = (i + 1) % n;
            LatLngPoint p1 = polygonPoints.get(i);
            LatLngPoint p2 = polygonPoints.get(j);

            // 将经纬度转换为米（近似）
            double x1 = p1.longitude * 111000.0 * Math.cos(Math.toRadians(p1.latitude));
            double y1 = p1.latitude * 111000.0;
            double x2 = p2.longitude * 111000.0 * Math.cos(Math.toRadians(p2.latitude));
            double y2 = p2.latitude * 111000.0;

            area += (x1 * y2 - x2 * y1);
        }

        return Math.abs(area) / 2.0;
    }


    /**
     * 优化航点序列：合并同一直线上的冗余航点，正确处理航线转弯
     * 时间复杂度：O(n)
     * 
     * @param waypoints 原始航点序列
     * @param toleranceAngle 角度容差（度），用于判断是否为同一直线，默认2度
     * @param minSegmentLength 最小线段长度（米），短于此长度的线段不进行优化
     * @return 优化后的航点序列
     */
    public static List<LatLngPoint> optimizeWaypoints(List<LatLngPoint> waypoints, double toleranceAngle, double minSegmentLength) {
        if (waypoints == null || waypoints.size() <= 2) {
            return new ArrayList<>(waypoints);
        }
        
        List<LatLngPoint> optimized = new ArrayList<>();
        optimized.add(waypoints.get(0)); // 添加起始点
        
        int segmentStart = 0;
        
        for (int i = 1; i < waypoints.size() - 1; i++) {
            LatLngPoint current = waypoints.get(i);
            LatLngPoint next = waypoints.get(i + 1);
            
            // 检查是否为真正的航线转弯点
            boolean isTurningPoint = false;
            
            if (i >= 1) {
                LatLngPoint prev = waypoints.get(i - 1);
                
                // 计算当前段和下一段的方向
                double bearing1 = calculateBearing(prev, current);
                double bearing2 = calculateBearing(current, next);
                
                // 计算方向变化
                double angleChange = calculateAngleDifference(bearing1, bearing2);
                
                // 只有当角度变化超过较大阈值时才认为是转弯点
                if (angleChange > toleranceAngle * 5) { // 提高阈值到5倍容差
                    isTurningPoint = true;
                }
                
                // 额外检查：距离突然增大的情况（跨航线连接）
                if (i >= 2) {
                    double dist1 = calculateDistance(waypoints.get(i-2), waypoints.get(i-1));
                    double dist2 = calculateDistance(waypoints.get(i-1), current);
                    double dist3 = calculateDistance(current, next);
                    
                    // 如果当前距离明显大于前后距离，说明是跨航线连接
                    if (dist2 > Math.max(dist1, dist3) * 3.0) {
                        isTurningPoint = true;
                    }
                }
            }
            
            // 如果是转弯点，结束当前线段并开始新线段
            if (isTurningPoint) {
                // 只添加转弯点，不添加额外的中间点
                optimized.add(current);
                segmentStart = i;
            } else {
                // 检查当前线段是否仍然是直线
                if (segmentStart < i - 1) {
                    LatLngPoint segStart = waypoints.get(segmentStart);
                    LatLngPoint segEnd = current;
                    
                    // 检查中间点是否都在直线上
                    boolean isStillStraight = true;
                    for (int j = segmentStart + 1; j < i; j++) {
                        double deviation = calculatePointToLineDistance(waypoints.get(j), segStart, segEnd);
                        // 使用适中的距离阈值判断
                        if (deviation > 3.0) { // 3米的偏差阈值
                            isStillStraight = false;
                            break;
                        }
                    }
                    
                    if (!isStillStraight) {
                        // 线段不再是直线，添加前一个点作为线段终点
                        optimized.add(waypoints.get(i - 1));
                        segmentStart = i - 1;
                    }
                }
            }
        }
        
        // 简化最后线段的处理：只添加最后一个点，不添加倒数第二个点
        optimized.add(waypoints.get(waypoints.size() - 1));
        
        return optimized;
    }
    
    /**
     * 使用默认参数的优化方法
     */
    public static List<LatLngPoint> optimizeWaypoints(List<LatLngPoint> waypoints) {
        return optimizeWaypoints(waypoints, 2.0, 10.0); // 2度角度容差，10米最小线段长度
    }
    
    /**
     * 计算两个角度之间的最小差值
     */
    private static double calculateAngleDifference(double angle1, double angle2) {
        double diff = Math.abs(angle2 - angle1);
        if (diff > 180) {
            diff = 360 - diff;
        }
        return diff;
    }
    
    /**
     * 计算点到直线的距离（米）
     */
    private static double calculatePointToLineDistance(LatLngPoint point, LatLngPoint lineStart, LatLngPoint lineEnd) {
        // 使用向量叉积计算点到直线的距离
        double A = point.latitude - lineStart.latitude;
        double B = point.longitude - lineStart.longitude;
        double C = lineEnd.latitude - lineStart.latitude;
        double D = lineEnd.longitude - lineStart.longitude;
        
        double dot = A * C + B * D;
        double lenSq = C * C + D * D;
        
        if (lenSq == 0) {
            return calculateDistance(point, lineStart);
        }
        
        double param = dot / lenSq;
        
        LatLngPoint projection;
        if (param < 0) {
            projection = lineStart;
        } else if (param > 1) {
            projection = lineEnd;
        } else {
            projection = new LatLngPoint(
                lineStart.latitude + param * C,
                lineStart.longitude + param * D
            );
        }
        
        return calculateDistance(point, projection);
    }
    
    /**
     * 计算线段的总长度
     */
    private static double calculateSegmentLength(List<LatLngPoint> waypoints, int startIndex, int endIndex) {
        double totalLength = 0;
        for (int i = startIndex; i < endIndex; i++) {
            totalLength += calculateDistance(waypoints.get(i), waypoints.get(i + 1));
        }
        return totalLength;
    }
    
    /**
     * 计算两点之间的方位角（度）- 改进版本
     */
    private static double calculateBearing(LatLngPoint from, LatLngPoint to) {
        double lat1 = Math.toRadians(from.latitude);
        double lat2 = Math.toRadians(to.latitude);
        double deltaLng = Math.toRadians(to.longitude - from.longitude);
        
        double y = Math.sin(deltaLng) * Math.cos(lat2);
        double x = Math.cos(lat1) * Math.sin(lat2) - Math.sin(lat1) * Math.cos(lat2) * Math.cos(deltaLng);
        
        double bearing = Math.atan2(y, x);
        return (Math.toDegrees(bearing) + 360) % 360;
    }

    public static void main(String[] args) {
        double sensorWidth = 18;
        double sensorHeight = 13.5;
        double height = 120;
        double focalLength = 24;
        int imageWidth = 1200;
        double v = calculateHeightFromGSD(7, sensorWidth, focalLength, imageWidth);
        System.out.println("v = " + v);
        double gsd = calculateGSD(sensorWidth, height, focalLength, imageWidth);
//        double gsd = 7;
        System.out.println("gsd = " + gsd);
        double flightHeight = calculateFlightHeight(gsd, focalLength, imageWidth, sensorWidth);

        double photoWidth = calculatePhotoWidth(sensorWidth,flightHeight,focalLength);
        double photoLength = calculatePhotoLength(sensorHeight, flightHeight, photoWidth);
        System.out.println("photoLength = " + photoLength);
    }
}