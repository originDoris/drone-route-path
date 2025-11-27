package com.origindoris.drone;

import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.NoArgsConstructor;
import lombok.EqualsAndHashCode;

import java.io.Serializable;

/**
 * LatLngPoint
 *
 * @author xhz
 * @date 2025/7/10 13:11
 */
@Data
@EqualsAndHashCode(of = {"latitude", "longitude"})
@AllArgsConstructor
@NoArgsConstructor
public class LatLngPoint implements Serializable {
    public double latitude;  // 纬度
    public double longitude; // 经度
    private Long id;

    public LatLngPoint(double latitude, double longitude) {
        this.latitude = latitude;
        this.longitude = longitude;
    }
}
