package org.firstinspires.ftc.teamcode.pedro.subsystems;

import java.util.ArrayList;
import java.util.List;

public class Zones {

    private static class Zone {
        double minX, maxX, minY, maxY, TPS;

        Zone(double minX, double maxX, double minY, double maxY, double TPS) {
            this.minX = Math.min(minX, maxX);
            this.maxX = Math.max(minX, maxX);
            this.minY = Math.min(minY, maxY);
            this.maxY = Math.max(minY, maxY);
            this.TPS = TPS;
        }

        boolean contains(double X, double Y) {
            return X >= minX && X <= maxX && Y >= minY && Y <= maxY;
        }
    }

    private final List<Zone> zones = new ArrayList<>();
    private final double regularTPS;

    public Zones(double regularTPS) {
        this.regularTPS = regularTPS;
    }

    public Zones addZone(double minX, double maxX, double minY, double maxY, double TPS) {
        zones.add(new Zone(minX, maxX, minY, maxY, TPS));
        return this;
    }

    public double getTPS(double x, double y) {
        for (Zone z : zones) {
            if (z.contains(x, y)) return z.TPS;
        }
        return regularTPS;
    }
}
