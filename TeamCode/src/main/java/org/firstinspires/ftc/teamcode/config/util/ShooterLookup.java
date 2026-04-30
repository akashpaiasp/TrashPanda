package org.firstinspires.ftc.teamcode.config.util;

import org.firstinspires.ftc.teamcode.config.core.Robot;

public class ShooterLookup {

    // --- Editable table entries ---
    // Add your own distances, hood positions, and RPMs here.
    // Make sure distances are in increasing order.
    private static final ShotPoint[] table = {
            new ShotPoint(37, 0.76, 2150),
            new ShotPoint(48.7, .76, 2200),
            new ShotPoint(58.37, .78, 2350),
            new ShotPoint(67, 0.78, 2600),
            new ShotPoint(77, 0.82, 2900),
            new ShotPoint(87.46, 0.82, 3070),
            new ShotPoint(97.7, 0.82, 3300),
            new ShotPoint(104, 0.82, 3400),
            new ShotPoint(109, 0.82, 3700),
            new ShotPoint(122, 0.81, 3700),
            new ShotPoint(136, 0.8, 3700),
            new ShotPoint(147, 0.8, 3800)



    };

    // Data structure for a single point
    public static class ShotPoint {
        public final double distance;
        public final double hood;
        public final double rpm;

        public ShotPoint(double distance, double hood, double rpm) {
            this.distance = distance;
            this.hood = hood;
            this.rpm = rpm;
        }
    }

    // Public helper that returns interpolated hood & rpm for any distance
    public static ShotPoint getShot(double distance) {
        // If below lowest distance → return first point
        if (distance <= table[0].distance) {
            return table[0];
        }

        // If above highest distance → return last point
        if (distance >= table[table.length - 1].distance) {
            return table[table.length - 1];
        }

        // Find the two table points around the given distance
        for (int i = 0; i < table.length - 1; i++) {
            ShotPoint low = table[i];
            ShotPoint high = table[i + 1];

            if (distance >= low.distance && distance <= high.distance) {
                double t = (distance - low.distance) / (high.distance - low.distance);

                double hoodInterp = low.hood + t * (high.hood - low.hood);
                double rpmInterp  = low.rpm  + t * (high.rpm  - low.rpm);

                if (Robot.airsort)
                    rpmInterp += Robot.airsortOffset;

                return new ShotPoint(distance, hoodInterp, rpmInterp);
            }
        }

        // fallback (should never hit)
        return table[table.length - 1];
    }
}
