package org.firstinspires.ftc.teamcode.config.util;

public class ShooterLookup {

    // --- Editable table entries ---
    // Add your own distances, hood positions, and RPMs here.
    // Make sure distances are in increasing order.
    private static final ShotPoint[] table = {
            new ShotPoint(45.2, .75, 2300),
            new ShotPoint(75, .77, 2500),
            new ShotPoint(62, 0.4, 4000),
            new ShotPoint(15, 0.45, 3500)
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

                return new ShotPoint(distance, hoodInterp, rpmInterp);
            }
        }

        // fallback (should never hit)
        return table[table.length - 1];
    }
}
