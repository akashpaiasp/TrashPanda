package org.firstinspires.ftc.teamcode.config.pedro;


import com.pedropathing.geometry.CustomCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathConstraints;

import java.util.Arrays;

public class HermiteCurve extends CustomCurve {

    private Pose p0;
    private Pose p1;

    // tangent vectors (in field units, same units as Pose)
    private double m0x, m0y;
    private double m1x, m1y;

    public HermiteCurve(Pose startPoint, Pose endPoint,
                        double m0x, double m0y,
                        double m1x, double m1y) {
        super(startPoint, endPoint);
        this.m0x = m0x;
        this.m0y = m0y;
        this.m1x = m1x;
        this.m1y = m1y;
    }

    public HermiteCurve(Pose startPoint, Pose endPoint,
                        double m0x, double m0y,
                        double m1x, double m1y,
                        PathConstraints constraints) {
        super(Arrays.asList(startPoint, endPoint), constraints);
        this.m0x = m0x;
        this.m0y = m0y;
        this.m1x = m1x;
        this.m1y = m1y;
    }

    @Override
    public String pathType() {
        return "Cubic Hermite Curve";
    }

    @Override
    public HermiteCurve getReversed() {
        HermiteCurve curve = new HermiteCurve(
                getControlPoints().get(1),
                getControlPoints().get(0),
                -m1x, -m1y,
                -m0x, -m0y,
                this.getPathConstraints()
        );
        curve.initialize();
        return curve;
    }

    @Override
    public void initialize() {
        p0 = getControlPoints().get(0);
        p1 = getControlPoints().get(1);
    }

    private void ensureInit() {
        if (p0 == null || p1 == null) {
            initialize();
        }
    }

    @Override
    public Pose getPose(double t) {
        ensureInit();

        double t2 = t * t;
        double t3 = t2 * t;

        double h00 =  2 * t3 - 3 * t2 + 1;
        double h10 =      t3 - 2 * t2 + t;
        double h01 = -2 * t3 + 3 * t2;
        double h11 =      t3 -     t2;

        double x = h00 * p0.getX() + h10 * m0x + h01 * p1.getX() + h11 * m1x;
        double y = h00 * p0.getY() + h10 * m0y + h01 * p1.getY() + h11 * m1y;

        return new Pose(x, y);
    }

    @Override
    public Vector getDerivative(double t) {
        ensureInit();

        double t2 = t * t;

        double dh00 =  6 * t2 - 6 * t;
        double dh10 =  3 * t2 - 4 * t + 1;
        double dh01 = -6 * t2 + 6 * t;
        double dh11 =  3 * t2 - 2 * t;

        double dx = dh00 * p0.getX() + dh10 * m0x + dh01 * p1.getX() + dh11 * m1x;
        double dy = dh00 * p0.getY() + dh10 * m0y + dh01 * p1.getY() + dh11 * m1y;

        return new Vector(dx, dy);
    }

    @Override
    public Vector getSecondDerivative(double t) {
        ensureInit();

        double d2h00 = 12 * t - 6;
        double d2h10 =  6 * t - 4;
        double d2h01 = -12 * t + 6;
        double d2h11 =  6 * t - 2;

        double ddx = d2h00 * p0.getX() + d2h10 * m0x + d2h01 * p1.getX() + d2h11 * m1x;
        double ddy = d2h00 * p0.getY() + d2h10 * m0y + d2h01 * p1.getY() + d2h11 * m1y;

        return new Vector(ddx, ddy);
    }
}
