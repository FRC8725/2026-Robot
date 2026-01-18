package frc.robot.lib.math;

import edu.wpi.first.math.geometry.Pose2d;

public class MathHelpers {
    public static final Pose2d POSE2D_ZERO = new Pose2d();

    public static boolean epsilonEqal(double a, double b, double epsilon) {
        return Math.abs(a - b) < epsilon;
    }
}
