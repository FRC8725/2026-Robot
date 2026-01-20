package frc.robot.lib.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;

public class MathHelpers {
    public static final Pose2d POSE2D_ZERO = new Pose2d();

    public static boolean epsilonEqal(double a, double b, double epsilon) {
        return Math.abs(a - b) < epsilon;
    }

    public static Transform3d reversePitch(Transform3d t) {
        Rotation3d rotation = t.getRotation();
        return new Transform3d(
                t.getTranslation(),
                new Rotation3d(rotation.getX(), -rotation.getY(), rotation.getZ()));
    }

    public static Transform2d toTransform2d(Transform3d t) {
        return new Transform2d(
                t.getTranslation().toTranslation2d(), t.getRotation().toRotation2d());
    }
}
