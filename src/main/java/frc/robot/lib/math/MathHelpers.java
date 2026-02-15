package frc.robot.lib.math;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Robot;

public class MathHelpers {
    public static final Pose2d POSE2D_ZERO = new Pose2d();

    public static Translation2d mirrorIfRed(Translation2d t) {
        return Robot.isRedAlliance.get() ?
            new Translation2d(Constants.Field.FIELD_X_SIZE - t.getX(), t.getY()) : t;
    }

    public static Pose2d mirrorIfRed(Pose2d t) {
        return Robot.isRedAlliance.get() ?
            new Pose2d(
                    Constants.Field.FIELD_X_SIZE - t.getX(), t.getY(),
                    t.getRotation().rotateBy(Rotation2d.k180deg))
            : t;
    }

    public static Pose2d[] mirrorIfRed(Pose2d[] t) {
        return Arrays.stream(t)
                .map(pose -> mirrorIfRed(pose))
                .toArray(Pose2d[]::new);
    }

    public static Rotation2d mirrorIfRed(Rotation2d r) {
        return Robot.isRedAlliance.get() ? r.plus(Rotation2d.k180deg) : r;
    }

    public static Rotation2d negativeRotation(Rotation2d r) {
        return Robot.isRedAlliance.get() ? Rotation2d.fromRadians(r.getRadians()) : r;
    }

    public static boolean epsilonEqal(double a, double b, double epsilon) {
        return Math.abs(a - b) < epsilon;
    }

    public static Rotation2d getAngleFromHub(Pose2d pose) {
        Translation2d hub = mirrorIfRed(Constants.Field.HUB_CENTER);
        return new Rotation2d(pose.getX() - hub.getX(), pose.getY() - hub.getY());
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

    public static boolean atInterval(Pose2d pose, double target, double d) {
        return pose.getX() > target - d && pose.getX() < target + d; 
    }

    public static boolean inAutoTimer(double time) {
        return Timer.getFPGATimestamp() - Robot.autoStartTime < time;
    }
}
