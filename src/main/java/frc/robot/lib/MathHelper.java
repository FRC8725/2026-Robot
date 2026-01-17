package frc.robot.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public class MathHelper {
    public static Rotation2d getAngleFromHub(Pose2d pose) {
        Translation2d hub = Constants.Field.HUB_CENTER;
        return new Rotation2d(pose.getX() - hub.getX(), pose.getY() - hub.getY());
    }
}