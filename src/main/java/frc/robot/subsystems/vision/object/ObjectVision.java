package frc.robot.subsystems.vision.object;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.limelight.LimelightHelpers;
import frc.robot.lib.limelight.LimelightHelpers.RawDetection;
import frc.robot.lib.math.FuelEstimator;
import frc.robot.subsystems.drive.Drive;

public class ObjectVision extends SubsystemBase {
    
    public List<Translation2d> getFuelsEstimator() {
        List<Translation2d> list = new ArrayList<>();
        RawDetection[] results = LimelightHelpers.getRawDetections("limelight-object");

        if (results.length > 0) {
            for (RawDetection detector : results) {
                list.add(FuelEstimator.getFuelTranslation(Units.degreesToRadians(detector.txnc), Units.degreesToRadians(detector.tync)));
            }
        }

        return list;
    }

    public Translation2d getBestFuelTranslation() {
        List<Translation2d> poses = getFuelsEstimator();

        return poses.stream()
                .min(Comparator.comparingDouble(
                        f -> this.caculateWeight(f, Drive.getInstance().getPose())))
                .orElse(null);
    }

    public double caculateWeight(Translation2d pose, Pose2d referencePose) {
        Pose2d swervePose = Drive.getInstance().getPose();
        Rotation2d angleToFuel = new Rotation2d(
                pose.getX() - referencePose.getX(),
                pose.getY() - referencePose.getY());

        if (Math.abs(angleToFuel.getRadians() - swervePose.getRotation().getRadians()) > Units.degreesToRadians(Constants.Drive.MAX_TURN_ANGLE)) {
            return Double.MAX_VALUE;
        }

        return angleToFuel.getRadians()
                * Constants.Drive.ALIGN_ANGLE_WEIGHT
                + pose.getDistance(swervePose.getTranslation())
                * Constants.Drive.ALIGN_TRANSLATION_WEIGHT;
    }

    @AutoLogOutput(key = "Vision/FuelPoses")
    public Pose3d[] getFuelsPose() {
        ArrayList<Pose3d> poses = new ArrayList<>();
        List<Translation2d> fuelTranslations = getFuelsEstimator();
        Pose2d robotPose = Drive.getInstance().getPose();

        for (Translation2d translation : fuelTranslations) {
            Pose2d fuelField = robotPose.plus(new Transform2d(translation, Rotation2d.kZero));
            Pose3d pose3d = new Pose3d(fuelField.getX(), fuelField.getY(), Units.inchesToMeters(5.91 / 2.0), Rotation3d.kZero);

            poses.add(pose3d);
        }

        return poses.toArray(Pose3d[]::new);
    }
}
