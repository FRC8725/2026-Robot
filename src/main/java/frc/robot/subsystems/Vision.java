package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.FuelEstimator;
import frc.robot.lib.LimelightHelpers;
import frc.robot.lib.LimelightHelpers.LimelightResults;
import frc.robot.lib.LimelightHelpers.LimelightTarget_Detector;

public class Vision extends SubsystemBase {
    
    public List<Translation2d> getFuelsEstimator() {
        List<Translation2d> list = new ArrayList<>();
        LimelightResults results = LimelightHelpers.getLatestResults("obejct");

        if (results.targets_Detector.length > 0) {
            for (LimelightTarget_Detector detector : results.targets_Detector) {
                list.add(FuelEstimator.getFuelTranslation(detector.tx, detector.ty));
            }
        }

        return list;
    }

    public Translation2d getBestFuelTranslation() {
        List<Translation2d> poses = getFuelsEstimator();

        return poses.stream()
                .min(Comparator.comparingDouble(this::caculateWeight))
                .orElse(null);
    }

    public double caculateWeight(Translation2d pose) {
        Pose2d swervePose = Swerve.getInstance().getPose();
        return pose.getAngle().minus(swervePose.getRotation()).getRadians()
                * Constants.Swerve.ALIGN_ANGLE_WEIGHT
                + pose.getDistance(swervePose.getTranslation())
                * Constants.Swerve.ALIGN_TRANSLATION_WEIGHT;
    }

    @AutoLogOutput(key = "Vision/FuelPoses")
    public Pose3d[] getFuelsPose() {
        ArrayList<Pose3d> poses = new ArrayList<>();
        List<Translation2d> fuelTranslations = getFuelsEstimator();
        Pose2d robotPose = Swerve.getInstance().getPose();

        for (Translation2d translation : fuelTranslations) {
            Pose2d fuelField = robotPose.plus(new Transform2d(translation, Rotation2d.kZero));

            poses.add(new Pose3d(fuelField));
        }

        return poses.toArray(Pose3d[]::new);
    }
}
