package frc.robot.lib.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.lib.math.MathHelpers;

public record MegatagPoseEstimate(
        Pose2d fieldToRobot,
        double timestampSeconds,
        double latency,
        double avgTagArea,
        double quality,
        int[] fiducialIds) {
    public MegatagPoseEstimate {
        if (fieldToRobot == null) {
            fieldToRobot = MathHelpers.POSE2D_ZERO;
        }
        if (fiducialIds == null) {
            fiducialIds = new int[0];
        }
    }

    public static MegatagPoseEstimate fromLimelight(LimelightHelpers.PoseEstimate poseEstimate) {
        Pose2d fieldToRobot = poseEstimate.pose;
        if (fieldToRobot == null) {
            fieldToRobot = MathHelpers.POSE2D_ZERO;
        }
        int[] fiducialIds = new int[poseEstimate.rawFiducials.length];
        for (int i = 0; i < poseEstimate.rawFiducials.length; i++) {
            if (poseEstimate.rawFiducials[i] != null) {
                fiducialIds[i] = poseEstimate.rawFiducials[i].id;
            }
        }
        return new MegatagPoseEstimate(
                fieldToRobot,
                poseEstimate.timestampSeconds,
                poseEstimate.latency,
                poseEstimate.avgTagArea,
                fiducialIds.length > 1 ? 1.0 : 1.0 - poseEstimate.rawFiducials[0].ambiguity,
                fiducialIds);
    }
}
