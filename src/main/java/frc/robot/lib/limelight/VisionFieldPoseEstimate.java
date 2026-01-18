package frc.robot.lib.limelight;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionFieldPoseEstimate {
    private final Pose2d visionRobotPose;
    private final double timestampSeconds;
    private final Matrix<N3, N1> visionMeasurementStdDevs;
    private final int numTags;

    public VisionFieldPoseEstimate(
            Pose2d visionRobotPose,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs,
            int numTags) {
        this.visionRobotPose = visionRobotPose;
        this.timestampSeconds = timestampSeconds;
        this.visionMeasurementStdDevs = visionMeasurementStdDevs;
        this.numTags = numTags;
    }

    public Pose2d getVisionRobotPose() {
        return this.visionRobotPose;
    }

    public double getTimestampSeconds() {
        return this.timestampSeconds;
    }

    public Matrix<N3, N1> getVisionMeasurementStdDevs() {
        return this.visionMeasurementStdDevs;
    }

    public int getNumTags() {
        return this.numTags;
    }
}
