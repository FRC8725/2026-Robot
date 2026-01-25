package frc.robot;

import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.limelight.VisionFieldPoseEstimate;
import frc.robot.lib.math.MathHelpers;
import frc.robot.lib.util.ConcurrentTimeInterpolatableBuffer;

public class RobotState extends SubsystemBase {
    private double lastUsedMegatagTimestamp = 0.0;
    private Pose2d lastUsedMegatagPose = Pose2d.kZero;
    private final ConcurrentTimeInterpolatableBuffer<Pose2d> fieldToRobot =
            ConcurrentTimeInterpolatableBuffer.createBuffer(1.0);
    private final ConcurrentTimeInterpolatableBuffer<Double> driveYawAngularVelocity =
            ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(1.0);
    private final AtomicReference<Optional<Integer>> exclusiveTag = 
            new AtomicReference<>(Optional.empty());

    private final TimeInterpolatableBuffer<Pose2d> fieldToRobotSimulatedTruth = 
            TimeInterpolatableBuffer.createBuffer(1.0);

    private final Consumer<VisionFieldPoseEstimate> visionEstimatorConsumer;

    public RobotState(Consumer<VisionFieldPoseEstimate> visionEstimatorConsumer) {
        this.visionEstimatorConsumer = visionEstimatorConsumer;
        this.fieldToRobot.addSample(0.0, MathHelpers.POSE2D_ZERO);
    }

    public void addDriveMotionMeasurements(
            double timestamp, double angularYawRadsPerS) {
        this.driveYawAngularVelocity.addSample(timestamp, angularYawRadsPerS);
    }

    public Optional<Pose2d> getFieldToRobot(double timestamp) {
        return this.fieldToRobot.getSample(timestamp);
    }

    public synchronized void addFieldToRobot(Pose2d pose) {
        this.fieldToRobotSimulatedTruth.addSample(Timer.getFPGATimestamp(), pose);
    }

    public synchronized Pose2d getLastestFieldToRobot() {
        var entry = this.fieldToRobotSimulatedTruth.getInternalBuffer().lastEntry();
        if (entry == null) return null;

        return entry.getValue();
    }

    private Optional<Double> getMaxAbsValueInRange(
            ConcurrentTimeInterpolatableBuffer<Double> buffer, double minTime, double maxTime) {
        var submap = buffer.getInternalBuffer().subMap(minTime, maxTime).values();
        var max = submap.stream().max(Double::compare);
        var min = submap.stream().min(Double::compare);
        if (max.isEmpty() || min.isEmpty()) return Optional.empty();
        if (Math.abs(max.get()) >= Math.abs(min.get())) return max;
        else return min;
    }

    public Optional<Double> getMaxAbsDriveYawAngularVelocityInRange(
            double minTime, double maxTime) {
        // Gyro yaw rate not set in sim.
        return getMaxAbsValueInRange(this.driveYawAngularVelocity, minTime, maxTime);
    }

    public void updateMegatagPoseEstimate(VisionFieldPoseEstimate poseEstimate) {
        this.lastUsedMegatagTimestamp = poseEstimate.getTimestampSeconds();
        this.lastUsedMegatagPose = poseEstimate.getVisionRobotPose();
        this.visionEstimatorConsumer.accept(poseEstimate);
    }

    public double lastUsedMegatagTimestamp() {
        return this.lastUsedMegatagTimestamp;
    }

    public Pose2d lastUsedMegatagPose() {
        return this.lastUsedMegatagPose;
    }

    public void setExclusiveTag(Optional<Integer> tagId) {
        this.exclusiveTag.set(tagId);
    }
    
    public void clearExclusiveTag() {
        this.exclusiveTag.set(Optional.empty());
    }

    public Optional<Integer> getExclusiveTag() {
        return this.exclusiveTag.get();
    }
}
