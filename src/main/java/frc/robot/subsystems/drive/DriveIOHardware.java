package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.lib.limelight.VisionFieldPoseEstimate;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveIOHardware extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder>
        implements DriveIO {
    private final AtomicReference<SwerveDriveState> telemetryCache = new AtomicReference<>();
    public final Consumer<SwerveDriveState> telemetryConsumer =
            swerveDriveState -> this.telemetryCache.set(swerveDriveState.clone());

    private final StatusSignal<AngularVelocity> angularPitchVelocity;
    private final StatusSignal<AngularVelocity> angularRollVelocity;
    private final StatusSignal<AngularVelocity> angularYawVelocity;
    private final StatusSignal<Angle> roll;
    private final StatusSignal<Angle> pitch;
    private final StatusSignal<LinearAcceleration> accelerationX;
    private final StatusSignal<LinearAcceleration> accelerationY;

    public DriveIOHardware(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, 250.0, modules);

        this.angularPitchVelocity = this.getPigeon2().getAngularVelocityYWorld();
        this.angularRollVelocity = this.getPigeon2().getAngularVelocityXWorld();
        this.angularYawVelocity = this.getPigeon2().getAngularVelocityZWorld();
        this.roll = this.getPigeon2().getRoll();
        this.pitch = this.getPigeon2().getPitch();
        this.accelerationX = this.getPigeon2().getAccelerationX();
        this.accelerationY = this.getPigeon2().getAccelerationY();

        BaseStatusSignal.setUpdateFrequencyForAll(250, this.angularYawVelocity);
        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                this.angularPitchVelocity,
                this.angularRollVelocity,
                this.roll,
                this.pitch,
                this.accelerationX,
                this.accelerationY);

        this.getOdometryThread().setThreadPriority(99);
        this.registerTelemetry(this.telemetryConsumer);
    }

    @Override
    public void readInput(DriveIOInputs inputs) {
        if (this.telemetryCache.get() == null) return;
        inputs.fromSwerveDriveState(this.telemetryCache.get());
        Rotation2d gyroRotation = inputs.Pose.getRotation();
        inputs.gyroAngle = gyroRotation.getDegrees();

        var measuredRobotRelativeChassisSpeeds =
                this.getKinematics().toChassisSpeeds(inputs.ModuleStates);
        var measuredFieldRelativeChassisSpeeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        measuredRobotRelativeChassisSpeeds, gyroRotation);
        var desiredRobotRelativeChassisSpeeds =
                this.getKinematics().toChassisSpeeds(inputs.ModuleTargets);
        var desiredFieldRelativeChassisSpeeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        desiredRobotRelativeChassisSpeeds, gyroRotation);

        BaseStatusSignal.refreshAll(
                this.angularPitchVelocity,
                this.angularRollVelocity,
                this.angularYawVelocity,
                this.roll,
                this.pitch,
                this.accelerationX,
                this.accelerationY);

        Logger.recordOutput(
                "RobotState/PitchAngularVelocity", this.angularPitchVelocity.getValueAsDouble());
        Logger.recordOutput(
                "RobotState/RollAngularVelocity", this.angularRollVelocity.getValueAsDouble());
        Logger.recordOutput(
                "RobotState/YawAngularVelocity", this.angularYawVelocity.getValueAsDouble());
        Logger.recordOutput(
                "RobotState/RollRads", Units.degreesToRadians(this.roll.getValueAsDouble()));
        Logger.recordOutput(
                "RobotState/PitchRads", Units.degreesToRadians(this.pitch.getValueAsDouble()));
        Logger.recordOutput("RobotState/AccelX", this.accelerationX.getValueAsDouble());
        Logger.recordOutput("RobotState/AccelY", this.accelerationY.getValueAsDouble());
        Logger.recordOutput(
                "RobotState/MeasuredRobotRelativeChassisSpeeds",
                measuredRobotRelativeChassisSpeeds);
        Logger.recordOutput(
                "RobotState/MeasuredFieldRelativeChassisSpeeds",
                measuredFieldRelativeChassisSpeeds);
        Logger.recordOutput(
                "RobotState/DesiredRobotRelativeChassisSpeeds", desiredRobotRelativeChassisSpeeds);
        Logger.recordOutput(
                "RobotState/DesiredFieldRelativeChassisSpeeds", desiredFieldRelativeChassisSpeeds);
    }

    @Override
    public void logModules(SwerveDriveState driveState) {
        String[] moduleName = {"Drive/FL", "Drive/FR", "Drive/BL", "Drive/BR"};
        if (driveState.ModuleStates == null) return;
        for (int i = 0; i < this.getModules().length; i++) {
            Logger.recordOutput(
                    moduleName[i] + " Absolute Encoder Angle",
                    this.getModule(i).getEncoder().getAbsolutePosition().getValueAsDouble()
                            * 360.0);
            Logger.recordOutput(moduleName[i] + " Steer Angle", driveState.ModuleStates[i].angle);
            Logger.recordOutput(
                    moduleName[i] + " Target Steer Angle", driveState.ModuleTargets[i].angle);
            Logger.recordOutput(
                    moduleName[i] + " Drive Velocity",
                    driveState.ModuleStates[i].speedMetersPerSecond);
            Logger.recordOutput(
                    moduleName[i] + " Target Drive Velocity",
                    driveState.ModuleTargets[i].speedMetersPerSecond);
        }
    }

    @Override
    public void resetOdometry(Pose2d pose) {
        super.resetPose(pose);
    }

    @Override
    public Command applyRequest(
            Supplier<SwerveRequest> requestSupplier, Subsystem subsystemRequired) {
        return Commands.run(() -> this.setControl(requestSupplier.get()), subsystemRequired);
    }

    @Override
    public void setControl(SwerveRequest request) {
        super.setControl(request);
    }

    @Override
    public void addVisionMeasurement(VisionFieldPoseEstimate visionFieldPoseEstimate) {
        if (visionFieldPoseEstimate.getVisionMeasurementStdDevs() == null) {
            this.addVisionMeasurement(
                    visionFieldPoseEstimate.getVisionRobotPose(),
                    Utils.fpgaToCurrentTime(visionFieldPoseEstimate.getTimestampSeconds()));
        } else {
            this.addVisionMeasurement(
                	visionFieldPoseEstimate.getVisionRobotPose(),
                	Utils.fpgaToCurrentTime(visionFieldPoseEstimate.getTimestampSeconds()),
                    visionFieldPoseEstimate.getVisionMeasurementStdDevs());
        }
    }

    @Override
    public void setStateStdDevs(double xStd, double yStd, double rotStd) {
        Matrix<N3, N1> stateStdDevs = VecBuilder.fill(xStd, yStd, rotStd);
        this.setStateStdDevs(stateStdDevs);
    }
}
