package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.lib.limelight.VisionFieldPoseEstimate;
import frc.robot.lib.math.MathHelpers;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;

public interface DriveIO {
    @AutoLog
    public class DriveIOInputs extends SwerveDriveState {
        public double gyroAngle = 0.0;

        DriveIOInputs() {
            this.Pose = MathHelpers.POSE2D_ZERO;
        }

        public void fromSwerveDriveState(SwerveDriveState input) {
            this.Pose = input.Pose;
            this.Speeds = input.Speeds;
            this.ModuleStates = input.ModuleStates;
            this.ModuleTargets = input.ModuleTargets;
            this.OdometryPeriod = input.OdometryPeriod;
            this.SuccessfulDaqs = input.SuccessfulDaqs;
            this.FailedDaqs = input.FailedDaqs;
        }
    }

    void readInput(DriveIOInputs inputs);

    void logModules(SwerveDriveState driveState);

    void resetOdometry(Pose2d pose);

    void setControl(SwerveRequest request);

    Command applyRequest(Supplier<SwerveRequest> requestSupplier, Subsystem subsystemRequired);

    void addVisionMeasurement(VisionFieldPoseEstimate visionFieldPoseEstimate);

    void setStateStdDevs(double xStd, double yStd, double rotStd);
}
