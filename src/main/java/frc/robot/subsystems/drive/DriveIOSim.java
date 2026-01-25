package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.lib.simulation.MapleSimDrivetrain;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

public class DriveIOSim extends DriveIOHardware {
    private static final double simLoopPeriod = 0.005; // 5 ms
    private double lastSimTime;
    private Notifier simNotifier = null;
    public MapleSimDrivetrain mapleSimDrivetrain = null;
    private RobotState robotState = null;

    private final Consumer<SwerveDriveState> simTelemetryConsumer =
            swerveDriveState -> {
                if (Constants.useMapleSim && this.mapleSimDrivetrain != null) {
                    swerveDriveState.Pose =
                            this.mapleSimDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose();
                }
                this.telemetryConsumer.accept(swerveDriveState);
                this.robotState.addFieldToRobot(swerveDriveState.Pose);
            };

    public DriveIOSim(
            RobotState robotState,
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        this.robotState = robotState;

        this.registerTelemetry(this.simTelemetryConsumer);
        this.startSimThread();
    }

    @SuppressWarnings("unchecked")
    public void startSimThread() {
        if (Constants.useMapleSim) {
            this.mapleSimDrivetrain =
                    new MapleSimDrivetrain(
                            Seconds.of(simLoopPeriod),
                            Pounds.of(Constants.Drive.ROBOT_WEIGHT_POUNDS),
                            Inches.of(Constants.Drive.BUMPER_LENGTH_INCHES),
                            Inches.of(Constants.Drive.BUMPER_LENGTH_INCHES),
                            1.2,
                            this.getModuleLocations(),
                            this.getPigeon2(),
                            this.getModules(),
                            SimTunerConstants.frontLeft,
                            SimTunerConstants.frontRight,
                            SimTunerConstants.backLeft,
                            SimTunerConstants.backRight);
            this.simNotifier = new Notifier(this.mapleSimDrivetrain::update);
        } else {
            this.lastSimTime = Utils.getCurrentTimeSeconds();
            this.simNotifier =
                    new Notifier(
                            () -> {
                                final double currentTime = Utils.getCurrentTimeSeconds();
                                double deltaTime = currentTime - this.lastSimTime;
                                this.lastSimTime = currentTime;

                                this.updateSimState(deltaTime, RobotController.getBatteryVoltage());
                            });
        }
        this.simNotifier.startPeriodic(simLoopPeriod);
    }

    @Override
    public void resetOdometry(Pose2d pose) {
        if (Constants.useMapleSim && this.mapleSimDrivetrain != null) {
            this.mapleSimDrivetrain.mapleSimDrive.setSimulationWorldPose(pose);
            Timer.delay(0.05);
        }
        super.resetOdometry(pose);
    }

    @Override
    public void readInput(DriveIOInputs inputs) {
        super.readInput(inputs);

        var pose = this.robotState.getLastestFieldToRobot();
        if (pose == null) return;
        Logger.recordOutput("Drive/Vi/SimPose", inputs.Pose);
    }

    public MapleSimDrivetrain getMapleSimDrive() {
        return this.mapleSimDrivetrain;
    }
}
