package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.limelight.VisionFieldPoseEstimate;
import frc.robot.lib.simulation.MapleSimDrivetrain;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
    private static Drive DRIVE;
    private final DriveIO io;
    private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

    private final SwerveRequest.ApplyFieldSpeeds choreoAutoRequest = new SwerveRequest.ApplyFieldSpeeds();

    private final PIDController xController = new PIDController(10.0, 0, 0.0);
    private final PIDController yController = new PIDController(10.0, 0, 0.0);
    private final PIDController headingController = new PIDController(7.0, 0.0, 0.0);

    public Drive(DriveIO io) {
        DRIVE = this;
        this.io = io;
        this.headingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public static Drive getInstance() {
        return DRIVE;
    }

    public void resetOdometry(Pose2d pose) {
        this.io.resetOdometry(pose);
    }

    public void setControl(SwerveRequest request) {
        this.io.setControl(request);
    }

    public void addVisionMeasurement(VisionFieldPoseEstimate poseEstimate) {
        this.io.addVisionMeasurement(poseEstimate);
    }

    public void setStateStdDevs(double xStd, double yStd, double rStd) {
        this.io.setStateStdDevs(xStd, yStd, rStd);
    }

    public void followSample(SwerveSample sample) {
        Pose2d pose = this.inputs.Pose;

        ChassisSpeeds speeds = new ChassisSpeeds(
                sample.vx + this.xController.calculate(pose.getX(), sample.x),
                sample.vy + this.yController.calculate(pose.getY(), sample.y),
                sample.omega + this.headingController.calculate(
                        pose.getRotation().getRadians(), sample.heading));
            
        this.setControl(
                this.choreoAutoRequest
                        .withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(sample.moduleForcesX())
                        .withWheelForceFeedforwardsY(sample.moduleForcesY()));;
    }

    public void followPose(Pose2d goalPose) {
        Pose2d pose = this.inputs.Pose;
        ChassisSpeeds speeds = new ChassisSpeeds(
                this.xController.calculate(pose.getX(), goalPose.getX()),
                this.yController.calculate(pose.getY(), goalPose.getY()),
                this.headingController.calculate(
                        pose.getRotation().getRadians(),
                        goalPose.getRotation().getRadians()));

        this.setControl(this.choreoAutoRequest.withSpeeds(speeds));
    }

    @Override
    public void periodic() {
        double timestamp = Timer.getFPGATimestamp();
        this.io.readInput(this.inputs);
        Logger.processInputs("DriveInputs", this.inputs);
        this.io.logModules(this.inputs);

        Pose2d pose = this.inputs.Pose;
        Pose3d pose3d =
                new Pose3d(
                        pose.getX(),
                        pose.getY(),
                        0.0,
                        new Rotation3d(0.0, 0.0, pose.getRotation().getRadians()));
        Logger.recordOutput("Drive/Pose3d", pose3d);
        Logger.recordOutput("Drive/latencyPeriodicSec", Timer.getFPGATimestamp() - timestamp);
    }

    public ChassisSpeeds getRobotChassisSpeeds() {
        return this.inputs.Speeds;
    }

    public Pose2d getPose() {
        return this.inputs.Pose;
    }

    public MapleSimDrivetrain getMapleSimDrivetrain() {
        if (this.io instanceof DriveIOSim) {
            return ((DriveIOSim) this.io).mapleSimDrivetrain;
        }
        return null;
    }

    public AutoFactory createFactory() {
        return this.createFactory((sample, start) -> {});
    }

    public AutoFactory createFactory(TrajectoryLogger<SwerveSample> logger) {
        return new AutoFactory(
                () -> this.inputs.Pose,
                this::resetOdometry,
                this::followSample,
                true,
                this,
                logger);
    }
}
