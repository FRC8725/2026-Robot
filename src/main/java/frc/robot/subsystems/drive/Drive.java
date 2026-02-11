package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.limelight.VisionFieldPoseEstimate;
import frc.robot.lib.math.MathHelpers;
import frc.robot.lib.simulation.MapleSimDrivetrain;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
    private static Drive DRIVE;
    private final DriveIO io;
    private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

    private final SwerveRequest.ApplyRobotSpeeds pathRequest = new SwerveRequest.ApplyRobotSpeeds();
    private final SwerveRequest.ApplyFieldSpeeds choreoAutoRequest = new SwerveRequest.ApplyFieldSpeeds();
    private final SwerveRequest.FieldCentric stopRequest = 
            new SwerveRequest.FieldCentric()
                    .withDriveRequestType(DriveRequestType.Velocity)
                    .withVelocityX(0.0)
                    .withVelocityY(0.0)
                    .withRotationalRate(0.0);

    private final PIDController xController = new PIDController(10.0, 0, 0.0);
    private final PIDController yController = new PIDController(10.0, 0, 0.0);
    private final PIDController headingController = new PIDController(7.0, 0.0, 0.0);

    public Drive(DriveIO io) {
        DRIVE = this;
        this.io = io;
        this.headingController.enableContinuousInput(-Math.PI, Math.PI);
        this.configureAutoBuilder();
    }

    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> this.inputs.Pose,   // Supplier of current robot pose
                this::resetOdometry,         // Consumer for seeding pose against auto
                () -> this.getRobotChassisSpeeds(), // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    this.pathRequest.withSpeeds(ChassisSpeeds.discretize(speeds, Constants.ROBOT_PERIODIC))
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(10.0, 0.0, 0.0),
                    // PID constants for rotation
                    new PIDConstants(7.0, 0.0, 0.0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    public static Drive getInstance() {
        return DRIVE;
    }

    public void resetOdometry(Pose2d pose) {
        this.io.resetOdometry(pose);
    }

    public void resetYaw(Rotation2d rotation) {
        this.io.resetOdometry(
                new Pose2d(this.getPose().getTranslation(), rotation));
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

    public void stopModules() {
        this.setControl(this.stopRequest);
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

    public Pose2d[] getClosestScorePoints() {
        Pose2d leftPoint = MathHelpers.mirrorIfRed(Constants.Field.LEFT_APPROACH_POSE);
        Pose2d rightPoint = MathHelpers.mirrorIfRed(Constants.Field.RIGHT_APPROACH_POSE);

        double left = this.getPose().getTranslation()
                .getDistance(leftPoint.getTranslation());
        double right = this.getPose().getTranslation()
                .getDistance(rightPoint.getTranslation());

        return left < right ? Constants.Field.LEFT_POINT_POSE_ARRAY : Constants. Field.RIGHT_POINT_POSE_ARRAY;
    }

    public boolean withinTolerance(Translation2d t) {
        return this.getPose().getTranslation().getDistance(t) < Constants.Drive.ALIGNMENT_TOLERANCE;
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
