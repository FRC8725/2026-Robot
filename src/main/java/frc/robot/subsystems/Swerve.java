package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Swerve extends SubsystemBase {
    private static Swerve SWERVE;
    // ---------- Object ---------- //
    private final SwerveModule frontLeft = new SwerveModule(
        2, 1, 9,
        false, true,
        0.05322265625);
    private final SwerveModule frontRight = new SwerveModule(
        4, 3, 10,
        true, true,
        0.3681640625);
    private final SwerveModule backLeft = new SwerveModule(
        6, 5, 11,
        false, true,
        -0.245361328125);
    private final SwerveModule backRight = new SwerveModule(
        8, 7, 12,
        true, true,
        -0.3212890625);

    private final Pigeon2 pigeon = new Pigeon2(13);
    public final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
        Constants.Swerve.KINEMATICS,
        new Rotation2d(this.getGyroAngle()),
        this.getModulePositions(),
        new Pose2d(0.0, 0.0, new Rotation2d(this.getGyroAngle())),
        VecBuilder.fill(0.1, 0.1, 0.1), // Odometry
        VecBuilder.fill(0.9, 0.9, 2.0)); // Vision
    private final StructPublisher<Pose2d> pose = NetworkTableInstance.getDefault()
        .getStructTopic("Component/SwervePose", Pose2d.struct).publish();

    public boolean isAligned = false;
    
    // ---------- Path PID ---------- //
    private final PIDController xController = new PIDController(12.0, 0.0, 0.0);
    private final PIDController yController = new PIDController(12.0, 0.0, 0.0);
    private final PIDController headingController = new PIDController(5.0, 0.0, 0.0);

    public Swerve() {
        SWERVE = this;

        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }
        AutoBuilder.configure(
                this::getPose,
                this::resetPose,
                this::getSpeeds,
                (ChassisSpeeds speeds) -> this.driveRobotRelative(speeds, false),
                new PPHolonomicDriveController(
                        new PIDConstants(5.0),
                        new PIDConstants(5.0)),
                config,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                }, this);

        this.headingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public static Swerve getInstance() {
        return SWERVE;
    }

    // ---------- Function ---------- //
    public Pose2d getPose() {
        return this.poseEstimator.getEstimatedPosition();
    }

    public ChassisSpeeds getSpeeds() {
        return Constants.Swerve.KINEMATICS.toChassisSpeeds(this.getModuleStates());
    }

    public boolean withinTolerance(Translation2d t) {
        return this.getPose().getTranslation().getDistance(t) < Constants.Swerve.ALIGNMENT_TOLERANCE;
    }

    public double getGyroAngle() {
        return MathUtil.angleModulus(Units.degreesToRadians(-this.pigeon.getYaw().getValueAsDouble()));
    }

    public double score(Pose2d pose) {
        double translation = pose.getTranslation().getDistance(this.getPose().getTranslation());
        double rotation = Math.abs(pose.getRotation().minus(this.getPose().getRotation()).getRadians());

        return Constants.Swerve.ALIGN_TRANSLATION_WEIGHT * translation +
            Constants.Swerve.ALIGN_ANGLE_WEIGHT * rotation;
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[]{
            this.frontLeft.getPosition(),
            this.frontRight.getPosition(),
            this.backLeft.getPosition(),
            this.backRight.getPosition()
        };
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            this.frontLeft.getState(),
            this.frontRight.getState(),
            this.backLeft.getState(),
            this.backRight.getState()
        };
    }

    // ---------- Method ---------- //
    public void driveRobotRelative(ChassisSpeeds speeds, boolean robotOriented) {
        ChassisSpeeds relativeSpeeds;
        if (robotOriented) relativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, this.getPose().getRotation());
        else relativeSpeeds = speeds;
        ChassisSpeeds discretizeSpeeds = ChassisSpeeds.discretize(relativeSpeeds, 0.02);
        SwerveModuleState[] states = Constants.Swerve.KINEMATICS.toSwerveModuleStates(discretizeSpeeds);
        this.setDesiredState(states);
        this.poseEstimator.update(new Rotation2d(this.getGyroAngle()), this.getModulePositions());
    }

    public void followSample(SwerveSample sample) {
        Pose2d pose = this.getPose();

        ChassisSpeeds speeds = new ChassisSpeeds(
            sample.vx + this.xController.calculate(pose.getX(), sample.x),
            sample.vy + this.yController.calculate(pose.getY(), sample.y),
            sample.omega - this.headingController.calculate(pose.getRotation().getRadians(), sample.heading)
        );

        // speeds.vxMetersPerSecond *= (Robot.isSimulation() ? -1.0 : 1.0);
        // speeds.vyMetersPerSecond *= (Robot.isSimulation() ? -1.0 : 1.0);
        // speeds.omegaRadiansPerSecond *= (Robot.isSimulation() ? -1.0 : 1.0);

        this.driveRobotRelative(speeds, true);
    }

    public void followPose(Pose2d goalPose) {
        Pose2d currentPose = this.getPose();
        ChassisSpeeds speeds = new ChassisSpeeds(
            this.xController.calculate(currentPose.getX(), goalPose.getX()),
            this.yController.calculate(currentPose.getY(), goalPose.getY()),
            this.headingController.calculate(currentPose.getRotation().getRadians(), goalPose.getRotation().getRadians()));
       
        // speeds.vxMetersPerSecond *= (Robot.isSimulation() ? -1.0 : 1.0);
        // speeds.vyMetersPerSecond *= (Robot.isSimulation() ? -1.0 : 1.0);
        speeds.omegaRadiansPerSecond *= (Robot.isSimulation() ? -1.0 : 1.0);

        this.driveRobotRelative(speeds, true);
    }

    public void setDesiredState(SwerveModuleState[] states) {
        this.frontLeft.setDesiredState(states[0]);
        this.frontRight.setDesiredState(states[1]);
        this.backLeft.setDesiredState(states[2]);
        this.backRight.setDesiredState(states[3]);
    }

    public void resetPose(Pose2d pose) {
        this.poseEstimator.resetPose(pose);
    }

    public void resetYaw(double angle) {
        this.pigeon.setYaw(Units.radiansToDegrees(MathUtil.angleModulus(angle)));
    }

    public void stopModules() {
        this.driveRobotRelative(new ChassisSpeeds(), true);
    }

    @Override
    public void periodic() {
        // Add vision measurement...
        this.poseEstimator.update(new Rotation2d(this.getGyroAngle()), this.getModulePositions());
        this.pose.accept(this.getPose());
    }
}
