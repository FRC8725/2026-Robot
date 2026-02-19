package frc.robot.lib.util;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.lib.math.MathHelpers;
import frc.robot.subsystems.drive.Drive;

public class ShootCaculator {
    private static ShootCaculator SHOOT_CACULATOR;
    private final double MIN_DISTANCE = 1.581;
    private final double MAX_DISTANCE = 4.29;

    private final LinearFilter hoodAngleFilter =
            LinearFilter.movingAverage((int) (0.1 / Constants.ROBOT_PERIODIC));
    private final LinearFilter driveAngleFilter =
            LinearFilter.movingAverage((int) (0.8 / Constants.ROBOT_PERIODIC));
    private static final double phaseDelay = 0.03;
    private static final Transform3d robotToShooter =
            new Transform3d(0.147067, 0.0, 0.4832096, new Rotation3d(0.0, 0.0, Math.PI));
    private LaunchingParameters latestParameters = null;

    private final InterpolatingDoubleTreeMap shooterAngleMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap shooterVelocityMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap shooterTimerMap = new InterpolatingDoubleTreeMap();

    private double lastHoodAngle;
    private Rotation2d lastDriveAngle;

    public ShootCaculator() {
        SHOOT_CACULATOR = this;
        for (Pair<Double, Double> pair : Constants.SHOOTER_ANGLE_MAP) {
            this.shooterAngleMap.put(pair.getFirst(), pair.getSecond());
        }
        for (Pair<Double, Double> pair : Constants.SHOOTER_VELOCITY_MAP) {
            this.shooterVelocityMap.put(pair.getFirst(), pair.getSecond());
        }
        for (Pair<Double, Double> pair : Constants.SHOOTER_TIMER_MAP) {
            this.shooterTimerMap.put(pair.getFirst(), pair.getSecond());
        }
    }

    public static ShootCaculator getInstance() {
        return SHOOT_CACULATOR;
    }

    /**
     * 
     * @param distance Units: Degrees
     * @return
     */
    public double getHoodAngle(double distance) {
        return this.getParameters().hoodAngle;
        // if (distance < MIN_DISTANCE) {
        //     return 0.0;
        // } else {
        //     return this.shooterAngleMap.get(distance);
        // }
    }

    public double getFlywheelVelocity(double distance) {
        return this.getParameters().flywheelSpeed;
    }

    public record LaunchingParameters(
      boolean isValid,
      Rotation2d driveAngle,
      double driveVelocity,
      double hoodAngle,
      double hoodVelocity,
      double flywheelSpeed,
      double distance,
      double distanceNoLookahead,
      double timeOfFlight) {}

    public LaunchingParameters getParameters() {
        // Calculate estimated pose while accounting for phase delay
        Pose2d estimatedPose = Drive.getInstance().getPose();
        ChassisSpeeds robotRelativeVelocity = Drive.getInstance().getRobotChassisSpeeds();
        estimatedPose =
                estimatedPose.exp(
                        new Twist2d(
                                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

        // Calculate target
        Translation2d target = MathHelpers.mirrorIfRed(Constants.Field.HUB_CENTER);

        // Calculate distance from launcher to target
        Pose2d launcherPosition = estimatedPose.transformBy(MathHelpers.toTransform2d(robotToShooter));
        double launcherToTargetDistance = target.getDistance(launcherPosition.getTranslation());

        // Calculate field relative launcher velocity
        // This isn't actually the launcherVelocity given it won't account for angular velocity of robot
        ChassisSpeeds robotFieldSpeeds = Drive.getInstance().getFieldChassisSpeeds();
        double launcherVelocityX = robotFieldSpeeds.vxMetersPerSecond;
        double launcherVelocityY = robotFieldSpeeds.vyMetersPerSecond;

        // Account for imparted velocity by robot (launcher) to offset
        double timeOfFlight = this.shooterTimerMap.get(launcherToTargetDistance);
        Pose2d lookaheadPose = launcherPosition;
        double lookaheadLauncherToTargetDistance = launcherToTargetDistance;

        for (int i = 0; i < 20; i++) {
            timeOfFlight = this.shooterTimerMap.get(lookaheadLauncherToTargetDistance);
            double offsetX = launcherVelocityX * timeOfFlight;
            double offsetY = launcherVelocityY * timeOfFlight;
            lookaheadPose =
                    new Pose2d(
                            launcherPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
                            launcherPosition.getRotation());
            lookaheadLauncherToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
        }

        // Account for launcher being off center
        Pose2d lookaheadRobotPose =
                lookaheadPose.transformBy(MathHelpers.toTransform2d(robotToShooter).inverse());
        Rotation2d driveAngle = getDriveAngleWithLauncherOffset(lookaheadRobotPose, target);

        // Calculate remaining parameters
        double hoodAngle = this.shooterAngleMap.get(lookaheadLauncherToTargetDistance);
        if (this.lastDriveAngle == null)
            this.lastDriveAngle = driveAngle;
        if (Double.isNaN(lastHoodAngle))
            this.lastHoodAngle = hoodAngle;

        double hoodVelocity = this.hoodAngleFilter.calculate(
                (hoodAngle - lastHoodAngle) / Constants.ROBOT_PERIODIC);
        lastHoodAngle = hoodAngle;
        double driveVelocity = driveAngleFilter.calculate(
            driveAngle.minus(lastDriveAngle).getRadians() / Constants.ROBOT_PERIODIC);

        this.lastDriveAngle = driveAngle;

        // Constructor parameters
        latestParameters =
            new LaunchingParameters(
                lookaheadLauncherToTargetDistance >= MIN_DISTANCE
                    && lookaheadLauncherToTargetDistance <= MAX_DISTANCE,
                driveAngle,
                driveVelocity,
                hoodAngle,
                hoodVelocity,
                this.shooterVelocityMap.get(lookaheadLauncherToTargetDistance),
                lookaheadLauncherToTargetDistance,
                launcherToTargetDistance,
                timeOfFlight);

        // Log calculated values
        Logger.recordOutput("Shooter/LookaheadPose", lookaheadPose);
        Logger.recordOutput(
            "Shooter/LauncherToTargetDistance", lookaheadLauncherToTargetDistance);

        return latestParameters;
    }
  
    public static Rotation2d getDriveAngleWithLauncherOffset(
            Pose2d robotPose, Translation2d target) {
        Rotation2d fieldToHubAngle = target.minus(robotPose.getTranslation()).getAngle();
        Rotation2d hubAngle = new Rotation2d(Math.asin(
                MathUtil.clamp(
                        robotToShooter.getTranslation().getY()
                            / target.getDistance(robotPose.getTranslation()),
                        -1.0,
                        1.0)));
        Rotation2d driveAngle =
            fieldToHubAngle.plus(hubAngle).plus(robotToShooter.getRotation().toRotation2d());
        return driveAngle;
  }
}
