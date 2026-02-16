// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.SimTunerConstants;
import frc.robot.subsystems.drive.TunerConstants;

public final class Constants {
    public static final boolean useMapleSim = true;
    public static final AprilTagFieldLayout kAprilTagLayout = 
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
    public static final double ROBOT_PERIODIC = 0.02;

    public static final class RobotMode {
        public static final boolean isReplay = false;

        public enum Mode {
            REAL,
            SIM,
            REPLAY
        }
    }

    public final class Drive {
        public static final double ROBOT_WEIGHT_POUNDS = 150.0;
        public static final double BUMPER_LENGTH_INCHES = 34.417;
        public static final double MAX_SPEED = 3.0; // m/s
        public static final double MAX_ANGULAR_RATE = 4.5; // 1 rotation per second
        public static final double STEER_JOYSTICK_DEADBAND = 0.05;
        public static final CommandSwerveDrivetrain drivetrain =
                RobotBase.isSimulation()
                        ? SimTunerConstants.createTrain()
                        : TunerConstants.createDrivetrain();
        
		// Phycical Limit
        public static final double MAX_MOTOR_RPM = 3300.0;
        public static final double MAX_VELOCITY = (MAX_MOTOR_RPM / 60.0) / TunerConstants.kDriveGearRatio * 2.0 * Units.inchesToMeters(1.897) * Math.PI;
        public static final double MAX_ANGULAR_VELOCITY = (MAX_VELOCITY - 500.0) / (Units.inchesToMeters(12.75) * Math.sqrt(2.0)); // TODO: Research HOW

        // Align weight
        public static final double ALIGN_TRANSLATION_WEIGHT = 5.0;
        public static final double ALIGN_ANGLE_WEIGHT = 2.7;
        public static final double MAX_TURN_ANGLE = 100.0;

        // Tolerance
        public static final double STRATING_TOLERANCE = 0.15;
        public static final double ALIGNMENT_TOLERANCE = 0.02;
        public static final double ALIGNMENT_ANGLE_TOLERANCE = 3.0;

        public static final PhoenixPIDController FACING_HUB_PID =
                new PhoenixPIDController(5.0, 0.0, 0.0);
        public static final PathConstraints CONSTRAINTS = new PathConstraints(
			2.0,
			4.0,
			Math.PI,
			2.0 * Math.PI);
    }
    
    public final class Intake {
        public static final int LIFTER_ID = 25;
        public static final int ROLLER_ID = 26;
        
        public static final double LIFTER_GEAR_RATIO = 24.0 / 5.0;
		public static final double DRUM_CIRCUMFERENCE = 10.0 * 0.00798;
		public static final double DRUM_RADIUS_METERS = DRUM_CIRCUMFERENCE / (2.0 * Math.PI);
		public static final double MECHANISM_GEAR_RATIO = LIFTER_GEAR_RATIO / DRUM_CIRCUMFERENCE;
        public static final double ROLLER_GEAR_RATIO = 1.0;
        public static final double LIFTER_ANGLE_TOLERANCE = 0.01;

		// TODO LATE
        public static final double LIFTER_LIMIT_DISTANCE = 0.0;

        public static final TalonFXConfiguration LIFTER_CONFIG = new TalonFXConfiguration()
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withSupplyCurrentLimitEnable(true)
                                .withSupplyCurrentLimit(40.0)
                                .withStatorCurrentLimitEnable(true)
                                .withStatorCurrentLimit(60.0))
                .withMotionMagic(
                        new MotionMagicConfigs()
                                .withMotionMagicCruiseVelocity(1.5)
                                .withMotionMagicAcceleration(10.0)
                                .withMotionMagicJerk(2000.0))
                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withInverted(InvertedValue.CounterClockwise_Positive)
                                .withNeutralMode(NeutralModeValue.Coast))
                .withFeedback(
                        new FeedbackConfigs()
                                .withSensorToMechanismRatio(MECHANISM_GEAR_RATIO))
                .withSlot0(
                        new Slot0Configs()
                                .withKP(100.0)
                                .withKS(0.0)
                                .withKV(0.0)
                                .withKG(0.0)
                                .withKA(0.0));
    }

    public final class Shooter {
		public static final int HOOD_ID = 20;
		public static final int FLYWHEEL_MAIN_ID = 21;
		public static final int FLYWHEEL_FOLLOW_ID = 22;
		public static final int FEEDER_ID = 23;

        public static final double HOOD_GEAR_RATIO = 294.0;
		public static final double FLYWHEEL_GEAR_RATIO = 1.0;
		
		public static final double FLYWHEEL_TOLERANCE = 3.0;
		public static final double HOOD_TOLERANCE = 0.5;

        public static final TalonFXConfiguration FLYWHEEL_CONFIG = new TalonFXConfiguration()
                .withCurrentLimits(
						new CurrentLimitsConfigs()
								.withSupplyCurrentLimitEnable(true)
								.withSupplyCurrentLimit(60.0)
								.withStatorCurrentLimitEnable(true)
								.withStatorCurrentLimit(80.0))
				.withMotorOutput(
						new MotorOutputConfigs()
								.withInverted(InvertedValue.Clockwise_Positive)
								.withNeutralMode(NeutralModeValue.Coast))
				.withSlot0(
						new Slot0Configs()
								.withKP(0.3));
		public static final TalonFXConfiguration HOOD_CONFIG = new TalonFXConfiguration()
				.withCurrentLimits(
						new CurrentLimitsConfigs()
								.withStatorCurrentLimitEnable(true)
                				.withStatorCurrentLimit(60.0)
                				.withSupplyCurrentLimitEnable(true)
                				.withSupplyCurrentLimit(40.0))
				.withMotorOutput(
						new MotorOutputConfigs()
								.withInverted(InvertedValue.CounterClockwise_Positive)
								.withNeutralMode(NeutralModeValue.Brake))
				.withFeedback(
						new FeedbackConfigs()
								.withSensorToMechanismRatio(HOOD_GEAR_RATIO))
				.withMotionMagic(
						new MotionMagicConfigs()
								.withMotionMagicCruiseVelocity(1.0)
                				.withMotionMagicAcceleration(200.0)
                				.withMotionMagicJerk(2000.0))
				.withSlot0(
						new Slot0Configs()
								.withKS(0.3)
								.withKV(30.0)
								.withKA(0.0)
								.withKG(0.0)
								.withKP(450.0)
								.withKD(0.0));
    }

	public final class Hopper {
		public static final int ROLLER_ID = 24;
		public static final int CENTER_ID = 27;
		public static final double GEAR_RATIO = 1.0;
	}

    public final class Vision {
        public static final double LARGE_VARIANCE = 1e6;

        // Standard deviation constants
        public static final int kMegatag1XStdDevIndex = 0;
        public static final int kMegatag1YStdDevIndex = 1;
        public static final int kMegatag1YawStdDevIndex = 5;

        // Standard deviation array indices for Megatag2
        public static final int kMegatag2XStdDevIndex = 6;
        public static final int kMegatag2YStdDevIndex = 7;
        public static final int kMegatag2YawStdDevIndex = 11;

        // Validation constants
        public static final int kExpectedStdDevArrayLength = 12;

        // Vision processing constants
        public static final double kDefaultAmbiguityThreshold = 0.19;
        public static final double kDefaultYawDiffThreshold = 5.0;
        public static final double kTagAreaThresholdForYawCheck = 2.0;
        public static final double kTagMinAreaForSingleTagMegatag = 1.0;
        public static final double kDefaultZThreshold = 0.2;
        public static final double kDefaultNormThreshold = 1.0;
        public static final double kMinAmbiguityToFlip = 0.08;

        // Camera pose on the robot
        public static final double CAMERA_LEFT_DEGS = 35.0;
        public static final double CAMERA_LEFT_PITCH_RADS = Units.degreesToRadians(CAMERA_LEFT_DEGS);
        public static final Rotation2d CAMERA_LEFT_YAW = Rotation2d.k180deg;
        public static final String CAMERA_LEFT_NAME = "limelight-left";
        public static final Transform3d CAMERA_LEFT_TRANSFORM = new Transform3d(
                -0.281546, -0.266652, 0.2033524,
                new Rotation3d(0.0, CAMERA_LEFT_PITCH_RADS, CAMERA_LEFT_YAW.getRadians()));
        public static final double CAMERA_RIGHT_DEGS = 35.0;
        public static final double CAMERA_RIGHT_PITCH_RADS = Units.degreesToRadians(CAMERA_RIGHT_DEGS);
        public static final Rotation2d CAMERA_RIGHT_YAW = Rotation2d.k180deg;
        public static final String CAMERA_RIGHT_NAME = "limelight-right";
        public static final Transform3d CAMERA_RIGHT_TRANSFORM = new Transform3d(
                -0.281546, 0.266652, 0.2033524,
                new Rotation3d(0.0, CAMERA_RIGHT_PITCH_RADS, CAMERA_LEFT_YAW.getRadians()));
    }
        
    public final class Field {
        // Field Size
        public static final double FIELD_X_SIZE = Units.inchesToMeters(651.22);
        public static final double FIELD_Y_SIZE = Units.inchesToMeters(317.69);

        // Alliance Line
        public static final double ALLIANCE_LINE_X = Units.inchesToMeters(156.61);

		public static final double TRENCH_X = Units.inchesToMeters(182.11);

        // Align Point
        // LEFT
		public static final Rotation2d LEFT_APPROACH_ROTATION = Rotation2d.fromDegrees(13.64342939);
		public static final Rotation2d LEFT_SCORE_ROTATION = Rotation2d.fromDegrees(180.0 - 62.46564449);

        public static final Pose2d LEFT_APPROACH_POSE =
                new Pose2d(5.7, 7.436104, Rotation2d.k180deg);
		public static final Pose2d LEFT_APPROACH_POSE_NEXT = 
				new Pose2d(3.8, 7.436104, Rotation2d.fromDegrees(13.64342939));
        public static final Pose2d LEFT_SCORE_POINT = 
                new Pose2d(3.2, 6.769326, LEFT_SCORE_ROTATION);
        public static final Pose2d[] LEFT_POINT_POSE_ARRAY = {
                LEFT_APPROACH_POSE,
                LEFT_APPROACH_POSE_NEXT,
                LEFT_SCORE_POINT};

        // RIGHT
		public static final Rotation2d RIGHT_APPROACH_ROTATION = Rotation2d.fromDegrees(-13.64342939);
		public static final Rotation2d RIGHT_SCORE_ROTATION = Rotation2d.fromDegrees(62.46564449 - 180.0);

        public static final Pose2d RIGHT_APPROACH_POSE = 
                new Pose2d(5.7, 0.633222, Rotation2d.k180deg);
        public static final Pose2d RIGHT_APPROACH_POSE_NEXT = 
                new Pose2d(3.8, 0.633222, Rotation2d.fromDegrees(-13.64342939));
        public static final Pose2d RIGHT_SCORE_POINT = 
                new Pose2d(3.2, 1.3, RIGHT_SCORE_ROTATION);
		
        public static final Pose2d[] RIGHT_POINT_POSE_ARRAY = {
            RIGHT_APPROACH_POSE,
			RIGHT_APPROACH_POSE_NEXT,
            RIGHT_SCORE_POINT};

        // HUB tanslation
        public static final Translation2d HUB_CENTER = new Translation2d(
                Units.inchesToMeters(182.11), Units.inchesToMeters(158.84));
    }

	// Distance & Angle 
	public static final List<Pair<Double, Double>> SHOOTER_ANGLE_MAP = List.of(
			new Pair<>(2.63, 0.999999992), // 0.878906
            new Pair<>(2.73, 1.4), // 1.230469
            new Pair<>(2.85, 2.0000000000000004), // 1.933594
            new Pair<>(2.95, 2.500000000000001), // 2.460938
            new Pair<>(3.075, 2.7), // 2.636719
            new Pair<>(3.17, 3.2000000000000015), // 3.164062
            new Pair<>(3.27, 3.8000000000000016), // 3.515625
            new Pair<>(3.39, 4.300000000000001), // 4.21875
            new Pair<>(3.46, 7.999999999999988), // 7.822266
            new Pair<>(3.615, 8.399999999999986), // 8.261719
            new Pair<>(3.66, 9.699999999999982),
            new Pair<>(3.798, 13.199999999999969),
            new Pair<>(3.89, 14.899999999999963),
            new Pair<>(4.01, 18.699999999999996)
        );

	// Distance & Velocity
	public static final List<Pair<Double, Double>> SHOOTER_VELOCITY_MAP = List.of(
			new Pair<>(1.454, 2650.0),
            new Pair<>(1.5005, 2700.0),
            new Pair<>(1.55, 2750.0),
            new Pair<>(1.599, 2800.0),
            new Pair<>(1.661, 2820.0),
            new Pair<>(1.7025, 2850.0),
            new Pair<>(1.755, 2900.0),
            new Pair<>(1.815, 2940.0),
            new Pair<>(1.85, 2940.0),
            new Pair<>(1.925, 2940.0),
            new Pair<>(2.015, 3000.0),
            new Pair<>(2.135, 3000.0),
            new Pair<>(2.24, 3050.0),
            new Pair<>(2.35, 3050.0),
            new Pair<>(2.42, 3100.0),
            new Pair<>(2.54, 3150.0));
}
