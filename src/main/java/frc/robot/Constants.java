// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
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
        public static final double MAX_ANGULAR_RATE = Math.PI * 2; // 1 rotation per second
        public static final double STEER_JOYSTICK_DEADBAND = 0.05;
        public static final CommandSwerveDrivetrain drivetrain =
                RobotBase.isSimulation()
                        ? SimTunerConstants.createTrain()
                        : TunerConstants.createDrivetrain();
        
    }
    
    public final class Intake {
        public static final int LIFTER_ID = 0;
        public static final int ROLLER_ID = 0;
        
        public static final double LIFTER_GEAR_RATIO = 1.0;
        public static final double ROLLER_GEAR_RATIO = 1.0;
        public static final double LIFTER_ANGLE_TOLERANCE = 0.01;

        public static final double LIFTER_LIMIT_DISTANCE = 0.0;

        public static final TalonFXConfiguration LIFTER_CONFIG = new TalonFXConfiguration()
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withSupplyCurrentLimitEnable(true)
                                .withSupplyCurrentLimit(40.0))
                .withMotionMagic(
                        new MotionMagicConfigs()
                                .withMotionMagicCruiseVelocity(1.0)
                                .withMotionMagicAcceleration(10.0)
                                .withMotionMagicJerk(2000.0))
                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withInverted(InvertedValue.Clockwise_Positive)
                                .withNeutralMode(NeutralModeValue.Brake))
                .withFeedback(
                        new FeedbackConfigs()
                                .withSensorToMechanismRatio(LIFTER_GEAR_RATIO))
                .withSlot0(
                        new Slot0Configs()
                                .withKP(0.0)
                                .withKS(0.0)
                                .withKV(0.0)
                                .withKG(0.0)
                                .withKA(0.0));
    }

    public final class Shooter {
		public static final int HOOD_ID = 0;
		public static final int FLYWHEEL_MAIN_ID = 0;
		public static final int FLYWHEEL_FOLLOW_ID = 0;
		public static final int FEEDER_ID = 0;

        public static final double HOOD_GEAR_RATIO = 17.5 * 75.0 / 8.0;
		public static final double FLYWHEEL_GEAR_RATIO = 1.0;
		
		public static final double TOLERANCE = 3.0;

        public static final TalonFXConfiguration FLYWHEEL_CONFIG = new TalonFXConfiguration()
                .withCurrentLimits(
						new CurrentLimitsConfigs()
								.withStatorCurrentLimitEnable(true)
								.withStatorCurrentLimit(60.0))
				.withMotorOutput(
						new MotorOutputConfigs()
								.withInverted(InvertedValue.CounterClockwise_Positive)
								.withNeutralMode(NeutralModeValue.Coast))
				.withSlot0(
						new Slot0Configs()
								.withKP(0.1));
		public static final TalonFXConfiguration HOOD_CONFIG = new TalonFXConfiguration()
				.withCurrentLimits(
						new CurrentLimitsConfigs()
								.withStatorCurrentLimitEnable(true)
                				.withStatorCurrentLimit(70.0)
                				.withSupplyCurrentLimitEnable(true)
                				.withSupplyCurrentLimit(50.0))
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
								.withKS(0.25)
								.withKV(13.0)
								.withKA(0.0)
								.withKG(0.0)
								.withKP(250.0)
								.withKD(0.0));
    }

	public final class Hopper {
		public static final int ID = 0;
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
        public static final double CAMERA_LEFT_DEGS = 45.0;
        public static final double CAMERA_LEFT_PITCH_RADS = Units.degreesToRadians(CAMERA_LEFT_DEGS);
        public static final Rotation2d CAMERA_LEFT_YAW = Rotation2d.kZero;
        public static final String CAMERA_LEFT_NAME = "limelight-left";
        public static final Transform3d CAMERA_LEFT_TRANSFORM = new Transform3d(
                0.306542, 0.076121, 0.120064,
                new Rotation3d(0.0, CAMERA_LEFT_PITCH_RADS, CAMERA_LEFT_YAW.getRadians()));
        public static final double CAMERA_RIGHT_DEGS = 45.0;
        public static final double CAMERA_RIGHT_PITCH_RADS = Units.degreesToRadians(CAMERA_RIGHT_DEGS);
        public static final Rotation2d CAMERA_RIGHT_YAW = Rotation2d.kZero;
        public static final String CAMERA_RIGHT_NAME = "limelight-right";
        public static final Transform3d CAMERA_RIGHT_TRANSFORM = new Transform3d(
                0.306542, -0.076121, 0.120064,
                new Rotation3d(0.0, CAMERA_RIGHT_PITCH_RADS, CAMERA_LEFT_YAW.getRadians()));
    }
        
    public final class Field {
        // Field Size
        public static final double FIELD_X_SIZE = Units.inchesToMeters(651.22);
        public static final double FIELD_Y_SIZE = Units.inchesToMeters(317.69);

        // HUB tanslation
        public static final Translation2d HUB_CENTER = new Translation2d(
                Units.inchesToMeters(182.11), Units.inchesToMeters(158.84));
    }
}
