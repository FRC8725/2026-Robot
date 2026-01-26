// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

    public static final double kRobotMassKg = Units.lbsToKilograms(150.0);
    public static final double kRobotMomentOfInertia = 2 * 9.38;
    public static final double kCOGHeightMeters = Units.inchesToMeters(0.0);
 
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
        public static final double kWheelCoefficientOfFriction = 1.0;
        public static final CommandSwerveDrivetrain drivetrain =
                RobotBase.isSimulation()
                        ? SimTunerConstants.createTrain()
                        : TunerConstants.createDrivetrain();

        // Phycical Limit
        public static final double MAX_MOTOR_RPM = 4675.0;
        public static final double MAX_VELOCITY = (MAX_MOTOR_RPM / 60.0) / TunerConstants.kDriveGearRatio * 2.0 * Units.inchesToMeters(1.897) * Math.PI;
        public static final double MAX_ANGULAR_VELOCITY = MAX_VELOCITY / (Units.inchesToMeters(12.75) * Math.sqrt(2.0)); // TODO: Research HOW

        // Align weight
        public static final double ALIGN_TRANSLATION_WEIGHT = 5.0;
        public static final double ALIGN_ANGLE_WEIGHT = 2.7;
        public static final double MAX_TURN_ANGLE = 100.0;

        // Tolerance
        public static final double STRATING_TOLERANCE = 0.15;
        public static final double ALIGNMENT_TOLERANCE = 0.02;
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
