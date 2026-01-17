// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class RobotMode {
        public static final boolean isReplay = false;

        public enum Mode {
            REAL,
            SIM,
            REPLAY
        }
    }
    
    public final class Swerve {
        // Module
        public static final double TRACK_WIDTH = Units.inchesToMeters(12.75);
        public static final double TRACK_LENGTH = Units.inchesToMeters(12.75);
        public static final double WHEEL_RADIUS = Units.inchesToMeters(3.935 / 2.0);
        public static final double DRIVE_GEAR_RATIO = 57.0 / 7.0;
        public static final double TURN_GEAR_RATIO = 150.0 / 7.0;

        // Phycical Limit
        public static final double MAX_MOTOR_RPM = 4675.0;
        public static final double MAX_VELOCITY = (MAX_MOTOR_RPM / 60.0) / DRIVE_GEAR_RATIO * 2.0 * WHEEL_RADIUS * Math.PI;
        public static final double MAX_ANGULAR_VELOCITY = MAX_VELOCITY / (TRACK_WIDTH * Math.sqrt(2.0)); // TODO: Research HOW

        // Align Limit
        public static final double MAX_BARGE_ALIGN_TRANSLATION_SPEED = 1.5;
        public static final double MAX_BARGE_ALIGN_ROTAITON_SPEED = 1.5;
        public static final double MAX_ALIGN_TRANSLATION_SPEED = 1.5;

        // Tolerance
        public static final double DEAD_BAND = 0.05;
        public static final double STRATING_TOLERANCE = 0.15;
        public static final double ALIGNMENT_TOLERANCE = 0.02;

        // Align weight
        public static final double ALIGN_TRANSLATION_WEIGHT = 5.0;
        public static final double ALIGN_ANGLE_WEIGHT = 2.7;

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(TRACK_LENGTH / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(TRACK_LENGTH / 2.0, -TRACK_WIDTH / 2.0),
            new Translation2d(-TRACK_LENGTH / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(-TRACK_LENGTH / 2.0, -TRACK_LENGTH / 2.0));
    }
    
    public final class Intake {
    
    }

    public final class Shooter {
        public static final double TOLERANCE = 5.0;
    }

    public final class Vision {
        private final Transform3d object_camera = new Transform3d();
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
