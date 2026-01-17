// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
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
    
    public final class Intake {
    
    }

    public final class Shooter {
    
    }

    public final class Vision {
        
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
