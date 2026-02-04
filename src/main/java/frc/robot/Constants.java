package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class Constants {
  	public final class Intake {
        public static final int LIFTER_ID = 25;
        public static final int ROLLER_ID = 26;
        
        public static final double LIFTER_GEAR_RATIO = 24.0 / 5.0;
		public static final double DRUM_CIRCUMFERENCE = 10.0 * 0.00798;
		public static final double DRUM_RADIUS_METERS = DRUM_CIRCUMFERENCE / (2.0 * Math.PI);
		public static final double MECHANISM_GEAR_RATIO = LIFTER_GEAR_RATIO / DRUM_CIRCUMFERENCE;
        public static final double ROLLER_GEAR_RATIO = 1.0;
        // TODO 2/5
        public static final double LIFTER_ANGLE_TOLERANCE = 0.1;

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
                                .withInverted(InvertedValue.CounterClockwise_Positive)
                                .withNeutralMode(NeutralModeValue.Brake))
                .withFeedback(
                        new FeedbackConfigs()
                                .withSensorToMechanismRatio(MECHANISM_GEAR_RATIO))
                .withSlot0( // TODO 2/5
                        new Slot0Configs()
                                .withKP(10.0)
                                .withKS(0.0)
                                .withKV(0.0)
                                .withKG(0.0)
                                .withKA(0.0));
    }
}
