package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;

public class SimTunerConstants {
    // Constants
    private static final double driveGearRatio = 7.03;
	// private static final double driveGearRatio = 1.0 / ((10.0 / 54.0) * (38.0 / 18.0) * (15.0 / 45.0));
    private static final double steerGearRatio = 287.0 / 11.0;
	// private static final double steerGearRatio = 1.0 / ((10.0 / 22.0) * (16.0 / 88.0));
    private static final double coupleGearRatio = 0.0;
    private static final Distance wheelRadius = Inches.of(1.897);
    private static final int pigeonId = 13;
    private static final boolean invertLeftSide = false;
    private static final boolean invertRightSide = true;

    private static final LinearVelocity speedAt12Volts = MetersPerSecond.of(4.54);
    private static final Current slipCurrent = Amps.of(120.0);

    // Device config
    private static final TalonFXConfiguration driveConfig =
            new TalonFXConfiguration()
                    .withMotorOutput(
                            new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
    private static final TalonFXConfiguration steerConfig =
            new TalonFXConfiguration()
                    .withCurrentLimits(
                            new CurrentLimitsConfigs()
                                    .withStatorCurrentLimit(Amps.of(60.0))
                                    .withStatorCurrentLimitEnable(true))
                    .withMotorOutput(
                            new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
    private static final CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    private static final Pigeon2Configuration pigeonConfig = null;

    private static final Slot0Configs driveGains =
            new Slot0Configs().withKP(0.1).withKI(0.0).withKD(0.0).withKS(0.0).withKV(0.124);
    private static final Slot0Configs steerGains =
            new Slot0Configs()
                    .withKP(100)
                    .withKI(0.0)
                    .withKD(0.5)
                    .withKS(0.1)
                    .withKV(0.01)
                    .withKA(0.0)
                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    // Simulation
    private static final MomentOfInertia driveInertia = KilogramSquareMeters.of(0.01);
    private static final MomentOfInertia steerInertia = KilogramSquareMeters.of(0.01);
    private static final Voltage driveFrictionVoltage = Volts.of(0.2);
    private static final Voltage steerFrictionVoltage = Volts.of(0.2);

    public static final CANBus CANBus = new CANBus("drivebase-climber", "./logs/example.hoot");

    public static final SwerveDrivetrainConstants drivetrainConstants =
            new SwerveDrivetrainConstants()
                    .withCANBusName(CANBus.getName())
                    .withPigeon2Configs(pigeonConfig)
                    .withPigeon2Id(pigeonId);

    private static final SwerveModuleConstantsFactory<
                    TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            constantsCreateor =
                    new SwerveModuleConstantsFactory<
                                    TalonFXConfiguration,
                                    TalonFXConfiguration,
                                    CANcoderConfiguration>()
                            .withDriveMotorGearRatio(driveGearRatio)
                            .withSteerMotorGearRatio(steerGearRatio)
                            .withCouplingGearRatio(coupleGearRatio)
                            .withWheelRadius(wheelRadius)
                            .withDriveMotorGains(driveGains)
                            .withSteerMotorGains(steerGains)
                            .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
                            .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
                            .withSlipCurrent(slipCurrent)
                            .withSpeedAt12Volts(speedAt12Volts)
                            .withDriveMotorType(DriveMotorArrangement.TalonFX_Integrated)
                            .withSteerMotorType(SteerMotorArrangement.TalonFX_Integrated)
                            .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
                            .withDriveMotorInitialConfigs(driveConfig)
                            .withSteerMotorInitialConfigs(steerConfig)
                            .withEncoderInitialConfigs(encoderConfig)
                            .withDriveInertia(driveInertia)
                            .withSteerInertia(steerInertia)
                            .withDriveFrictionVoltage(driveFrictionVoltage)
                            .withSteerFrictionVoltage(steerFrictionVoltage);

    // Front Left
    private static final int frontLeftDriveId = 1;
    private static final int frontLeftSteerId = 2;
    private static final int frontLeftEncoderId = 9;
    private static final Angle frontLeftEncoderOffset = Rotations.of(0.0);
    private static final boolean frontLeftSteerInverted = true;
    private static final boolean frontLeftEncoderInverted = false;

    private static final Distance frontLeftXPos = Inches.of(11.375);
    private static final Distance frontLeftYPos = Inches.of(11.375);

    // Front Righ
    private static final int frontRightDriveId = 3;
    private static final int frontRightSteerId = 4;
    private static final int frontRightEncoderId = 10;
    private static final Angle frontRightEncoderOffset = Rotations.of(0.0);
    private static final boolean frontRightSteerInverted = true;
    private static final boolean frontRightEncoderInverted = false;

    private static final Distance frontRightXPos = Inches.of(11.375);
    private static final Distance frontRightYPos = Inches.of(-11.375);

    // Back Left
    private static final int backLeftDriveId = 5;
    private static final int backLeftSteerId = 6;
    private static final int backLeftEncoderId = 11;
    private static final Angle backLeftEncoderOffset = Rotations.of(0.0);
    private static final boolean backLeftSteerInverted = true;
    private static final boolean backLeftEncoderInverted = false;

    private static final Distance backLeftXPos = Inches.of(-11.375);
    private static final Distance backLeftYPos = Inches.of(11.375);

    // Back Right
    private static final int backRightDriveId = 7;
    private static final int backRightSteerId = 8;
    private static final int backRightEncoderId = 12;
    private static final Angle backRightEncoderOffset = Rotations.of(0.0);
    private static final boolean backRightSteerInverted = true;
    private static final boolean backRightEncoderInverted = false;

    private static final Distance backRightXPos = Inches.of(-11.375);
    private static final Distance backRightYPos = Inches.of(-11.375);

    public static final SwerveModuleConstants<
                    TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            frontLeft =
                    constantsCreateor.createModuleConstants(
                            frontLeftSteerId,
                            frontLeftDriveId,
                            frontLeftEncoderId,
                            frontLeftEncoderOffset,
                            frontLeftXPos,
                            frontLeftYPos,
                            invertLeftSide,
                            frontLeftSteerInverted,
                            frontLeftEncoderInverted);
    public static final SwerveModuleConstants<
                    TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            frontRight =
                    constantsCreateor.createModuleConstants(
                            frontRightSteerId,
                            frontRightDriveId,
                            frontRightEncoderId,
                            frontRightEncoderOffset,
                            frontRightXPos,
                            frontRightYPos,
                            invertRightSide,
                            frontRightSteerInverted,
                            frontRightEncoderInverted);
    public static final SwerveModuleConstants<
                    TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            backLeft =
                    constantsCreateor.createModuleConstants(
                            backLeftSteerId,
                            backLeftDriveId,
                            backLeftEncoderId,
                            backLeftEncoderOffset,
                            backLeftXPos,
                            backLeftYPos,
                            invertLeftSide,
                            backLeftSteerInverted,
                            backLeftEncoderInverted);
    public static final SwerveModuleConstants<
                    TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            backRight =
                    constantsCreateor.createModuleConstants(
                            backRightSteerId,
                            backRightDriveId,
                            backRightEncoderId,
                            backRightEncoderOffset,
                            backRightXPos,
                            backRightYPos,
                            invertRightSide,
                            backRightSteerInverted,
                            backRightEncoderInverted);

    public static final CommandSwerveDrivetrain createTrain() {
        return new CommandSwerveDrivetrain(
                drivetrainConstants, frontLeft, frontRight, backLeft, backRight);
    }

    public static class TunerSwerveDriveTrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
        public TunerSwerveDriveTrain(
                SwerveDrivetrainConstants drivetrainConstants,
                SwerveModuleConstants<?, ?, ?>... modules) {
            super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, modules);
        }

        public TunerSwerveDriveTrain(
                SwerveDrivetrainConstants drivetrainConstants,
                double odometryUpdateFrequency,
                SwerveModuleConstants<?, ?, ?>... modules) {
            super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, modules);
        }

        public TunerSwerveDriveTrain(
                SwerveDrivetrainConstants drivetrainConstants,
                double odometryUpdateFrequency,
                Matrix<N3, N1> odometryStandardDeviation,
                Matrix<N3, N1> visionStandardDeviation,
                SwerveModuleConstants<?, ?, ?>... modules) {
            super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, modules);
        }
    }
}
