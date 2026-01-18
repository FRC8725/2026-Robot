package frc.robot.lib.simulation;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

public class MapleSimDrivetrain {
    private final Pigeon2SimState pigeonSim;
    private final SimSwerveModule[] simModules;
    public final SwerveDriveSimulation mapleSimDrive;

    public MapleSimDrivetrain(
            Time simPeriod,
            Mass robotMassWithBumper,
            Distance bumperLengthX,
            Distance bumperLengthY,
            double wheelCOF,
            Translation2d[] moduleTranslations,
            Pigeon2 pigeon2,
            SwerveModule<TalonFX, TalonFX, CANcoder>[] modules,
            @SuppressWarnings("unchecked")
                    SwerveModuleConstants<
                                    TalonFXConfiguration,
                                    TalonFXConfiguration,
                                    CANcoderConfiguration>...
                            moduleConstants) {
        this.pigeonSim = pigeon2.getSimState();
        this.simModules = new SimSwerveModule[moduleConstants.length];
        DriveTrainSimulationConfig config =
                DriveTrainSimulationConfig.Default()
                        .withBumperSize(bumperLengthX, bumperLengthY)
                        .withCustomModuleTranslations(moduleTranslations)
                        .withGyro(COTS.ofPigeon2())
                        .withRobotMass(robotMassWithBumper)
                        .withSwerveModule(
                                new SwerveModuleSimulationConfig(
                                        DCMotor.getKrakenX60(1),
                                        DCMotor.getKrakenX60(1),
                                        moduleConstants[0].DriveMotorGearRatio,
                                        moduleConstants[0].SteerMotorGearRatio,
                                        Volts.of(moduleConstants[0].DriveFrictionVoltage),
                                        Volts.of(moduleConstants[0].SteerFrictionVoltage),
                                        Meters.of(moduleConstants[0].WheelRadius),
                                        KilogramSquareMeters.of(moduleConstants[0].SteerInertia),
                                        wheelCOF));
        this.mapleSimDrive = new SwerveDriveSimulation(config, new Pose2d());

        SwerveModuleSimulation[] moduleSimulations = this.mapleSimDrive.getModules();
        for (int i = 0; i < this.simModules.length; i++)
            this.simModules[i] =
                    new SimSwerveModule(moduleConstants[0], moduleSimulations[i], modules[i]);

        SimulatedArena.overrideSimulationTimings(simPeriod, 1);
        SimulatedArena.getInstance().addDriveTrainSimulation(this.mapleSimDrive);
    }

    public void update() {
        SimulatedArena.getInstance().simulationPeriodic();
        this.pigeonSim.setRawYaw(
                this.mapleSimDrive.getSimulatedDriveTrainPose().getRotation().getMeasure());
        this.pigeonSim.setAngularVelocityZ(
                RadiansPerSecond.of(
                        this.mapleSimDrive.getDriveTrainSimulatedChassisSpeedsRobotRelative()
                                .omegaRadiansPerSecond));
    }

    public static SwerveModuleConstants<?, ?, ?>[] regulateModuleConstantsForSimulation(
            SwerveModuleConstants<?, ?, ?>[] moduleConstants) {
        for (SwerveModuleConstants<?, ?, ?> module : moduleConstants)
            regulateModuleConstantForSimulation(module);

        return moduleConstants;
    }

    private static void regulateModuleConstantForSimulation(
            SwerveModuleConstants<?, ?, ?> moduleConstants) {
        if (RobotBase.isReal()) return;

        moduleConstants
                .withEncoderOffset(0.0)
                // Disable motor inversions for drive and steer motors
                .withDriveMotorInverted(false)
                .withSteerMotorInverted(false)
                // Disable CanCoder inversion
                .withEncoderInverted(false)
                // Adjust steer motor PID gains for simulation
                .withSteerMotorGains(
                        moduleConstants
                                .SteerMotorGains
                                        .withKP(50.0).withKI(0).withKD(0.0)
                                        .withKS(0.0).withKV(0.0).withKA(0))
                                // .withKP(100.0) // Proportional gain
                                // .withKD(0.5)) // Derivative gain
                // Adjust friction voltages
                .withDriveFrictionVoltage(Volts.of(0.2))
                .withSteerFrictionVoltage(Volts.of(0.15))
                // Adjust steer inertia
                .withSteerInertia(KilogramSquareMeters.of(0.05));
    }

    protected static class SimSwerveModule {
        public final SwerveModuleConstants<
                        TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
                moduleConstants;
        public final SwerveModuleSimulation moduleSimulation;

        public SimSwerveModule(
                SwerveModuleConstants<
                                TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
                        moduleConstants,
                SwerveModuleSimulation moduleSimulation,
                SwerveModule<TalonFX, TalonFX, CANcoder> module) {
            this.moduleConstants = moduleConstants;
            this.moduleSimulation = moduleSimulation;
            moduleSimulation.useDriveMotorController(
                    new TalonFXMotorControlSim(module.getDriveMotor()));
            moduleSimulation.useSteerMotorController(
                    new TalonFXMotorControlWithCANCoderSim(
                            module.getSteerMotor(), module.getEncoder()));
        }
    }

    public static class TalonFXMotorControlSim implements SimulatedMotorController {
        public final int id;

        private final TalonFXSimState simState;

        public TalonFXMotorControlSim(TalonFX talonFX) {
            this.id = talonFX.getDeviceID();
            this.simState = talonFX.getSimState();
        }

        @Override
        public Voltage updateControlSignal(
                Angle mechanismAngle,
                AngularVelocity mechanismVelocity,
                Angle encoderAngle,
                AngularVelocity encoderVelocity) {
            this.simState.setRawRotorPosition(encoderAngle);
            this.simState.setRotorVelocity(encoderVelocity);
            this.simState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());

            return this.simState.getMotorVoltageMeasure();
        }
    }

    public static class TalonFXMotorControlWithCANCoderSim extends TalonFXMotorControlSim {
        private final CANcoderSimState canCoderSimState;

        public TalonFXMotorControlWithCANCoderSim(TalonFX talonFX, CANcoder canCoder) {
            super(talonFX);
            this.canCoderSimState = canCoder.getSimState();
        }

        @Override
        public Voltage updateControlSignal(
                Angle mechanismAngle,
                AngularVelocity mechanismVelocity,
                Angle encoderAngle,
                AngularVelocity encoderVelocity) {
            this.canCoderSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
            this.canCoderSimState.setRawPosition(mechanismAngle);
            this.canCoderSimState.setVelocity(mechanismVelocity);
            return super.updateControlSignal(
                    mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity);
        }
    }
}
