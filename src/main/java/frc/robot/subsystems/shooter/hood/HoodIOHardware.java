package frc.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class HoodIOHardware implements HoodIO {
    private final TalonFX lifter;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> volts;
    private final StatusSignal<Current> supplyCurrent;
    
    public HoodIOHardware(int id) {
        this.lifter = new TalonFX(id);

        TalonFXConfiguration config = new TalonFXConfiguration();
        Slot0Configs slot0 = new Slot0Configs();
        slot0.kS = 0.25;
        slot0.kV = 13.0;
        slot0.kA = 0.0;
        slot0.kG = 0.0;
        slot0.kP = 250.0;
        slot0.kD = 0.0;
        config.CurrentLimits
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(70.0)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(50.0);
        config.MotionMagic
                .withMotionMagicCruiseVelocity(1.0)
                .withMotionMagicAcceleration(200.0)
                .withMotionMagicJerk(2000.0);
        config.MotorOutput
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);
        config.Feedback
                .withSensorToMechanismRatio(Constants.Shooter.HOOD_GEAR_RATIO);
        config.Slot0 = slot0;

        this.lifter.getConfigurator().apply(config);

        this.position = this.lifter.getPosition();
        this.velocity = this.lifter.getVelocity();
        this.volts = this.lifter.getMotorVoltage();
        this.supplyCurrent = this.lifter.getSupplyCurrent();
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                this.position,
                this.velocity,
                this.volts,
                this.supplyCurrent);

        inputs.positionRads = Units.rotationsToRadians(this.position.getValueAsDouble());
        inputs.velocityRPS = this.velocity.getValueAsDouble();
        inputs.appliedVolts = this.volts.getValueAsDouble();
        inputs.supplyCurrent = this.supplyCurrent.getValueAsDouble();
        inputs.connected = this.lifter.isConnected();
    }

    @Override
    public void setControl(MotionMagicVoltage requst) {
        this.lifter.setControl(requst);
    }

    @Override
    public void setZeroPosition() {
        this.lifter.setPosition(0.0);
    }

    @Override
    public void setVolts(double volts) {
        this.lifter.setVoltage(volts);
    }

    @Override
    public void stop() {
        this.lifter.stopMotor();
    }

    @Override
    public double getPosition() {
        return this.lifter.getPosition().getValueAsDouble();
    }
}
