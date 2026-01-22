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

        this.position = this.lifter.getPosition();
        this.velocity = this.lifter.getVelocity();
        this.volts = this.lifter.getMotorVoltage();
        this.supplyCurrent = this.lifter.getSupplyCurrent();

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(70.0)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(50.0);
        config.MotionMagic
                .withMotionMagicCruiseVelocity(0.5)
                .withMotionMagicAcceleration(2000.0)
                .withMotionMagicJerk(2000.0);
        config.MotorOutput
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);
        config.Feedback
                .withSensorToMechanismRatio(Constants.Shooter.GEAR_RATIO);
        
        Slot0Configs slot0 = new Slot0Configs();
        slot0.kS = 0.1;
        slot0.kV = 0.1;
        slot0.kA = 0.0;
        slot0.kG = 0.0;
        slot0.kP = 10.0;
        config.Slot0 = slot0;

        this.lifter.getConfigurator().apply(config);
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
    public void stop() {
        this.lifter.stopMotor();
    }
}
