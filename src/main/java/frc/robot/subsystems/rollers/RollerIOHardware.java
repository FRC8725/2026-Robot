package frc.robot.subsystems.rollers;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class RollerIOHardware implements RollerIO {
    private final TalonFX roller;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> supplyCurrent;

    public RollerIOHardware(int id, boolean reverse) {
        this.roller = new TalonFX(id);

        this.position = this.roller.getPosition();
        this.velocity = this.roller.getVelocity();
        this.appliedVolts = this.roller.getMotorVoltage();
        this.supplyCurrent = this.roller.getSupplyCurrent();

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(40.0);
        config.MotorOutput
                .withInverted(
                        reverse 
                                ? InvertedValue.Clockwise_Positive
                                : InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);
        config.Feedback
                .withVelocityFilterTimeConstant(0.1);

        this.roller.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        inputs.positionRads = this.position.getValueAsDouble();
        inputs.velocityRPS = this.velocity.getValueAsDouble();
        inputs.appliedVolts = this.appliedVolts.getValueAsDouble();
        inputs.supplyCurrent = this.supplyCurrent.getValueAsDouble();
        inputs.connected = this.roller.isConnected();
    }

    @Override
    public void setVolts(double volts) {
        this.roller.setVoltage(volts);
    }

    @Override
    public void stop() {
        this.roller.stopMotor();
    }
}
