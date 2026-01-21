package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class FlywheelIOHardware implements FlywheelIO {
    private final TalonFX flywheel;
    private final VoltageOut voltageOut = new VoltageOut(0.0);
    private final VelocityVoltage velocityControl =
            new VelocityVoltage(0.0).withUpdateFreqHz(0.0);

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> volts;
    private final StatusSignal<Current> supplyCurrent;

    public FlywheelIOHardware(int id) {
        this.flywheel = new TalonFX(id);

        this.position = this.flywheel.getPosition();
        this.velocity = this.flywheel.getVelocity();
        this.volts = this.flywheel.getMotorVoltage();
        this.supplyCurrent = this.flywheel.getSupplyCurrent();

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(40.0);
        config.MotorOutput
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.positionRads = this.position.getValueAsDouble();
        inputs.velocityRPS = this.velocity.getValueAsDouble();
        inputs.appliedVolts = this.volts.getValueAsDouble();
        inputs.supplyCurrent = this.supplyCurrent.getValueAsDouble();
        inputs.connected = this.flywheel.isConnected();
    }

    @Override
    public void setVolts(double volts) {
        this.flywheel.setControl(this.voltageOut.withOutput(volts));
    }

    @Override
    public void runVelocity(double rpm, double feedforward) {
        this.flywheel.setControl(
                this.velocityControl.withVelocity(rpm / 60.0).withFeedForward(feedforward));
    }

    @Override
    public void stop() {
        this.flywheel.stopMotor();
    }
}
