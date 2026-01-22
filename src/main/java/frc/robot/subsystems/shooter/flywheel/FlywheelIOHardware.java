package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class FlywheelIOHardware implements FlywheelIO {
    private final TalonFX flywheelMain;
    private final TalonFX flywheelFollow;
    private final Follower follower;
    private final VoltageOut voltageOut = new VoltageOut(0.0);
    private final VelocityVoltage velocityControl =
            new VelocityVoltage(0.0).withUpdateFreqHz(0.0);

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> volts;
    private final StatusSignal<Current> supplyCurrent;

    public FlywheelIOHardware(int main, int follow) {
        this.flywheelMain = new TalonFX(main);
        this.flywheelFollow = new TalonFX(follow);
        this.follower = new Follower(this.flywheelMain.getDeviceID(), MotorAlignmentValue.Opposed);

        this.position = this.flywheelMain.getPosition();
        this.velocity = this.flywheelMain.getVelocity();
        this.volts = this.flywheelMain.getMotorVoltage();
        this.supplyCurrent = this.flywheelMain.getSupplyCurrent();

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(60.0);
        config.MotorOutput
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);
        
        this.flywheelMain.getConfigurator().apply(config);
        this.flywheelFollow.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.positionRads = this.position.getValueAsDouble();
        inputs.velocityRPS = this.velocity.getValueAsDouble();
        inputs.appliedVolts = this.volts.getValueAsDouble();
        inputs.supplyCurrent = this.supplyCurrent.getValueAsDouble();
        inputs.connected = this.flywheelMain.isConnected();

        BaseStatusSignal.refreshAll(
                this.position,
                this.velocity,
                this.volts,
                this.supplyCurrent);
    }

    @Override
    public void setVolts(double volts) {
        this.flywheelMain.setControl(this.voltageOut.withOutput(volts));
        this.flywheelFollow.setControl(this.follower);
    }

    @Override
    public void runVelocity(double rpm, double feedforward) {
        this.flywheelMain.setControl(
                this.velocityControl.withVelocity(rpm / 60.0).withFeedForward(feedforward));
        this.flywheelFollow.setControl(this.follower);
    }

    @Override
    public void stop() {
        this.flywheelMain.stopMotor();
        this.flywheelFollow.stopMotor();
    }
}
