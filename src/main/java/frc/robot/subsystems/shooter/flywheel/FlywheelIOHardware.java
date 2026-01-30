package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.Shooter;

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

    public FlywheelIOHardware() {
        this.flywheelMain = new TalonFX(Shooter.FLYWHEEL_MAIN_ID);
        this.flywheelFollow = new TalonFX(Shooter.FLYWHEEL_FOLLOW_ID);
        this.follower = new Follower(this.flywheelMain.getDeviceID(), MotorAlignmentValue.Opposed);

        this.position = this.flywheelMain.getPosition();
        this.velocity = this.flywheelMain.getVelocity();
        this.volts = this.flywheelMain.getMotorVoltage();
        this.supplyCurrent = this.flywheelMain.getSupplyCurrent();
        
        this.flywheelMain.getConfigurator().apply(Shooter.FLYWHEEL_CONFIG);
        this.flywheelFollow.getConfigurator().apply(Shooter.FLYWHEEL_CONFIG);
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                this.position,
                this.velocity,
                this.volts,
                this.supplyCurrent);
                
        inputs.positionRads = this.position.getValueAsDouble();
        inputs.velocityRPS = this.velocity.getValueAsDouble();
        inputs.appliedVolts = this.volts.getValueAsDouble();
        inputs.supplyCurrent = this.supplyCurrent.getValueAsDouble();
        inputs.connected = this.flywheelMain.isConnected();
    }

    @Override
    public void setVolts(double volts) {
        this.flywheelMain.setControl(this.voltageOut.withOutput(volts));
        this.flywheelFollow.setControl(this.follower);
    }

    @Override
    public void runVelocity(double rpm, double feedforward) {
        if (rpm == 0.0) {
            this.flywheelMain.stopMotor();
            this.flywheelFollow.setControl(this.follower);
            return;
        }
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
