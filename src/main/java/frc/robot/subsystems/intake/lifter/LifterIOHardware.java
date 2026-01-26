package frc.robot.subsystems.intake.lifter;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.Intake;

public class LifterIOHardware implements LifterIO {
    private final TalonFX lifter;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> supplyCurrent;

    public LifterIOHardware() {
        this.lifter = new TalonFX(Intake.LIFTER_ID);
        this.position = this.lifter.getPosition();
        this.velocity = this.lifter.getVelocity();
        this.appliedVolts = this.lifter.getMotorVoltage();
        this.supplyCurrent = this.lifter.getSupplyCurrent();

        this.lifter.getConfigurator().apply(Intake.LIFTER_CONFIG);
    }

    @Override
    public void updateInputs(LifterIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                this.position,
                this.velocity,
                this.appliedVolts,
                this.supplyCurrent);

        inputs.positionRads = Units.rotationsToRadians(this.position.getValueAsDouble());
        inputs.velocityRPS = this.velocity.getValueAsDouble();
        inputs.appliedVolts = this.appliedVolts.getValueAsDouble();
        inputs.supplyCurrent = this.supplyCurrent.getValueAsDouble();
        inputs.connected = this.lifter.isConnected();
                
        Logger.recordOutput("Intake/Lifter/PositionRads", this.position.getValueAsDouble());
        Logger.recordOutput("Intake/Lifter/VelocityRPS", this.velocity.getValueAsDouble());
        Logger.recordOutput("Intake/Lifter/AppliedVolts", this.appliedVolts.getValueAsDouble());
        Logger.recordOutput("Intake/Lifter/SupplyCurrent", this.supplyCurrent.getValueAsDouble());
    }

    @Override
    public void setControl(MotionMagicVoltage request) {
        this.lifter.setControl(request);
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
