package frc.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.Shooter;

public class HoodIOHardware implements HoodIO {
    private final TalonFX lifter;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> volts;
    private final StatusSignal<Current> supplyCurrent;
    
    public HoodIOHardware() {
        this.lifter = new TalonFX(Shooter.HOOD_ID);

        this.lifter.getConfigurator().apply(Constants.Shooter.HOOD_CONFIG);

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
        return Units.rotationsToRadians(this.lifter.getPosition().getValueAsDouble());
    }
}
