package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class LifterIOHardware implements LifterIO {
    private final TalonFX lifter;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> supplyCurrent;

    public LifterIOHardware(int id) {
        this.lifter = new TalonFX(id);
        this.position = this.lifter.getPosition();
        this.velocity = this.lifter.getVelocity();
        this.appliedVolts = this.lifter.getMotorVoltage();
        this.supplyCurrent = this.lifter.getSupplyCurrent();

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(40.0);
        config.MotionMagic
                .withMotionMagicCruiseVelocity(1.0)
                .withMotionMagicAcceleration(10.0)
                .withMotionMagicJerk(2000.0);
        config.MotorOutput
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);
        config.DifferentialSensors
                .withSensorToDifferentialRatio(Constants.Intake.LIFTER_GEAR_RATIO);
        config.Slot0
                .withKP(0.0)
                .withKS(0.0)
                .withKV(0.0)
                .withKG(0.0)
                .withKA(0.0);

        this.lifter.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(LifterIOInputs inputs) {
        inputs.positionRads = this.position.getValueAsDouble();
        inputs.velocityRPS = this.velocity.getValueAsDouble();
        inputs.appliedVolts = this.appliedVolts.getValueAsDouble();
        inputs.supplyCurrent = this.supplyCurrent.getValueAsDouble();
        inputs.connected = this.lifter.isConnected();

        BaseStatusSignal.refreshAll(
                this.position,
                this.velocity,
                this.appliedVolts,
                this.supplyCurrent);
                
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
