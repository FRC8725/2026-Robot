package frc.robot.subsystems.intake.lifter;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import frc.robot.Constants.Intake;

public class LifterIOSim implements LifterIO {
    private final TalonFX lifter;
    private final TalonFXSimState simState;
    private final ElevatorSim sim = new ElevatorSim(
        DCMotor.getFalcon500(1),
        Intake.LIFTER_GEAR_RATIO, 
        2.0,
        Intake.DRUM_RADIUS_METERS,
        0.0,
        0.5,
        false,
        0.0);

    public LifterIOSim() {
        this.lifter = new TalonFX(Intake.LIFTER_ID);

        this.lifter.getConfigurator().apply(Intake.LIFTER_CONFIG);
        this.simState = this.lifter.getSimState();
    }

    @Override
    public void updateInputs(LifterIOInputs inputs) {
        this.sim.setInputVoltage(this.simState.getMotorVoltage());
        this.sim.update(Constants.ROBOT_PERIODIC);

        inputs.positionLength = this.lifter.getPosition().getValueAsDouble();
        inputs.velocityRPS = this.lifter.getVelocity().getValueAsDouble();
        inputs.appliedVolts = this.simState.getMotorVoltage();
        inputs.supplyCurrent = this.sim.getCurrentDrawAmps();
        inputs.connected = true;   

        this.simState.setRawRotorPosition(this.sim.getPositionMeters() * Intake.MECHANISM_GEAR_RATIO);
        this.simState.setRotorVelocity(this.sim.getVelocityMetersPerSecond() * Intake.MECHANISM_GEAR_RATIO);
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
