package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.Constants.Shooter;

public class FlywheelIOSim implements FlywheelIO {
    private final TalonFX flywheel;
    private final TalonFXSimState simState;
    private final FlywheelSim sim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                    DCMotor.getKrakenX60(2),
                    0.001,
                    Shooter.FLYWHEEL_GEAR_RATIO),
            DCMotor.getKrakenX60(2));

    private final VoltageOut voltageOut = new VoltageOut(0.0);
    private final VelocityVoltage velocityControl =
            new VelocityVoltage(0.0).withUpdateFreqHz(0.0);

    public FlywheelIOSim() {
        this.flywheel = new TalonFX(Shooter.FLYWHEEL_MAIN_ID);

        this.flywheel.getConfigurator().apply(Shooter.FLYWHEEL_CONFIG);
        this.simState = this.flywheel.getSimState();
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        this.sim.update(Constants.ROBOT_PERIODIC);
        this.sim.setInputVoltage(this.simState.getMotorVoltage());

        inputs.positionRads += this.sim.getAngularVelocityRadPerSec() * 0.02;
        inputs.velocityRPS = this.sim.getAngularVelocityRPM() / 60.0;
        inputs.appliedVolts = this.simState.getMotorVoltage();
        inputs.supplyCurrent = this.sim.getCurrentDrawAmps();
        inputs.connected = true;

        double velocity = Units.radiansToRotations(this.sim.getAngularVelocityRadPerSec())
                * Shooter.FLYWHEEL_GEAR_RATIO;
        this.simState.setRotorVelocity(velocity);
        this.simState.addRotorPosition(velocity * 0.02);
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
