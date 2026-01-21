package frc.robot.subsystems.rollers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class RollerIOSim implements RollerIO {
    private final DCMotorSim sim;
    private final DCMotor gearbox;
    private double appliedVolts = 0.0;

    public RollerIOSim(DCMotor motor, double reduction, double moi) {
        this.gearbox = motor;
        this.sim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(motor, reduction, moi), motor);
    }

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        this.sim.update(0.02);

        inputs.positionRads = this.sim.getAngularPositionRad();
        inputs.velocityRPS = this.sim.getAngularVelocityRPM() / 60.0;
        inputs.appliedVolts = this.appliedVolts;
        inputs.supplyCurrent = this.sim.getCurrentDrawAmps();
        inputs.connected = true;
    }

    @Override
    public void setVolts(double volts) {
        if (DriverStation.isDisabled()) {
            this.appliedVolts = 0.0;
        } else {
            this.appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        }
        this.sim.setInputVoltage(this.appliedVolts);
    }

    @Override
    public void stop() {
        this.sim.setInputVoltage(0.0);
    }
}
