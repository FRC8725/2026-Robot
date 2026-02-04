package frc.robot.subsystems.intake.lifter;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Constants.Intake;

public class LifterIOSim implements LifterIO {
    private final TalonFX lifter;
    private final TalonFXSimState simState;
    private final SingleJointedArmSim sim = new SingleJointedArmSim(
            DCMotor.getFalcon500(1),
            Intake.LIFTER_GEAR_RATIO,
            SingleJointedArmSim.estimateMOI(0.1, 1.0),
            0.1,
            0.0,
            Constants.Intake.LIFTER_MECHANISM,
            false,
            0);

    public LifterIOSim() {
        this.lifter = new TalonFX(Intake.LIFTER_ID);

        this.lifter.getConfigurator().apply(Intake.LIFTER_CONFIG);
        this.simState = this.lifter.getSimState();
    }

    @Override
    public void updateInputs(LifterIOInputs inputs) {
        inputs.positionRads = this.sim.getAngleRads();
        inputs.velocityRPS = Units.radiansToRotations(this.sim.getVelocityRadPerSec());
        inputs.appliedVolts = this.simState.getMotorVoltage();
        inputs.supplyCurrent = this.sim.getCurrentDrawAmps();
        inputs.connected = true;

        this.sim.update(Constants.ROBOT_PERIODIC);
        this.sim.setInputVoltage(this.simState.getMotorVoltage());

        this.simState.setRawRotorPosition(
                Units.radiansToRotations(this.sim.getAngleRads())
                        * Intake.LIFTER_GEAR_RATIO);
        this.simState.setRotorVelocity(
                Units.radiansToRotations(this.sim.getVelocityRadPerSec())
                        * Intake.LIFTER_GEAR_RATIO);
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
