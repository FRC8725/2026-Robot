package frc.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Constants.Shooter;

public class HoodIOSim implements HoodIO {
    private final TalonFX hood;
    private final TalonFXSimState simState;
    private final SingleJointedArmSim sim = new SingleJointedArmSim(
            DCMotor.getKrakenX44(1),
            Constants.Shooter.HOOD_GEAR_RATIO,
            SingleJointedArmSim.estimateMOI(0.1, 0.5),
            0.1,
            0.0,
            0.0,
            true,
            0.0);

    public HoodIOSim() {
        this.hood = new TalonFX(Shooter.HOOD_ID);

        this.hood.getConfigurator().apply(Constants.Shooter.HOOD_CONFIG);
        this.simState = this.hood.getSimState();
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.positionRads = this.sim.getAngleRads();
        inputs.velocityRPS = Units.radiansToRotations(this.sim.getVelocityRadPerSec());
        inputs.appliedVolts = this.simState.getMotorVoltage();
        inputs.supplyCurrent = this.sim.getCurrentDrawAmps();
        inputs.connected = true;

        this.sim.update(0.020);
        this.sim.setInputVoltage(this.simState.getMotorVoltage());

        this.simState.setRawRotorPosition(
                Units.radiansToRotations(this.sim.getAngleRads())
                        * Constants.Shooter.HOOD_GEAR_RATIO);
        this.simState.setRotorVelocity(
                Units.radiansToRotations(this.sim.getVelocityRadPerSec())
                        * Constants.Shooter.HOOD_GEAR_RATIO);
    }

    @Override
    public void setControl(MotionMagicVoltage requst) {
        this.hood.setControl(requst);
    }

    @Override
    public void setVolts(double volts) {
        this.hood.setVoltage(volts);
    }

    @Override
    public void setZeroPosition() {
        this.hood.setPosition(0.0);
    }

    @Override
    public void stop() {
        this.hood.stopMotor();
    }

    @Override
    public double getPosition() {
        return this.hood.getPosition().getValueAsDouble();
    }
    
}
