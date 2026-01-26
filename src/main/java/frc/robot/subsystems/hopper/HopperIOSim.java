package frc.robot.subsystems.hopper;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants.Hopper;
import frc.robot.subsystems.rollers.RollerIOSim;

public class HopperIOSim extends RollerIOSim {
    private static final DCMotor motor = DCMotor.getFalcon500(1);
    private static final double moi = 0.001;

    public HopperIOSim() {
        super(motor, Hopper.GEAR_RATIO, moi);
    }
}
