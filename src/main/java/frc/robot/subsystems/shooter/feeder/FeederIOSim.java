package frc.robot.subsystems.shooter.feeder;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.rollers.RollerIOSim;

public class FeederIOSim extends RollerIOSim {
    private static final DCMotor motor = DCMotor.getFalcon500(1);
    private static final double reducion = 1.0;
    private static final double moi = 0.001;

    public FeederIOSim() {
        super(motor, reducion, moi);
    }
}
