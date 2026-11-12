package frc.robot.subsystems.shooter.feeder;

import frc.robot.Constants;
import frc.robot.subsystems.rollers.RollerIOHardware;

public class FeederIOHardware extends RollerIOHardware {
    private static final boolean reverse = true;

    public FeederIOHardware() {
        super(Constants.Shooter.FEEDER_ID, reverse);
    }
}
