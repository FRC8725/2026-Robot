package frc.robot.subsystems.hopper.roller;

import frc.robot.Constants.Hopper;
import frc.robot.subsystems.rollers.RollerIOHardware;

public class HopperRollerIOHardware extends RollerIOHardware {
    private static final boolean reverse = false;

    public HopperRollerIOHardware() {
        super(Hopper.ROLLER_ID, reverse);
    }    
}
