package frc.robot.subsystems.hopper;

import frc.robot.Constants.Hopper;
import frc.robot.subsystems.rollers.RollerIOHardware;

public class HopperIOHardware extends RollerIOHardware {
    private static final boolean reverse = false;

    public HopperIOHardware() {
        super(Hopper.ID, reverse);
    }    
}
