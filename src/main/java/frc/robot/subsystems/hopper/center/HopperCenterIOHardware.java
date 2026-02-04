package frc.robot.subsystems.hopper.center;

import frc.robot.Constants.Hopper;
import frc.robot.subsystems.rollers.RollerIOHardware;

public class HopperCenterIOHardware extends RollerIOHardware {
    // TODO 2/5
    private static final boolean reverse = false;

    public HopperCenterIOHardware() {
        super(Hopper.CENTER_ID, reverse);
    }    
}
