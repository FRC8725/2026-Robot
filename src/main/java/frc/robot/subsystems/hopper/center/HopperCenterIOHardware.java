package frc.robot.subsystems.hopper.center;

import frc.robot.Constants.Hopper;
import frc.robot.subsystems.rollers.RollerIOHardware;

public class HopperCenterIOHardware extends RollerIOHardware {
    private static final boolean reverse = true;

    public HopperCenterIOHardware() {
        super(Hopper.CENTER_ID, reverse);
    }    
}
