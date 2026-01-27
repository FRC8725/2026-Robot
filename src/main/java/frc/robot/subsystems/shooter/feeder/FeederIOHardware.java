package frc.robot.subsystems.shooter.feeder;

import frc.robot.subsystems.rollers.RollerIOHardware;

public class FeederIOHardware extends RollerIOHardware {
    private static final int id = 0;
    private static final boolean reverse = false;

    public FeederIOHardware() {
        super(id, reverse);
    }
}
