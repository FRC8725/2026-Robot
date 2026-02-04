package frc.robot.subsystems.intake.roller;

import frc.robot.Constants.Intake;
import frc.robot.subsystems.rollers.RollerIOHardware;

public class IntakeRollerHardware extends RollerIOHardware {
    // TODO 2/5
    private static final boolean reverse = false;

    public IntakeRollerHardware() {
        super(Intake.ROLLER_ID, reverse);
    }
    
}
