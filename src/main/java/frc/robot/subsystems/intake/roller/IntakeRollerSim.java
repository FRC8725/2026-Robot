package frc.robot.subsystems.intake.roller;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants.Intake;
import frc.robot.subsystems.rollers.RollerIOSim;

public class IntakeRollerSim extends RollerIOSim {
    private static final DCMotor motor = DCMotor.getFalcon500(1);
    private static final double moi = 0.001;

    public IntakeRollerSim() {
        super(motor, Intake.ROLLER_GEAR_RATIO, moi);
    }
    
}
