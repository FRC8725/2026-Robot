package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Joysticks.AlignMode;
import frc.robot.Robot;

public class SuperStructure extends SubsystemBase {
    private static SuperStructure SUPERSTRUCTURE;

    @AutoLogOutput(key = "SuperStructure/State")
    public final Timer stateTime = new Timer();
    public StructureInput inputs = new StructureInput();

    public SuperStructure() {
        SUPERSTRUCTURE = this;
    }

    public static SuperStructure getInstance() {
        return SUPERSTRUCTURE;
    }

    public static class StructureInput {
        public boolean wantIntake = false;
    }

    public void emptyInputs() {
        this.inputs = new StructureInput();
    }

    public ParallelCommandGroup makeZeroAllSubsystemsCommand() {
        return new ParallelCommandGroup();
    }

    @Override
    public void periodic() {
        this.stateTime.start();
        Logger.recordOutput("Robot/IsInAllianceZone",
                Robot.isInAllianceZone == null ? false : Robot.isInAllianceZone.get());
    }
}
