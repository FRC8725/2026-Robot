package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.rollers.RollerIO;
import frc.robot.subsystems.rollers.RollerIOSystem;

public class Hopper extends SubsystemBase {
    private static Hopper HOPPER;
    private final RollerIOSystem roller;
    private final RollerIOSystem center;

    @AutoLogOutput(key = "Hopper/State")
    private HopperState hopperState = HopperState.Off;

    public Hopper(RollerIO rollerIO, RollerIO centerIO) {
        HOPPER = this;
        this.roller = new RollerIOSystem(rollerIO, "Hopper/Roller");
        this.center = new RollerIOSystem(centerIO, "Hopper/Center");
    }

    public static Hopper getInstance() {
        return HOPPER;
    }
 
    public enum HopperState {
        Off(0.0, 0.0),
        Convey(6.0, 6.5);

        public final double rollerVolts;
        public final double centerVolts;

        HopperState(double rollerVolts, double centerVolts) {
            this.rollerVolts = rollerVolts;
            this.centerVolts = centerVolts;
        }
    }

    public void setState(HopperState hopperState) {
        this.hopperState = hopperState;
    }

    @Override
    public void periodic() {
        this.roller.periodic();
        this.center.periodic();
        
        this.roller.setVolts(this.hopperState.rollerVolts);
        this.center.setVolts(this.hopperState.centerVolts);
    }
}
