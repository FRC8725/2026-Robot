package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.rollers.RollerIO;
import frc.robot.subsystems.rollers.RollerIOSystem;

public class Hopper extends SubsystemBase {
    private static Hopper HOPPER;
    private final RollerIOSystem roller;

    @AutoLogOutput(key = "Hopper/State")
    private HopperState hopperState = HopperState.Off;

    public Hopper(RollerIO rollerIO) {
        HOPPER = this;
        this.roller = new RollerIOSystem(rollerIO, "Hopper");
    }

    public static Hopper getInstance() {
        return HOPPER;
    }
 
    public enum HopperState {
        Off(0.0),
        Convey(3.0);

        public final double volts;

        HopperState(double volts) {
            this.volts = volts;
        }
    }

    public void setState(HopperState hopperState) {
        this.hopperState = hopperState;
    }

    @Override
    public void periodic() {
        this.roller.periodic();
        
        double volts = Math.abs(Math.sin(Timer.getFPGATimestamp() * 10.0 * Math.PI)) * 3.0;
        this.roller.setVolts(this.hopperState.volts);
    }
}
