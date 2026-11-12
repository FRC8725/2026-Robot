package frc.robot.subsystems.shooter.feeder;

import frc.robot.subsystems.rollers.RollerIO;
import frc.robot.subsystems.rollers.RollerIOSystem;

public class Feeder {
    private final RollerIOSystem roller;

    public Feeder(RollerIO io) {
        this.roller = new RollerIOSystem(io, "Shooter/Feeder");
    }

    public void periodic() {
        this.roller.periodic();
    }

    public void setVolts(double volts) {
        this.roller.setVolts(volts);
    }

    public void stop() {
        this.roller.stop();
    }
}
