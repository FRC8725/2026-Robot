package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.Logger;

public class RollerSystem {
    private final RollerIO io;
    private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();
    private final String name;

    public RollerSystem(RollerIO io, String name) {
        this.io = io;
        this.name = name;
    }

    public void periodic() {
        this.io.updateInputs(this.inputs);
        Logger.processInputs(this.name, this.inputs);
    }

    public void setVolts(double volts) {
        this.io.setVolts(volts);
    }
}
