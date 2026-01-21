package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.MotionMagicVoltage;

public class Hood {
    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    public Hood(HoodIO io) {
        this.io = io;
    }

    public void periodic() {
        this.io.updateInputs(this.inputs);
        Logger.processInputs("Shooter/Hood", this.inputs);
    }

    public void setControl(MotionMagicVoltage request) {
        this.io.setControl(request);
    }
}
