package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.MotionMagicVoltage;

public class LifterSubsystem {
    private final LifterIO io;
    private final LifterIOInputsAutoLogged inputs = new LifterIOInputsAutoLogged();

    public LifterSubsystem(LifterIO io) {
        this.io = io;
    }

    public void periodic() {
        this.io.updateInputs(this.inputs);
        Logger.processInputs("Intake/Lifter", this.inputs);
    }

    public void setControl(MotionMagicVoltage request) {
        this.io.setControl(request);
    }

    public void setZeroPosition() {
        this.io.setZeroPosition();
    }

    public double getPosition() {
        return this.inputs.positionRads;
    }
}
