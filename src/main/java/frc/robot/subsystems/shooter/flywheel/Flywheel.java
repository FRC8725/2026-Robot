package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class Flywheel {
    private final FlywheelIO io;
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

    private final SimpleMotorFeedforward feedforward =
            new SimpleMotorFeedforward(0.097838, 0.11561, 0.0039729);

    public Flywheel(FlywheelIO io) {
        this.io = io;
    }

    public void periodic() {
        this.io.updateInputs(this.inputs);
        Logger.processInputs("Shooter/Flywheel", this.inputs);
    }

    public void setVolts(double volts) {
        this.io.setVolts(volts);
    }

    public void setVelocity(double rpm) {
        this.io.runVelocity(rpm, this.feedforward.calculate(rpm / 60.0));
    }

    public double getVelocity() {
        return this.inputs.velocityRPS;
    }
}
