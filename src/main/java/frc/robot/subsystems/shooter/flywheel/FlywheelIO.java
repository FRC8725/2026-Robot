package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
    @AutoLog
    public class FlywheelIOInputs {
        public double positionRads = 0.0;
        public double velocityRPS = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrent = 0.0;
        public boolean connected = false;
    }

    void updateInputs(FlywheelIOInputs inputs);

    void setVolts(double volts);

    void runVelocity(double rpm, double feedforward);

    void stop();
}
