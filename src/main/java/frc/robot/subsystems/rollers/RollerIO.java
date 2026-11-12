package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
    @AutoLog
    public class RollerIOInputs {
        double positionRads = 0.0;
        double velocityRPS = 0.0;
        double appliedVolts = 0.0;
        double supplyCurrent = 0.0;
        boolean connected = false;
    }

    void updateInputs(RollerIOInputs inputs);

    void setVolts(double volts);

    void stop();
}
