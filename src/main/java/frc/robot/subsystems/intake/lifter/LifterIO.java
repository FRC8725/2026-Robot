package frc.robot.subsystems.intake.lifter;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.controls.MotionMagicVoltage;

public interface LifterIO {
    @AutoLog
    public class LifterIOInputs {
        public double positionLength = 0.0;
        public double velocityRPS = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrent = 0.0;
        public boolean connected = false;
    }

    void updateInputs(LifterIOInputs inputs);

    void setControl(MotionMagicVoltage request);

    void setZeroPosition();

    void stop();
}
