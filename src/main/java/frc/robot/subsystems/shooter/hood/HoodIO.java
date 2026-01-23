package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.controls.MotionMagicVoltage;

public interface HoodIO {
    @AutoLog
    public class HoodIOInputs {
        public double positionRads = 0.0;
        public double velocityRPS = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrent = 0.0;
        public boolean connected = false;
    }

    void updateInputs(HoodIOInputs inputs);

    void setControl(MotionMagicVoltage requst);

    void setVolts(double volts);

    void setZeroPosition();
    
    void stop();

    double getPosition();
}
