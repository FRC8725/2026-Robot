package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private static Intake INTAKE;
    private final TalonFX intake = new TalonFX(0);
    @AutoLogOutput(key = "Intake/state")
    private State state = State.Off;
    
    public enum State {
        Off(0.0),
        SlowIn(3.0),
        In(3.0);

        public final double voltage;

        State(double voltage) {
            this.voltage = voltage;
        }
    }

    public Intake() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        config.CurrentLimits
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(40.0);
        config.MotorOutput
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast);

        this.intake.getConfigurator().apply(config);
    }

    public static Intake getInstance() {
        return INTAKE;
    }

    public void setState(State state) {
        this.state = state;
    }

    @Override
    public void periodic() {
        this.intake.setVoltage(this.state.voltage);
    }
}
