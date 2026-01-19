package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private static Shooter SHOOTER;
    private final TalonFX shooter = new TalonFX(13);
    private final VelocityVoltage request = new VelocityVoltage(0.0);
    @AutoLogOutput(key = "Shooter/state")
    private State state = State.Idle;

    public enum State {
        Idle(0.1),
        Shooting(90.0);

        // RPS
        public final double speed;

        State(double speed)  {
            this.speed = speed;
        }
    }

    public Shooter() {
        SHOOTER = this;
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(60.0)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(60.0);
        config.MotorOutput
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast);
        config.Slot0
                .withKP(0.0)
                .withKI(0.0)
                .withKD(0.0)
                .withKV(0.12);
        
        this.shooter.getConfigurator().apply(config);
    }

    public static Shooter getInstance() {
        return SHOOTER;
    }

    public void setState(State state) {
        this.state = state;
    }

    @AutoLogOutput(key = "Shooter/VelocityRPS")
    public double getVelocity() {
        return this.shooter.getVelocity().getValueAsDouble();
    }

    @AutoLogOutput(key = "Shooter/AtSetpoint")
    public boolean atSetpoint() {
        return Math.abs(this.getVelocity() - this.state.speed) < 5.0;
    }

    @Override
    public void periodic() {
        this.shooter.setControl(this.request.withVelocity(this.state.speed));
    }
}
