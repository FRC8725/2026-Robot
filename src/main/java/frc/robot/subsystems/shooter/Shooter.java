package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.rollers.RollerIO;
import frc.robot.subsystems.shooter.feeder.Feeder;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIO;

public class Shooter extends SubsystemBase {
    private static Shooter SHOOTER;
    private final Flywheel flywheel;
    private final Hood hood;
    private final Feeder feeder;
    private final MotionMagicVoltage request = new MotionMagicVoltage(0.0);
    public double offset = 0.0;

    private FlywheelState flywheelState = FlywheelState.Off;
    private HoodState hoodState = HoodState.Default;
    private FeederState feederState = FeederState.Off;

    public enum FlywheelState {
        Off(0.0),
        Shoot(0.0),
        SlowShoot(0.0);

        // RPM
        public final double speed;

        FlywheelState(double speed) {
            this.speed = speed;
        }
    }

    public enum HoodState {
        Default(0.0),
        AutoAim(0.0),
        Return(0.0);

        // Degree
        public final double angle;

        HoodState(double angle) {
            this.angle = angle;
        }
    }

    public enum FeederState {
        Off(0.0),
        Push(0.0),
        SlowPush(0.0);

        public final double volts;

        FeederState(double volts) {
            this.volts = volts;
        }
    }

    public Shooter(FlywheelIO flywheelIO, HoodIO hoodIO, RollerIO rollerIO) {
        SHOOTER = this;
        this.flywheel = new Flywheel(flywheelIO);
        this.hood = new Hood(hoodIO);
        this.feeder = new Feeder(rollerIO);
    }

    public static Shooter getInstance() {
        return SHOOTER;
    }

    public void setStates(
            FlywheelState flywheelState, HoodState hoodState, FeederState feederState) {
        this.flywheelState = flywheelState;
        this.hoodState = hoodState;
        this.feederState = feederState;
    }

    public void setFlywheelVolts(double volts) {
        this.flywheel.setVolts(volts);
    }

    @Override
    public void periodic() {
        this.flywheel.periodic();
        this.hood.periodic();
        this.feeder.periodic();

        this.flywheel.setVelocity(this.flywheelState.speed);
        this.hood.setControl(this.request.withPosition(this.hoodState.angle + this.offset));
        this.feeder.setVolts(this.feederState.volts);
    }
}
