package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIO;

public class Shooter extends SubsystemBase {
    private static Shooter SHOOTER;
    private final Flywheel flywheel;
    private final Hood hood;
    private final MotionMagicVoltage request = new MotionMagicVoltage(0.0);

    private FlywheelState flywheelState = FlywheelState.Off;
    private HoodState hoodState = HoodState.Default;

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

        public final double angle;

        HoodState(double angle) {
            this.angle = angle;
        }
    }

    public Shooter(FlywheelIO flywheelIO, HoodIO hoodIO) {
        SHOOTER = this;
        this.flywheel = new Flywheel(flywheelIO);
        this.hood = new Hood(hoodIO);
    }

    public static Shooter getInstance() {
        return SHOOTER;
    }

    public void setStates(FlywheelState flywheelState, HoodState hoodState) {
        this.flywheelState = flywheelState;
        this.hoodState = hoodState;
    }

    public void setFlywheelVolts(double volts) {
        this.flywheel.setVolts(volts);
    }

    @Override
    public void periodic() {
        this.flywheel.periodic();
        this.hood.periodic();

        this.flywheel.setVelocity(this.flywheelState.speed);
        this.hood.setControl(this.request.withPosition(this.hoodState.angle));
    }
}
