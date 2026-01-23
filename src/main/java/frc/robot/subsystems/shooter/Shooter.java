package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
    public double offset = 5.0;

    @AutoLogOutput(key = "Shooter/Flywheel State")
    private FlywheelState flywheelState = FlywheelState.Off;
    @AutoLogOutput(key = "Shooter/Hood State")    
    private HoodState hoodState = HoodState.Default;
    @AutoLogOutput(key = "Shooter/Feeder State")
    private FeederState feederState = FeederState.Off;
    private final Supplier<Boolean> wantOffsetPositive;
    private final Supplier<Boolean> wantOffsetNegative;

    public enum FlywheelState {
        Off(0.0),
        Shoot(5000.0),
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
        Push(5.0),
        SlowPush(0.0);

        public final double volts;

        FeederState(double volts) {
            this.volts = volts;
        }
    }

    public Shooter(
            FlywheelIO flywheelIO, HoodIO hoodIO, RollerIO rollerIO,
            Supplier<Boolean> wantOffsetPositive, Supplier<Boolean> wantOffsetNegative) {
        SHOOTER = this;
        this.flywheel = new Flywheel(flywheelIO);
        this.hood = new Hood(hoodIO);
        this.feeder = new Feeder(rollerIO);
        this.wantOffsetPositive = wantOffsetPositive;
        this.wantOffsetNegative = wantOffsetNegative;
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

    @AutoLogOutput(key = "Shooter/atSetpoint")
    public boolean atSetpoint() {
        return Math.abs(this.flywheel.getVelocity() - this.flywheelState.speed / 60.0) < Constants.Shooter.TOLERANCE;
    }

    @Override
    public void periodic() {
        if (this.wantOffsetPositive.get()) this.offset += 0.1;
        if (this.wantOffsetNegative.get()) this.offset -= 0.1;
        this.flywheel.periodic();
        this.hood.periodic();
        this.feeder.periodic();

        this.flywheel.setVelocity(this.flywheelState.speed);
        this.hood.setControl(new MotionMagicVoltage(Units.degreesToRotations(this.offset)));
        this.feeder.setVolts(this.feederState.volts);

        Logger.recordOutput("Shooter Measure Deg", Units.rotationsToDegrees(this.hood.getPosition()));
        Logger.recordOutput("Shooter Setpoint Deg", this.offset);
    }
}
