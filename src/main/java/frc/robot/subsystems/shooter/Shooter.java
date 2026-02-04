package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.util.ShootCaculator;
import frc.robot.subsystems.drive.Drive;
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
    private final ShootCaculator shootCaculator = new ShootCaculator();
    public double offset = 0.0;

    @AutoLogOutput(key = "Shooter/FlywheelState")
    private FlywheelState flywheelState = FlywheelState.Off;
    @AutoLogOutput(key = "Shooter/HoodState")    
    private HoodState hoodState = HoodState.Default;
    @AutoLogOutput(key = "Shooter/FeederState")
    private FeederState feederState = FeederState.Off;

    public enum FlywheelState {
        Off(0.0),
        Rest(100.0),
        Auto(5000.0),
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
            FlywheelIO flywheelIO, HoodIO hoodIO, RollerIO rollerIO) {
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
        this.hood.setControl(
                this.request.withPosition(
                        Units.degreesToRotations(this.hoodState.angle)));
        this.feeder.setVolts(this.feederState.volts);
    }

    @AutoLogOutput(key = "Shooter/HoodDesiredPosition")
    public double getDesiredPosition() {
        if (this.hoodState != HoodState.AutoAim)
            return this.hoodState.angle;

        double distance = Constants.Field.HUB_CENTER.getDistance(
                Drive.getInstance().getPose().getTranslation());

        return this.shootCaculator.getHoodAngle(distance);
    }

    @AutoLogOutput(key = "Shoooter/FlywheelDesiredVelocity")
    public double getDesiredVelocity() {
        if (this.flywheelState != FlywheelState.Auto)
            return this.flywheelState.speed;

        double distance = Constants.Field.HUB_CENTER.getDistance(
                Drive.getInstance().getPose().getTranslation());
        
        return this.shootCaculator.getFlywheelVelocity(distance);
    }

    @AutoLogOutput(key = "Shooter/atSetpoint")
    public boolean atSetpoint() {
        return Math.abs(this.flywheel.getVelocity() - this.flywheelState.speed / 60.0)
                < Constants.Shooter.TOLERANCE;
    }

    @AutoLogOutput(key = "Component/Shooter")
    public Pose3d getSimulationPose() {
        return new Pose3d(
                -Units.inchesToMeters(10.0), 0.0, Units.inchesToMeters(17.5),
                new Rotation3d(0.0, this.hood.getPosition(), 0.0));
    }
}
