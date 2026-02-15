package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.math.MathHelpers;
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
    @AutoLogOutput(key = "Shooter/Test_Hood_Position")
    public double hoodPosition = 0.0;
    @AutoLogOutput(key = "Shooter/Test_Flywheel_Velocity")
    public double velocityOffset = 0.0;
    private final Supplier<Boolean> up;
    private final Supplier<Boolean> down;
    private final Supplier<Boolean> fup;
    private final Supplier<Boolean> fdown;

    @AutoLogOutput(key = "Shooter/FlywheelState")
    private FlywheelState flywheelState = FlywheelState.Off;
    @AutoLogOutput(key = "Shooter/HoodState")    
    private HoodState hoodState = HoodState.Default;
    @AutoLogOutput(key = "Shooter/FeederState")
    private FeederState feederState = FeederState.Off;

    public enum FlywheelState {
        Off(0.0),
        Rest(100.0),
        Auto(3150.0),
        Home(4000.0),
        SlowShoot(2000.0);

        // RPM
        public final double speed;

        FlywheelState(double speed) {
            this.speed = speed;
        }
    }

    public enum HoodState {
        Default(0.0),
        AutoAim(0.0),
        Home(25.0),
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
            Supplier<Boolean> up, Supplier<Boolean> down,
            Supplier<Boolean> fup, Supplier<Boolean> fdown) {
        SHOOTER = this;
        this.flywheel = new Flywheel(flywheelIO);
        this.hood = new Hood(hoodIO);
        this.feeder = new Feeder(rollerIO);
        this.up = up;
        this.down = down;
        this.fup = fup;
        this.fdown = fdown;
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

    public void setHoodState(HoodState hoodState) {
        this.hoodState = hoodState;
    }
    
    public void setFlywheelVolts(double volts) {
        this.flywheel.setVolts(volts);
    }

    @Override
    public void periodic() {
        this.flywheel.periodic();
        this.hood.periodic();
        this.feeder.periodic();

        if (up.get()) hoodPosition += 0.1;
        if (down.get()) hoodPosition -= 0.1;

        if (fup.get()) velocityOffset += 50;
        if (fdown.get()) velocityOffset -= 50;

        Logger.recordOutput("HUB_DISTANCE", MathHelpers.mirrorIfRed(Constants.Field.HUB_CENTER).getDistance(Drive.getInstance().getPose().getTranslation()));

        this.flywheel.setVelocity(this.getDesiredVelocity());
        this.hood.setControl(
                this.request.withPosition(
                        Units.degreesToRotations(this.getDesiredPosition())));
        this.feeder.setVolts(this.feederState.volts);
    }

    public double getPosition() {
        return this.hood.getPosition();
    }

    /**
     * 
     * @return Units: Degrees
     * 
     */
    @AutoLogOutput(key = "Shooter/HoodDesiredPosition")
    public double getDesiredPosition() {
        if (this.atTrenchZone())
            return 0.0;
        
        if (this.hoodState != HoodState.AutoAim)
            return this.hoodState.angle;

        // return hoodPosition;
        double distance = MathHelpers.mirrorIfRed(Constants.Field.HUB_CENTER).getDistance(
                Drive.getInstance().getPose().getTranslation());

        return this.shootCaculator.getHoodAngle(distance);
    }

    @AutoLogOutput(key = "Shooter/FlywheelDesiredVelocity") // Rotate per minute
    public double getDesiredVelocity() {
        if (this.flywheelState != FlywheelState.Auto)
            return this.flywheelState.speed;

        // return this.velocityOffset;
        double distance = MathHelpers.mirrorIfRed(Constants.Field.HUB_CENTER).getDistance(
                Drive.getInstance().getPose().getTranslation());
        
        return this.shootCaculator.getFlywheelVelocity(distance);
    }

    @AutoLogOutput(key = "Shooter/FlywheelAtSetpoint")
    public boolean flywheelAtSetpoint() {
        return Math.abs(this.flywheel.getVelocity() - this.getDesiredVelocity() / 60.0)
                < Constants.Shooter.FLYWHEEL_TOLERANCE;
    }

    @AutoLogOutput(key = "Shooter/HoodAtSetpoint")
    public boolean hoodAtSetpoint() {
        return Math.abs(Units.radiansToDegrees(this.hood.getPosition()) - this.getDesiredPosition())
                < Constants.Shooter.HOOD_TOLERANCE;
    }

    @AutoLogOutput(key = "Shooter/atTrenchZone")
    public boolean atTrenchZone() {
        Pose2d robot = Drive.getInstance().getPose();
        return MathHelpers.atInterval(robot, Constants.Field.TRENCH_X, 1.0)
                && (robot.getY() < 0.847 || robot.getY() > 6.353312);
    }

    @AutoLogOutput(key = "Component/Shooter")
    public Pose3d getSimulationPose() {
        return new Pose3d(
                -Units.inchesToMeters(10.0), 0.0, Units.inchesToMeters(17.5),
                new Rotation3d(0.0, this.hood.getPosition(), 0.0));
    }
}
