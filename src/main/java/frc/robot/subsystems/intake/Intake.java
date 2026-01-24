package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.lifter.LifterIO;
import frc.robot.subsystems.intake.lifter.LifterSubsystem;
import frc.robot.subsystems.rollers.RollerIO;
import frc.robot.subsystems.rollers.RollerIOSystem;

public class Intake extends SubsystemBase {
    private static Intake INTAKE;
    private final LifterSubsystem lifter;
    private final RollerIOSystem roller;
    private final MotionMagicVoltage request = new MotionMagicVoltage(0.0);
    private boolean isZeroed = false;

    private LifterState lifterState = LifterState.Up;
    private RollerState rollerState = RollerState.Off;

    public enum LifterState {
        Up(0.0),
        Down(0.0);

        // Units: degree
        public final double angle;

        LifterState(double angle) {
            this.angle = angle;
        }
    }

    public enum RollerState {
        Off(0.0),
        SlowIn(0.0),
        In(0.0);

        public final double volts;

        RollerState(double volts) {
            this.volts = volts;
        }
    }

    public Intake(LifterIO lifterIO, RollerIO rollerIO) {
        INTAKE = this;
        this.lifter = new LifterSubsystem(lifterIO);
        this.roller = new RollerIOSystem(rollerIO, "Intake/Roller");
    }

    public static Intake getInstance() {
        return INTAKE;
    }

    public void setZeroPosition() {
        this.lifter.setZeroPosition();
        this.isZeroed = true;
    }

    public void setStates(LifterState lifterState, RollerState rollerState) {
        this.lifterState = lifterState;
        this.rollerState = rollerState;
    }

    @Override
    public void periodic() {
        if (!this.isZeroed) return;

        this.lifter.periodic();
        this.roller.periodic();

        this.lifter.setControl(
                this.request
                        .withPosition(Units.degreesToRotations(this.lifterState.angle)));
        this.roller.setVolts(this.rollerState.volts);
    }

    @AutoLogOutput(key = "Intake/atSetpoint")
    public boolean atSetpoint() {
        return Math.abs(
                this.lifter.getPosition() - Units.degreesToRotations(this.lifterState.angle))
                        < Constants.Intake.LIFTER_ANGLE_TOLERANCE;
    }

    @AutoLogOutput(key = "Intake/isUnsafe")
    public boolean isUnsafe() {
        double robotSide = Drive.getInstance().getPose().getY();
        double distance = Math.min(Constants.Field.FIELD_Y_SIZE - robotSide, robotSide);

        return distance < Constants.Intake.LIFTER_LIMIT_DISTANCE;
    }
}
