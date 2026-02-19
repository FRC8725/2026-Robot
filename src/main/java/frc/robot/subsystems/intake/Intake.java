package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.math.MathHelpers;
import frc.robot.subsystems.SuperStructure;
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

    private final double distance = 0.1;

    public enum LifterState {
        Up(0.05),
        // Up(0.262),
        Down(0.29),
        Zero(0.03),
        Slide(0.29),
        OperateControl(0.0);

        // Units: rotation
        public final double angle;

        LifterState(double angle) {
            this.angle = angle;
        }
    }

    public enum RollerState {
        Off(0.0),
        Rest(1.0),
        SlowIn(5.0),
        In(7.0),
        OperateControl(0.0);

        public final double volts;

        RollerState(double volts) {
            this.volts = volts;
        }
    }

    public Intake(LifterIO lifterIO, RollerIO rollerIO) {
        INTAKE = this;
        this.lifter = new LifterSubsystem(lifterIO);
        this.roller = new RollerIOSystem(rollerIO, "Intake/Roller");
        this.setZeroPosition();
    }

    public static Intake getInstance() {
        return INTAKE;
    }

    public void setZeroPosition() {
        // this.lifter.setZeroPosition();
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
                this.request.withPosition(this.getEffectiveLifterLength()));
        this.roller.setVolts(this.getEffectiveRollerState().volts);
    }

    @AutoLogOutput(key = "Intake/LifterState")
    public double getEffectiveLifterLength() {
        if (MathHelpers.inAutoTimer(0.5))
            return LifterState.Zero.angle;
        if (this.lifterState == LifterState.Slide)
            return this.lifterState.angle -
                    Math.abs(Math.sin(4.0 * SuperStructure.getInstance().stateTime.get()) * distance);
        else if (this.lifterState != LifterState.OperateControl)
            return this.lifterState.angle;
        else if (SuperStructure.getInstance().inputs.wantIntake)
            return LifterState.Down.angle;
        else 
            return LifterState.Down.angle;
    }

    @AutoLogOutput(key = "Intake/RollerState")
    public RollerState getEffectiveRollerState() {
        if (this.rollerState != RollerState.OperateControl)
            return this.rollerState;
        else if (SuperStructure.getInstance().inputs.wantIntake)
            return RollerState.In;
        else
            return RollerState.Off;
    }

    @AutoLogOutput(key = "Intake/atSetpoint")
    public boolean atSetpoint() {
        return Math.abs(
                this.lifter.getPosition() - this.getEffectiveLifterLength())
                        < Constants.Intake.LIFTER_ANGLE_TOLERANCE;
    }

    // @AutoLogOutput(key = "Intake/isUnsafe")
    // public boolean isUnsafe() {
    //     double robotSide = Drive.getInstance().getPose().getY();
    //     double distance = Math.min(Constants.Field.FIELD_Y_SIZE - robotSide, robotSide);

    //     return distance < Constants.Intake.LIFTER_LIMIT_DISTANCE;
    // }

    @AutoLogOutput(key = "Component/IntakeLifter")
    public Pose3d getSimulationPose() {
        double length = this.lifter.getPosition();

        return new Pose3d(
                length * Math.cos(Units.degreesToRadians(17.0)),
                0.0,
                -length * Math.sin(Units.degreesToRadians(17.0)),
                Rotation3d.kZero);
    }
}
