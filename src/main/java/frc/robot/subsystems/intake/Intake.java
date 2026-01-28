package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

    public enum LifterState {
        Up(0.0),
        Down(3.0),
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
        SlowIn(0.0),
        In(3.0),
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
                this.request.withPosition(this.getEffectiveLifterState().angle));
        this.roller.setVolts(this.getEffectiveRollerState().volts);
    }

    @AutoLogOutput(key = "Intake/LifterState")
    public LifterState getEffectiveLifterState() {
        if (this.lifterState != LifterState.OperateControl)
            return this.lifterState;
        else if (SuperStructure.getInstance().input.wantIntake)
            return LifterState.Down;
        else 
            return LifterState.Up;
    }

    @AutoLogOutput(key = "Intake/RollerState")
    public RollerState getEffectiveRollerState() {
        if (this.rollerState != RollerState.OperateControl)
            return this.rollerState;
        else if (SuperStructure.getInstance().input.wantIntake)
            return RollerState.In;
        else
            return RollerState.Off;
    }

    @AutoLogOutput(key = "Intake/atSetpoint")
    public boolean atSetpoint() {
        return Math.abs(
                this.lifter.getPosition() - Units.rotationsToRadians(this.getEffectiveLifterState().angle))
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
        double length = Units.radiansToRotations(this.lifter.getPosition())
                * Constants.Intake.LIFTER_GEAR_DIAMETER * Math.PI;
        return new Pose3d(
                length * Math.cos(Units.degreesToRadians(11.175)),
                0.0,
                -length * Math.sin(Units.degreesToRadians(11.175)),
                Rotation3d.kZero);
    }
}
