package frc.robot.subsystems;

import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Stream;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Joysticks.AlignMode;
import frc.robot.commands.AutoRunnerCmd;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class SuperStructure extends SubsystemBase {
    private static SuperStructure SUPERSTRUCTURE;

    @AutoLogOutput(key = "SuperStructure/State")
    public State state = State.Start;
    public StructureInput inputs = new StructureInput();
    public final Timer stateTime = new Timer();

    public SuperStructure() {
        SUPERSTRUCTURE = this;
    }

    public static SuperStructure getInstance() {
        return SUPERSTRUCTURE;
    }

    public enum State {
        Start(
            Shooter.FlywheelState.Off,
            Shooter.HoodState.Default,
            Shooter.FeederState.Off,
            Intake.LifterState.Up,
            Intake.RollerState.Off,
            Hopper.HopperState.Off),
        Rest(
            Shooter.FlywheelState.Rest,
            Shooter.HoodState.Default,
            Shooter.FeederState.Off,
            Hopper.HopperState.Off),
        PreShootHome(
            Shooter.FlywheelState.Home,
            Shooter.HoodState.Home,
            Shooter.FeederState.Off,
            Intake.LifterState.Slide,
            Intake.RollerState.SlowIn,
            Hopper.HopperState.Off),
        ShootHome(
            Shooter.FlywheelState.Home,
            Shooter.HoodState.Home,
            Shooter.FeederState.Push,
            Intake.LifterState.Slide,
            Intake.RollerState.SlowIn,
            Hopper.HopperState.Convey),
        PreShoot(
            Shooter.FlywheelState.Auto,
            Shooter.HoodState.AutoAim,
            Shooter.FeederState.Off,
            Intake.LifterState.Slide,
            Intake.RollerState.SlowIn,
            Hopper.HopperState.Off),
        Shoot(
            Shooter.FlywheelState.Auto,
            Shooter.HoodState.AutoAim,
            Shooter.FeederState.Push,
            Intake.LifterState.Slide,
            Intake.RollerState.SlowIn,
            Hopper.HopperState.Convey),
        
        // Autonoumous
        ZeroIntakeAuto(
            Shooter.FlywheelState.Rest,
            Shooter.HoodState.Default,
            Shooter.FeederState.Off,
            Intake.LifterState.Zero,
            Intake.RollerState.Off,
            Hopper.HopperState.Off),
        SlideIntake(
            Shooter.FlywheelState.Rest,
            Shooter.HoodState.Default,
            Shooter.FeederState.Off,
            Intake.LifterState.Slide,
            Intake.RollerState.SlowIn,
            Hopper.HopperState.Off),
        ;

        public final Shooter.FlywheelState flywheelState;
        public final Shooter.HoodState hoodState;
        public final Shooter.FeederState feederState;
        public final Intake.LifterState lifterState;
        public final Intake.RollerState rollerState;
        public final Hopper.HopperState hopperState;

        State(
                Shooter.FlywheelState flywheelState,
                Shooter.HoodState hoodState,
                Shooter.FeederState feederState,
                Intake.LifterState lifterState,
                Intake.RollerState rollerState,
                Hopper.HopperState hopperState) {
            this.flywheelState = flywheelState;
            this.hoodState = hoodState;
            this.feederState = feederState;
            this.lifterState = lifterState;
            this.rollerState = rollerState;
            this.hopperState = hopperState;
        }

        State(
                Shooter.FlywheelState flywheelState,
                Shooter.HoodState hoodState,
                Shooter.FeederState feederState,
                Hopper.HopperState hopperState) {
            this.flywheelState = flywheelState;
            this.hoodState = hoodState;
            this.feederState = feederState;
            this.lifterState = Intake.LifterState.OperateControl;
            this.rollerState = Intake.RollerState.OperateControl;
            this.hopperState = hopperState;
        }
    }

    public static class StructureInput {
        public boolean wantIntake = false;
        public boolean wantScore = false;
        public boolean wantTrack = false;
        public boolean wantShootHome = false;
        public boolean zeroIntake = false;
        public boolean slideIntake = false;
        public AlignMode alignMode = AlignMode.None;
    }

    private final List<Transition> transitions = Stream.of(
        new Transition(State.Start, State.Rest, () -> this.inputs.wantIntake),
        new Transition(State.Start, State.Rest, () -> RobotState.isAutonomous()),

        new Transition(State.Rest, State.PreShoot, () -> this.inputs.wantScore && this.inputs.alignMode != AlignMode.None && DriveCommand.isAlignFinished),
        new Transition(State.PreShoot, State.Rest, () -> !this.inputs.wantScore),
        new Transition(State.PreShoot, State.Shoot, () -> Shooter.getInstance().flywheelAtSetpoint() && Shooter.getInstance().hoodAtSetpoint()),
        new Transition(State.Shoot, State.Rest, () -> !this.inputs.wantScore),

        // Autonoumous
        new Transition(State.Rest, State.PreShoot, () -> RobotState.isAutonomous() && this.inputs.wantScore && AutoRunnerCmd.isAlignFinished),
        new Transition(State.Rest, State.ZeroIntakeAuto, () -> this.inputs.zeroIntake),
        new Transition(State.ZeroIntakeAuto, State.Rest, () -> !this.inputs.zeroIntake),
        new Transition(State.Rest, State.SlideIntake, () -> this.inputs.slideIntake),
        new Transition(State.SlideIntake, State.Rest, () -> !this.inputs.slideIntake),
        // new Transition(State.Shoot, State.ZeroIntakeAuto, () -> this.inputs.zeroIntake),

        new Transition(State.Rest, State.PreShootHome, () -> this.inputs.wantShootHome && !Robot.isInAllianceZone.get()),
        new Transition(State.PreShootHome, State.Rest, () -> !this.inputs.wantShootHome || Robot.isInAllianceZone.get()),
        new Transition(State.PreShootHome, State.ShootHome, () -> Shooter.getInstance().flywheelAtSetpoint() && Shooter.getInstance().hoodAtSetpoint()),
        new Transition(State.ShootHome, State.Rest, () -> !this.inputs.wantShootHome || Robot.isInAllianceZone.get())
    ).toList();

    public class Transition {
        public State currentState;
        public State nextState;
        public Runnable enterFunction;
        public Supplier<Boolean> booleanSupplier;

        public Transition(State cur, State next, Runnable enterFunction, Supplier<Boolean> booleanSupplier) {
            this.currentState = cur;
            this.nextState = next;
            this.enterFunction = enterFunction != null ? enterFunction : () -> {};
            this.booleanSupplier = booleanSupplier;
        }

        public Transition(State cur, State next, Supplier<Boolean> booleanSupplier) {
            this(cur, next, () -> {}, booleanSupplier);
        }
    }

    public void setStates() {
        Shooter.getInstance().setStates(
                this.state.flywheelState,
                this.state.hoodState,
                this.state.feederState);
        Intake.getInstance().setStates(
                this.state.lifterState,
                this.state.rollerState);
        Hopper.getInstance().setState(
                this.state.hopperState);
    }

    public void emptyInputs() {
        this.inputs = new StructureInput();
    }

    public ParallelCommandGroup makeZeroAllSubsystemsCommand() {
        return new ParallelCommandGroup(
        );
    }

    @Override
    public void periodic() {
        this.stateTime.start();
        Logger.recordOutput("Robot/IsInAllianceZone", Robot.isInAllianceZone == null ? false : Robot.isInAllianceZone.get());

        for (Transition transition : this.transitions) {
            if (this.state == transition.currentState && transition.booleanSupplier.get()) {
                this.state = transition.nextState;
                this.stateTime.reset();
                transition.enterFunction.run();
                this.setStates();
                return;
            }
        }
    }
}
