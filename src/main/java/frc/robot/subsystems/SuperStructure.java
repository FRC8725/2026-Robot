package frc.robot.subsystems;

import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Stream;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class SuperStructure extends SubsystemBase {
    private static SuperStructure SUPERSTRUCTURE;

    @AutoLogOutput(key = "SuperStructure/State")
    public State state = State.Start;
    public StructureInput input = new StructureInput();
    private final Timer stateTime = new Timer();

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
            Hopper.HopperState.Off)
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
    }

    private final List<Transition> transitions = Stream.of(
        new Transition(State.Start, State.Rest, () -> this.input.wantIntake)
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
        // System.out.println(this.state.i);
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
        this.input = new StructureInput();
    }

    public ParallelCommandGroup makeZeroAllSubsystemsCommand() {
        return new ParallelCommandGroup(
        );
    }

    @Override
    public void periodic() {
        this.stateTime.start();

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
