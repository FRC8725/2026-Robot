package frc.robot.subsystems;

import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Stream;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.Intake;

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
            Intake.LifterState.Up,
            Intake.RollerState.Off),
        Rest(
            Intake.LifterState.Down,
            Intake.RollerState.In);

        public final Intake.LifterState intakeLifter;
        public final Intake.RollerState intakeRoller;

        State(Intake.LifterState intakeLifter, Intake.RollerState intakeRoller) {
            this.intakeLifter = intakeLifter;
            this.intakeRoller = intakeRoller;
        }
    }

    public static class StructureInput {
        public boolean wantIntake = false;
        public boolean wantScore = false;
    }

    private final List<Transition> transitions = Stream.of(
        new Transition(State.Start, State.Rest, () -> this.input.wantIntake),
        new Transition(State.Rest, State.Start, () -> !this.input.wantIntake)

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
        Intake.getInstance().setStates(this.state.intakeLifter, this.state.intakeRoller);
    }

    public void emptyInputs() {
        this.input = new StructureInput();
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
