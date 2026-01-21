package frc.robot.subsystems;

import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Stream;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
                Shooter.FeederState.Off),
        PreShoot(
                Shooter.FlywheelState.Shoot,
                Shooter.HoodState.Default,
                Shooter.FeederState.Off),
        Shoot(
                Shooter.FlywheelState.Shoot,
                Shooter.HoodState.Default,
                Shooter.FeederState.Push),
        ;

        public final Shooter.FlywheelState flywheelState;
        public final Shooter.HoodState hoodState;
        public final Shooter.FeederState feederState;

        State(
                Shooter.FlywheelState flywheelState,
                Shooter.HoodState hoodState,
                Shooter.FeederState feederState) {
            this.flywheelState = flywheelState;
            this.hoodState = hoodState;
            this.feederState = feederState;
        }
    }

    public static class StructureInput {
        public boolean wantIntake = false;
        public boolean wantScore = false;
    }

    private final List<Transition> transitions = Stream.of(

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
