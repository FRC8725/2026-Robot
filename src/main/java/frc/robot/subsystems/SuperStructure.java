package frc.robot.subsystems;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.shooter.Shooter;

public class SuperStructure extends SubsystemBase {
    private static SuperStructure SUPERSTRUCTURE;

    @AutoLogOutput(key = "SuperStructure/State")
    private State state = State.Start;
    private StructureInput inputs = new StructureInput();
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
            Hopper.HopperState.Off);

        private final Shooter.FlywheelState flywheelState;
        private final Shooter.HoodState hoodState;
        private final Shooter.FeederState feederState;
        private final Hopper.HopperState hopperState;

        State(
                Shooter.FlywheelState flywheelState,
                Shooter.HoodState hoodState,
                Shooter.FeederState feederState,
                Hopper.HopperState hopperState) {
            this.flywheelState = flywheelState;
            this.hoodState = hoodState;
            this.feederState = feederState;
            this.hopperState = hopperState;
        }
    }

    public static class StructureInput {
        
        
    }

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
                this.state.flywheelState, this.state.hoodState, this.state.feederState);
        Hopper.getInstance().setState(
                this.state.hopperState);
    }

    public void emptyInput() {
        this.inputs = new StructureInput();
    }

    @Override
    public void periodic() {
        this.stateTime.start();

        // for (Transition translate : this.transitions) {
        //     if (translate.currentState == state && translate.booleanSupplier.get()) {
        //         state = translate.nextState;
        //         this.stateTime.restart();
        //         translate.enterFunction.run();
        //         this.setStates();
        //         return;
        //     }
        // }
    }
}
