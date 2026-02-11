package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.State;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class Simulation extends SubsystemBase {
    private final RobotContainer container;
    private final IntakeSimulation intakeSimulation;
    private double lastShotTime = 0.0;
    private final double SHOOT_DELAY = 0.1;

    public Simulation(RobotContainer container) {
        this.container = container;
        this.intakeSimulation = IntakeSimulation.OverTheBumperIntake(
                "Fuel",
                container.getDriveSubsystem().getMapleSimDrivetrain().mapleSimDrive,
                Meters.of(0.63062),
                Meters.of(0.258555),
                IntakeSimulation.IntakeSide.FRONT,
            30);
        
        this.intakeSimulation.addGamePiecesToIntake(8);
    }

    public void handleIntakeSimulation() {
        if (Intake.getInstance().getEffectiveLifterState() == Intake.LifterState.Down
                && Intake.getInstance().getEffectiveRollerState() == Intake.RollerState.In)
            this.intakeSimulation.startIntake();
        else
            this.intakeSimulation.stopIntake();
    }

    public void proccessShooter() {
        double currentTime = Timer.getFPGATimestamp();

        if (this.intakeSimulation.getGamePiecesAmount() != 0
                && SuperStructure.getInstance().state == State.Shoot
                && (currentTime - this.lastShotTime > SHOOT_DELAY)) {
            this.lastShotTime = currentTime;
            this.intakeSimulation.obtainGamePieceFromIntake();
            
            SimulatedArena.getInstance().addGamePieceProjectile(
                    new RebuiltFuelOnFly(
                            this.container.getDriveSubsystem().getPose().getTranslation(),
                            new Translation2d(-0.18415, 0.0),
                            this.container.getDriveSubsystem().getRobotChassisSpeeds(),
                            this.container.getDriveSubsystem().getPose().getRotation(),
                            Meters.of(0.4446524),
                            MetersPerSecond.of(6.5),
                            Degrees.of(20.0 + Units.radiansToDegrees(Shooter.getInstance().getPosition()) + 90.0)));
        }
    }

    @Override
    public void periodic() {
        this.handleIntakeSimulation();
        this.proccessShooter();
    }
}
