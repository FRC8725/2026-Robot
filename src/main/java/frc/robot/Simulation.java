package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.State;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class Simulation extends SubsystemBase {
    private final RobotContainer container;
    private final IntakeSimulation leftIntake;
    private double lastShotTime = 0.0;
    private boolean isRightShooterNext = false;

    public Simulation(RobotContainer container) {
        this.container = container;
        this.leftIntake = IntakeSimulation.OverTheBumperIntake(
                "Fuel",
                container.getDriveSubsystem().getMapleSimDrivetrain().mapleSimDrive,
                Meters.of(0.63062),
                Meters.of(0.258555),
                IntakeSimulation.IntakeSide.FRONT,
            30);
        
        this.leftIntake.addGamePiecesToIntake(8);
    }

    public void handleIntakeSimulation() {
        if (Intake.getInstance().getEffectiveLifterState() == Intake.LifterState.Down
                && Intake.getInstance().getEffectiveRollerState() == Intake.RollerState.In)
            this.leftIntake.startIntake();
        else
            this.leftIntake.stopIntake();
    }

    public void proccessShooter() {
        double currentTime = Timer.getFPGATimestamp();
        double requiredDelay = this.isRightShooterNext ? 0.05 : 0.1;

        if (this.leftIntake.getGamePiecesAmount() != 0
                && (SuperStructure.getInstance().state == State.Shoot 
                        || SuperStructure.getInstance().state == State.ShootHome)
                && (currentTime - this.lastShotTime > requiredDelay)) {
            this.lastShotTime = currentTime;
            this.leftIntake.obtainGamePieceFromIntake();
            
            double flywheel = Shooter.getInstance().getDesiredVelocity();
            double velocity = (1.5 * flywheel - 450.0) / 600.0;

            double yOffset = this.isRightShooterNext ? 0.17145 : -0.17145;
            
            SimulatedArena.getInstance().addGamePieceProjectile(
                    new RebuiltFuelOnFly(
                            this.container.getDriveSubsystem().getPose().getTranslation(),
                            new Translation2d(-0.18415, yOffset),
                            this.container.getDriveSubsystem().getRobotChassisSpeeds(),
                            this.container.getDriveSubsystem().getPose().getRotation(),
                            Meters.of(0.4446524),
                            MetersPerSecond.of(velocity),
                            Degrees.of(20.0 + Units.radiansToDegrees(Shooter.getInstance().getPosition()) + 90.0))
                        .withTargetTolerance(new Translation3d(0.1, 0.1, 0.2)));

            this.isRightShooterNext = !this.isRightShooterNext;
        }
    }

    @Override
    public void periodic() {
        this.handleIntakeSimulation();
        this.proccessShooter();
    }
}
