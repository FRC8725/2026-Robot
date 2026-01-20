package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.lib.math.MathHelpers;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.function.Supplier;

public class DriveCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final Supplier<Double> xSpeed, ySpeed, rSpeed;
    private double joystickLastTouched = -1.0;

    private final SwerveRequest.FieldCentric driveRequest =
            new SwerveRequest.FieldCentric()
                    .withDeadband(3.0 * 0.05)
                    .withRotationalDeadband(
                            8.2
                                    * 0.05)
                    .withDriveRequestType(DriveRequestType.Velocity);

    public DriveCommand(
            DriveSubsystem driveSubsystem,
            Supplier<Double> xSpeed,
            Supplier<Double> ySpeed,
            Supplier<Double> rSpeed) {
        this.driveSubsystem = driveSubsystem;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rSpeed = rSpeed;
        this.addRequirements(this.driveSubsystem);

        if (RobotBase.isSimulation()) {
            this.driveRequest.DriveRequestType = DriveRequestType.OpenLoopVoltage;
        }
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double xSpeed = this.xSpeed.get() * 3.0;
        double ySpeed = this.ySpeed.get() * 3.0;
        double rSpeed = this.rSpeed.get();

        xSpeed *= Robot.isRedAlliance.get() ? 1.0 : -1.0;
        ySpeed *= Robot.isRedAlliance.get() ? 1.0 : -1.0;

        xSpeed = MathUtil.applyDeadband(xSpeed, 0.05);
        ySpeed = MathUtil.applyDeadband(ySpeed, 0.05);
        rSpeed = MathUtil.applyDeadband(rSpeed, 0.05);

        if (Math.abs(rSpeed) > 0.05) {
            this.joystickLastTouched = Timer.getFPGATimestamp();
        }
        // if (Math.abs(rSpeed) > Constants.DriveConstants.STEER_JOYSTICK_DEADBAND
        //         || (MathHelpers.epsilonEqal(
        //                         this.joystickLastTouched, Timer.getFPGATimestamp(), 0.25)
        //                 && Math.abs(
        //                                 this.driveSubsystem.getRobotChassisSpeeds()
        //                                         .omegaRadiansPerSecond)
        //                         > Math.toRadians(10.0))) {
        //     this.driveSubsystem.setControl(
        //             this.driveRequest
        //                     .withVelocityX(xSpeed)
        //                     .withVelocityY(ySpeed)
        //                     .withRotationalRate(
        //                             rSpeed * Constants.DriveConstants.MAX_ANGULAR_RATE));
        // }
        this.driveSubsystem.setControl(
                    this.driveRequest
                            .withVelocityX(xSpeed)
                            .withVelocityY(ySpeed)
                            .withRotationalRate(
                                    rSpeed * 8.2));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
