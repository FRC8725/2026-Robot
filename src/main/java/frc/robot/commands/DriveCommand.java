package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.lib.math.MathHelpers;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class DriveCommand extends Command {
    private final Drive driveSubsystem;
    private final Supplier<Double> xSpeed, ySpeed, rSpeed;

    private final SwerveRequest.FieldCentric driveRequest =
            new SwerveRequest.FieldCentric()
                    .withDeadband(3.0 * 0.05)
                    .withRotationalDeadband(
                            8.2
                                    * 0.05)
                    .withDriveRequestType(DriveRequestType.Velocity);
    private final SwerveRequest.FieldCentricFacingAngle driveWithHeading =
            new SwerveRequest.FieldCentricFacingAngle()
                    .withDeadband(0.05)
                    .withDriveRequestType(DriveRequestType.Velocity);

    public DriveCommand(
            Drive driveSubsystem,
            Supplier<Double> xSpeed,
            Supplier<Double> ySpeed,
            Supplier<Double> rSpeed) {
        this.driveSubsystem = driveSubsystem;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rSpeed = rSpeed;
        this.addRequirements(this.driveSubsystem);

        this.driveWithHeading.HeadingController.setPID(12.0, 0.0, 0.0);

        if (RobotBase.isSimulation()) {
            this.driveRequest.DriveRequestType = DriveRequestType.OpenLoopVoltage;
            this.driveWithHeading.DriveRequestType = DriveRequestType.OpenLoopVoltage;
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

        Rotation2d targetAngle = MathHelpers.getAngleFromHub(this.driveSubsystem.getPose());

        Logger.recordOutput("target", targetAngle.getRadians());
        Logger.recordOutput("measure", this.driveSubsystem.getPose().getRotation().getRadians());
        // this.driveSubsystem.setControl(
        //         this.driveWithHeading
        //                 .withVelocityX(xSpeed)
        //                 .withVelocityY(ySpeed)
        //                 .withTargetDirection(targetAngle));

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
