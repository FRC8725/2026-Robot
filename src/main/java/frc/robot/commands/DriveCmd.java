// Copyright (c) FIRST and other WPILib contributors.
package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Joysticks;
import frc.robot.Robot;
import frc.robot.subsystems.Swerve;

public class DriveCmd extends Command {
	private final Swerve swerve;
	private final Supplier<Joysticks.DriveInputs> driveInputs;
	
	private final SlewRateLimiter xLimiter = new SlewRateLimiter(4.5);
	private final SlewRateLimiter yLimiter = new SlewRateLimiter(4.5);
	private final SlewRateLimiter rLimiter = new SlewRateLimiter(4.5);

	public DriveCmd(Swerve swerve, Supplier<Joysticks.DriveInputs> driveInputs) {
		this.swerve = swerve;
		this.driveInputs = driveInputs;
		this.addRequirements(this.swerve);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		Joysticks.DriveInputs inputs = this.driveInputs.get();
		if (Robot.isRedAlliance.get()) inputs = inputs.getRedFlipped();
		
		this.swerve.driveRobotRelative(this.getSpeeds(), !inputs.oriented);
	}

	@Override
	public void end(boolean interrupted) {
		this.swerve.stopModules();
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	public ChassisSpeeds getSpeeds() {
		double x = this.xLimiter.calculate(-this.driveInputs.get().leftY);
		double y = this.yLimiter.calculate(-this.driveInputs.get().leftX);
		double rot = this.rLimiter.calculate(-this.driveInputs.get().rightX);

		double theta = Math.atan2(y, x);
		double r = Math.hypot(x, y);
		
		r = this.deadZone(r, this.driveInputs.get().deadZone);
		rot = this.deadZone(rot, this.driveInputs.get().deadZone);

		r = r * r;
		rot = rot * rot * Math.signum(rot);

		double xSpeed = r * Math.cos(theta) * Constants.Swerve.MAX_VELOCITY;
		double ySpeed = r * Math.sin(theta) * Constants.Swerve.MAX_VELOCITY;
		double rSpeed = rot * Constants.Swerve.MAX_ANGULAR_VELOCITY;

		return new ChassisSpeeds(xSpeed, ySpeed, rSpeed);
	}

	public double deadZone(double input, double deadZone) {
		if (Math.abs(input) < deadZone) return 0.0;
		else if (input > 1.0) return 1.0;
		else if (input < -1.0) return -1.0;
		else return (input - deadZone) / (1.0 - deadZone);
	}
}
