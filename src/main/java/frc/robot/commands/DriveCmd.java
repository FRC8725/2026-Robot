// Copyright (c) FIRST and other WPILib contributors.
package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Joysticks;
import frc.robot.Robot;
import frc.robot.lib.MathHelper;
// import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class DriveCmd extends Command {
	private final Swerve swerve;
	private final Supplier<Joysticks.DriveInputs> driveInputs;
	private final FuelTracking fuelTracking;
	private boolean isTraking = false;
	
	private final SlewRateLimiter xLimiter = new SlewRateLimiter(4.5);
	private final SlewRateLimiter yLimiter = new SlewRateLimiter(4.5);
	private final SlewRateLimiter rLimiter = new SlewRateLimiter(4.5);

	private final PIDController facingHubPid = new PIDController(0, 0, 0);

	public DriveCmd(Swerve swerve, Vision vision, Supplier<Joysticks.DriveInputs> driveInputs) {
		this.swerve = swerve;
		this.driveInputs = driveInputs;
		this.fuelTracking = new FuelTracking(swerve, vision);
		this.addRequirements(this.swerve);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		Joysticks.DriveInputs inputs = this.driveInputs.get();
		if (Robot.isRedAlliance.get()) inputs = inputs.getRedFlipped();

		if (inputs.wantTrack) {
			if (!this.isTraking) {
				this.fuelTracking.initialize();
				this.isTraking = true;
			}
			this.fuelTracking.execute();
		} else {
			if (this.isTraking) {
				this.fuelTracking.end(true);
				this.isTraking = false;
			}
			if (!inputs.isNonZero()) {
				// Facing hub
				ChassisSpeeds speeds = this.getSpeeds();
				double measurement = this.swerve.getPose().getRotation().getRadians();
				double setpoint = MathHelper.getAngleFromHub(this.swerve.getPose()).getRadians();
				speeds.omegaRadiansPerSecond =
						this.facingHubPid.calculate(measurement, setpoint);
			}
			this.swerve.driveRobotRelative(this.getSpeeds(), !inputs.oriented);
		}
	}

	@Override
	public void end(boolean interrupted) {
		// this.swerve.stopModules();
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
