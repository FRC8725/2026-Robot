package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.lib.simulation.MapleSimDrivetrain;

public class CommandSwerveDrivetrain {
    SwerveDrivetrainConstants drivetrainConstants;
    SwerveModuleConstants<?, ?, ?>[] moduleConstants;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... models) {
        this.drivetrainConstants = drivetrainConstants;
        if (Constants.useMapleSim) {
            this.moduleConstants = MapleSimDrivetrain.regulateModuleConstantsForSimulation(models);
        } else {
            this.moduleConstants = models;
        }
    }

    public SwerveDrivetrainConstants getConstants() {
        return this.drivetrainConstants;
    }

    public SwerveModuleConstants<?, ?, ?>[] getModuleConstants() {
        return this.moduleConstants;
    }
}
