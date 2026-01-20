package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {
    private final TalonFX centerMaster = new TalonFX(0);
    private final TalonFX centerFollow = new TalonFX(0);
    private final TalonFX roller = new TalonFX(0);
    private final Follower follower = new Follower(this.centerMaster.getDeviceID(), MotorAlignmentValue.Aligned);

    private double centerVolt = 0.0;
    private double rollerVolt = 0.0;
    private boolean canRun = false;

    public Hopper() {
        TalonFXConfiguration centerConfig = new TalonFXConfiguration();
        centerConfig.CurrentLimits
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(40.0);
        centerConfig.MotorOutput
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast);
        this.centerMaster.getConfigurator().apply(centerConfig);
        this.centerFollow.getConfigurator().apply(centerConfig);

        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        rollerConfig.CurrentLimits
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(40.0);
        rollerConfig.MotorOutput
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast);
        this.roller.getConfigurator().apply(rollerConfig);

        SmartDashboard.putNumber("RollerVolts", 0.0);
        SmartDashboard.putNumber("CenterVolts", 0.0);
        SmartDashboard.putBoolean("CanRun", this.canRun);
    }

    @Override
    public void periodic() {
        this.rollerVolt = SmartDashboard.getNumber("RollerVolts", 0.0);
        this.centerVolt = SmartDashboard.getNumber("CenterVolts", 0.0);
        this.canRun = SmartDashboard.getBoolean("CanRun", false);

        if (this.canRun) {
            this.centerMaster.setVoltage(this.centerVolt);
            this.centerFollow.setControl(this.follower);
            this.roller.setVoltage(this.rollerVolt);
        } else {
            this.centerMaster.stopMotor();
            this.centerFollow.setControl(this.follower);
            this.roller.stopMotor();
        }
    }
}
