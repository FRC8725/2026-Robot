package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Motor extends SubsystemBase {
    private final TalonFX motor = new TalonFX(30);
    private final SendableChooser<String> modeChooser = new SendableChooser<>();

    public static double VALUE = 0.0;

    public Motor() {
        this.modeChooser.setDefaultOption("NULL", "NULL");
        this.modeChooser.addOption("Percent", "Percent");
        this.modeChooser.addOption("Voltage", "Voltage");
        SmartDashboard.putData("ModeChooser", this.modeChooser);
        SmartDashboard.putBoolean("CanRUN", false);

        this.modeChooser.onChange((String selected) -> {
            VALUE = 0.0;
        });
    }

    public String getMode() {
        return this.modeChooser.getSelected();
    }

    @Override
    public void periodic() {
        boolean canRun = SmartDashboard.getBoolean("CanRUN", false);
        SmartDashboard.putNumber("Value", VALUE);
        
        if (!canRun) {
            this.motor.stopMotor();
            return;
        }

        if (this.modeChooser.getSelected() == "Percent") {
            this.motor.set(VALUE);
        } else if (this.modeChooser.getSelected() == "Voltage") {
            this.motor.setVoltage(VALUE);
        } else {
            this.motor.stopMotor();
            VALUE = 0.0;
        }
        
    }
}
