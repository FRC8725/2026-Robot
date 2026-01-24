package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.SuperStructure;

public class Joysticks {
    private final XboxController driver = new XboxController(0);

    public static class DriveInputs {
        public double leftY;
        public double leftX;
        public double rightX;
        public double deadZone;
        public boolean oriented;

        public boolean isNonZero() {
            return Math.abs(leftX) > deadZone ||
                Math.abs(leftY) > deadZone ||
                Math.abs(rightX) > deadZone;
        }

        public DriveInputs getRedFlipped() {
            DriveInputs flipped = new DriveInputs();
            flipped.leftX = this.leftX;
            flipped.leftY = this.leftY;
            flipped.rightX = this.rightX;
            flipped.deadZone = deadZone;
            flipped.oriented = oriented;
            return flipped;
        }
    }

    public DriveInputs getDriveInput() {
        DriveInputs input = new DriveInputs();
        input.leftX = this.driver.getLeftX() * (Robot.isRedAlliance.get() ? -1.0 : 1.0);
        input.leftY = this.driver.getLeftY() * (Robot.isRedAlliance.get() ? -1.0 : 1.0);
        input.rightX = this.driver.getRightX();
        input.oriented = this.driver.getLeftBumperButton();
        input.deadZone = 0.05;
        
        return input;
    }

    public SuperStructure.StructureInput getInput() {
        SuperStructure.StructureInput input = new SuperStructure.StructureInput();

        return input;
    }
}
