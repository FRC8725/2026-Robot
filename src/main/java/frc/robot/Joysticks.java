package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.SuperStructure;

public class Joysticks {
    private final XboxController driver = new XboxController(0);
    public final Supplier<Boolean> up = () -> this.driver.getXButton();
    public final Supplier<Boolean> down = () -> this.driver.getBButton();
    public final Supplier<Boolean> fup = () -> this.driver.getPOV() == 180;
    public final Supplier<Boolean> fdown = () -> this.driver.getPOV() == 0;

    private AlignMode launchAlignMode = AlignMode.None;


    public enum AlignMode {
        None,
        ZoneAlign,
        PointAlign
    }

    public static class DriveInputs {
        public double leftY;
        public double leftX;
        public double rightX;
        public double deadZone;
        public boolean oriented;
        public AlignMode alignMode;
        public boolean wantTrack;

        public boolean isNonZero() {
            return Math.abs(leftX) > deadZone ||
                Math.abs(leftY) > deadZone ||
                Math.abs(rightX) > deadZone;
        }

        public boolean isRotateZero() {
            return Math.abs(rightX) < deadZone;
        }

        public DriveInputs getRedFlipped() {
            DriveInputs flipped = new DriveInputs();
            flipped.leftX = this.leftX;
            flipped.leftY = this.leftY;
            flipped.rightX = this.rightX;
            flipped.deadZone = deadZone;
            flipped.oriented = oriented;
            flipped.alignMode = alignMode;
            flipped.wantTrack = wantTrack;
            return flipped;
        }
    }

    public DriveInputs getDriveInput() {
        DriveInputs input = new DriveInputs();
        input.leftX = this.driver.getLeftX() * (Robot.isRedAlliance.get() ? -1.0 : 1.0);
        input.leftY = this.driver.getLeftY() * (Robot.isRedAlliance.get() ? -1.0 : 1.0);
        input.rightX = this.driver.getRightX();
        input.oriented = this.driver.getLeftBumperButton();
        input.deadZone = 0.03;
        input.wantTrack = this.driver.getAButton();

        boolean wantScore = this.getInput().wantScore;

        if (wantScore) {
            if (this.launchAlignMode == AlignMode.None) {
                this.launchAlignMode = Robot.isInAllianceZone.get()
                        ? AlignMode.ZoneAlign
                        : AlignMode.PointAlign;
            }
        } else {
            this.launchAlignMode = AlignMode.None;
        }

        input.alignMode = this.launchAlignMode;
        
        return input;
    }

    public SuperStructure.StructureInput getInput() {
        SuperStructure.StructureInput input = new SuperStructure.StructureInput();

        input.wantIntake = this.driver.getRightTriggerAxis() > 0.3;
        input.wantScore = this.driver.getRightBumperButton();
        input.resetHood = this.driver.getLeftBumperButton();
        
        return input;
    }
}
