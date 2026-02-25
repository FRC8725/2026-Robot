package frc.robot.lib.helpers;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj.DriverStation;

public class FieldTimer {
    @AutoLogOutput(key = "Field/MatchTime")
    public double getMatchTime() {
        return DriverStation.getMatchTime();
    }

    @AutoLogOutput(key = "Field/BlueHubTime")
    public double getBlueHubTime() {
        if (!DriverStation.isTeleopEnabled() && !DriverStation.isAutonomousEnabled()) {
            return 0.0;
        }
        if (DriverStation.isAutonomousEnabled()) {
            return 0.0; 
        }

        double matchTime = DriverStation.getMatchTime(); 
        String gameData = DriverStation.getGameSpecificMessage();

        if (gameData.isEmpty())
            return 0.0;

        boolean reverse = false;
        switch (gameData.charAt(0)) {
            case 'R' -> reverse = true;
            case 'B' -> reverse = false;
            default -> reverse = false;
        }

        if (matchTime > 130) {
            return matchTime - 130; // Transition shift (藍方預設 active)
        } else if (matchTime > 105) {
            return reverse ? (matchTime - 105) : 0.0; // Shift 1
        } else if (matchTime > 80) {
            return !reverse ? (matchTime - 80) : 0.0; // Shift 2 (狀態與 Shift 1 相反)
        } else if (matchTime > 55) {
            return reverse ? (matchTime - 55) : 0.0;  // Shift 3 (狀態與 Shift 1 相同)
        } else if (matchTime > 30) {
            return !reverse ? (matchTime - 30) : 0.0; // Shift 4 (狀態與 Shift 1 相反)
        } else {
            return matchTime; // End game (<=30秒預設 active，回傳最後的剩餘時間)
        }
    }

    @AutoLogOutput(key = "Field/RedHubTime")
    public double getRedHubTime() {
        if (!DriverStation.isTeleopEnabled() && !DriverStation.isAutonomousEnabled()) {
            return 0.0;
        }
        if (DriverStation.isAutonomousEnabled()) {
            return 0.0; 
        }

        double matchTime = DriverStation.getMatchTime(); 
        String gameData = DriverStation.getGameSpecificMessage();

        if (gameData.isEmpty())
            return 0.0;

        boolean reverse = false;
        switch (gameData.charAt(0)) {
            case 'R' -> reverse = true;
            case 'B' -> reverse = false;
            default -> reverse = false;
        }

        if (matchTime > 130) {
            return matchTime - 130; // Transition shift (藍方預設 active)
        } else if (matchTime > 105) {
            return !reverse ? (matchTime - 105) : 0.0; // Shift 1
        } else if (matchTime > 80) {
            return reverse ? (matchTime - 80) : 0.0; // Shift 2 (狀態與 Shift 1 相反)
        } else if (matchTime > 55) {
            return !reverse ? (matchTime - 55) : 0.0;  // Shift 3 (狀態與 Shift 1 相同)
        } else if (matchTime > 30) {
            return reverse ? (matchTime - 30) : 0.0; // Shift 4 (狀態與 Shift 1 相反)
        } else {
            return matchTime; // End game (<=30秒預設 active，回傳最後的剩餘時間)
        }
    }
}
