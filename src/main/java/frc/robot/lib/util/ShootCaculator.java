package frc.robot.lib.util;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.Shooter;

public class ShootCaculator {
    private final double MIN_DISTANCE = 9.88;
    private final double MAX_DISTANCE = 0.0;
    private final double DEFAULT_VELOCITY = Shooter.FlywheelState.Auto.speed;

    private final InterpolatingDoubleTreeMap shooterAngleMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap shooterVelocityMap = new InterpolatingDoubleTreeMap();

    public ShootCaculator() {
        for (Pair<Double, Double> pair : Constants.SHOOTER_ANGLE_MAP) {
            this.shooterAngleMap.put(pair.getFirst(), pair.getSecond());
        }
    }

    public double getHoodAngle(double distance) {
        if (distance > MIN_DISTANCE) {
            return MIN_DISTANCE;
        } else {
            return this.shooterVelocityMap.get(distance);
        }
    }

    public double getFlywheelVelocity(double distance) {
        if (distance > MIN_DISTANCE) {
            return DEFAULT_VELOCITY;
        } else {
            return this.shooterVelocityMap.get(distance);
        }
    }
}
