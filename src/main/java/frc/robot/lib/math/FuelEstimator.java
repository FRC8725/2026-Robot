package frc.robot.lib.math;

import org.apache.commons.math3.geometry.euclidean.threed.Plane;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class FuelEstimator {
    private static final double TOLERANCE = 0.001;
    private static final Vector3D CAMERA_VEC = new Vector3D(0.143019, -0.346677, 0.341093);
    private static final Vector3D CENTRAL_SIGHT = new Vector3D(
            0.8137976813494, 0.296198132726, -0.50);
    private static final Vector3D X_AXIS = new Vector3D(
        0.2961981327260, -0.8137976813494, 0.0);
    private static final Vector3D Y_AXIS = new Vector3D(
        0.4068988406747, 0.148099066363, 0.75);
    private static final Plane GROUND = new Plane(
            new Vector3D(0.0, 0.0, Units.inchesToMeters(5.91 / 2.0)),
            new Vector3D(0.0, 0.0, 1.0),
            TOLERANCE);

    public static Translation2d getFuelTranslation(double tx, double ty) {
        Rotation xRot = new Rotation(Y_AXIS, -tx, RotationConvention.VECTOR_OPERATOR);
        Rotation yRot = new Rotation(X_AXIS, ty, RotationConvention.VECTOR_OPERATOR);

        Vector3D xVec = xRot.applyTo(CENTRAL_SIGHT);
        Vector3D yVec = yRot.applyTo(CENTRAL_SIGHT);

        Plane xPlane = new Plane(CAMERA_VEC, CAMERA_VEC.add(xVec), CAMERA_VEC.add(Y_AXIS), TOLERANCE);
        Plane yPlane = new Plane(CAMERA_VEC, CAMERA_VEC.add(yVec), CAMERA_VEC.add(X_AXIS), TOLERANCE);

        Vector3D intersect = Plane.intersection(xPlane, yPlane, GROUND);
        Vector2D objectVec = new Vector2D(intersect.getX(), intersect.getY());

        if (objectVec.getNorm() == 0) return new Translation2d();

        return new Translation2d(objectVec.getX(), objectVec.getY());
    }
}
