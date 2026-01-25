package frc.robot.lib.limelight;

import java.util.Arrays;
import java.util.Objects;

public record FiducialObservation(int id, double txnc, double tync, double ambiguity, double area) {
    public static FiducialObservation fromLimelight(LimelightHelpers.RawFiducial fiducial) {
        if (fiducial == null) {
            return null;
        }
        return new FiducialObservation(
                fiducial.id,
                fiducial.txnc,
                fiducial.tync,
                fiducial.ambiguity,
                fiducial.ta);
    }

    public static FiducialObservation[] fromLimelight(LimelightHelpers.RawFiducial[] fiducials) {
        if (fiducials == null) {
            return new FiducialObservation[0];
        }
        return Arrays.stream(fiducials)
                .map(FiducialObservation::fromLimelight)
                .filter(Objects::nonNull)
                .toArray(FiducialObservation[]::new);
    }
}