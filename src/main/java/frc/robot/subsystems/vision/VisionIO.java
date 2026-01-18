package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.lib.limelight.FiducialObservation;
import frc.robot.lib.limelight.MegatagPoseEstimate;

public interface VisionIO {
    public class VisionIOInputs {
        public class CameraInputs {
            public boolean hasTarget;
            public FiducialObservation[] fiducialObservations;
            public MegatagPoseEstimate megatagPoseEstimate;
            public MegatagPoseEstimate megatag2PoseEstimate;
            public int megatag2Count;
            public int megatagCount;
            public Pose3d pose3d;
            public double[] standardDeviations = new double[12];
        }

        public CameraInputs cameraA = new CameraInputs();
        public CameraInputs cameraB = new CameraInputs();
    }

    void updateInputs(VisionIOInputs inputs);
}
