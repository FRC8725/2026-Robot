package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.Vision;
import frc.robot.lib.limelight.FiducialObservation;
import frc.robot.lib.limelight.LimelightHelpers;
import frc.robot.lib.limelight.MegatagPoseEstimate;

public class VisionIOHardware implements VisionIO {
    private final NetworkTable tableA = NetworkTableInstance.getDefault()
            .getTable(Vision.CAMERA_LEFT_NAME);
    private final NetworkTable tableB = NetworkTableInstance.getDefault()
            .getTable(Vision.CAMERA_RIGHT_NAME);
    private final double[] DEFAULT_STDDEVS = new double[Vision.kExpectedStdDevArrayLength];

    public VisionIOHardware() {
        this.setLimelightSettings();
    }

    private void setLimelightSettings() {
        double[] cameraAPose = {
            Vision.CAMERA_LEFT_TRANSFORM.getX(),
            Vision.CAMERA_LEFT_TRANSFORM.getY(),
            Vision.CAMERA_LEFT_TRANSFORM.getZ(),
            0.0,
            Vision.CAMERA_LEFT_DEGS,
            0.0
        };
        this.tableA.getEntry("camerapose_robotspace_set").setDoubleArray(cameraAPose);

        double[] cameraBPose = {
            Vision.CAMERA_RIGHT_TRANSFORM.getX(),
            Vision.CAMERA_RIGHT_TRANSFORM.getY(),
            Vision.CAMERA_RIGHT_TRANSFORM.getZ(),
            0.0,
            Vision.CAMERA_RIGHT_DEGS,
            0.0
        };
        this.tableB.getEntry("camerapose_robotspace_set").setDoubleArray(cameraBPose);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        this.updateCameraData(this.tableA, inputs.cameraA, Vision.CAMERA_RIGHT_NAME);
        this.updateCameraData(this.tableB, inputs.cameraB, Vision.CAMERA_RIGHT_NAME);    
    }

    private void updateCameraData(NetworkTable table, VisionIOInputs.CameraInputs camera, String limelightName) {
        camera.hasTarget = table.getEntry("tv").getDouble(0.0) == 1.0;
        if (camera.hasTarget) {
            try {
                var megatag = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
                var robotPose3d = LimelightHelpers.toPose3D(LimelightHelpers.getBotPose_wpiBlue(limelightName));

                if (megatag != null) {
                    camera.megatagPoseEstimate = MegatagPoseEstimate.fromLimelight(megatag);
                    camera.megatagCount = megatag.tagCount;
                    camera.fiducialObservations = FiducialObservation.fromLimelight(megatag.rawFiducials);
                }
                if (robotPose3d != null) {
                    camera.pose3d = robotPose3d;
                }
                camera.standardDeviations = table.getEntry("stddevs").getDoubleArray(DEFAULT_STDDEVS);
            } catch (Exception e) {
                System.out.println("Error processing Limelight data: " + e.getMessage());
            }
        }
    }
}
