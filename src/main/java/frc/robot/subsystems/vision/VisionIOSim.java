package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degree;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.lib.math.MathHelpers;

public class VisionIOSim extends VisionIOHardware {
    private final PhotonCamera cameraA = new PhotonCamera("cameraA");
    private final PhotonCamera cameraB = new PhotonCamera("cameraB");
    private PhotonCameraSim cameraASim;
    private PhotonCameraSim cameraBSim;
    private final VisionSystemSim visionSim = new VisionSystemSim("main");

    private final RobotState robotState;

    public VisionIOSim(RobotState robotState) {
        this.robotState = robotState;
        this.visionSim.addAprilTags(Constants.kAprilTagLayout);

        SimCameraProperties prop = new SimCameraProperties();
        prop.setCalibration(1280, 800, Rotation2d.fromDegrees(97.7)); // TODO Simulation
        prop.setCalibError(0.35, 0.5);
        prop.setFPS(45);
        prop.setAvgLatencyMs(20);
        prop.setLatencyStdDevMs(5);
        prop.setExposureTimeMs(0.65);

        this.cameraASim = new PhotonCameraSim(this.cameraA, prop);
        this.cameraASim.setMinTargetAreaPixels(1000);
        this.cameraBSim = new PhotonCameraSim(this.cameraB, prop);
        this.cameraBSim.setMinTargetAreaPixels(1000);

        this.visionSim.addCamera(
                cameraASim, MathHelpers.reversePitch(Constants.Vision.CAMERA_LEFT_TRANSFORM));
        this.visionSim.addCamera(
                cameraBSim, MathHelpers.reversePitch(Constants.Vision.CAMERA_RIGHT_TRANSFORM));

        this.cameraASim.enableRawStream(true);
        this.cameraASim.enableProcessedStream(true);
        this.cameraASim.enableDrawWireframe(true);
        this.cameraBSim.enableRawStream(true);
        this.cameraBSim.enableProcessedStream(true);
        this.cameraBSim.enableDrawWireframe(true);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        Pose2d estimatedPose = this.robotState.getLastestFieldToRobot();
        if (estimatedPose != null) {
            this.visionSim.update(estimatedPose);
            Logger.recordOutput("Vision/SimIO/updateSimPose", estimatedPose);
        }

        NetworkTable tableA =
                NetworkTableInstance.getDefault().getTable(Constants.Vision.CAMERA_LEFT_NAME);
        NetworkTable tableB = 
                NetworkTableInstance.getDefault().getTable(Constants.Vision.CAMERA_RIGHT_NAME);

        this.writeToTable(this.cameraA.getAllUnreadResults(), tableA, this.cameraASim);
        this.writeToTable(this.cameraB.getAllUnreadResults(), tableB, this.cameraBSim);

        super.updateInputs(inputs);
    }

    /** Generates robot pose data from PhotonVision results. */
    private List<Double> getBotpose(
            Transform3d fieldToCamera,
            int numTags,
            PhotonPipelineResult result,
            PhotonCameraSim cameraSim) {
        if (result == null || result.targets.isEmpty()) return null;

        Optional<Transform3d> optRobotToCamera =
                visionSim.getRobotToCamera(cameraSim, Timer.getFPGATimestamp());
        Pose3d fieldToRobot;
        if (optRobotToCamera.isPresent()) {
            Transform3d cameraToRobot = optRobotToCamera.get().inverse();
            Pose3d robotPose3d =
                    new Pose3d(fieldToCamera.getTranslation(), fieldToCamera.getRotation())
                            .transformBy(cameraToRobot);
            fieldToRobot = robotPose3d;
        } else {
            fieldToRobot = new Pose3d(fieldToCamera.getTranslation(), fieldToCamera.getRotation());
        }

        List<Double> pose_data =
                new ArrayList<>(
                        Arrays.asList(
                                fieldToRobot.getX(),
                                fieldToRobot.getY(),
                                fieldToRobot.getZ(),
                                0.0,
                                0.0,
                                fieldToRobot.getRotation().getMeasureZ().in(Degree),
                                result.metadata.getLatencyMillis(),
                                (double) numTags,
                                0.0,
                                0.0,
                                result.getBestTarget().getArea()));

        for (var target : result.targets) {
            pose_data.addAll(
                    Arrays.asList(
                            (double) target.getFiducialId(),
                            target.getYaw(), // txnc
                            target.getPitch(), // tync
                            target.area, // ta
                            0.0, // distToCamera
                            0.0, // distToRobot
                            target.getPoseAmbiguity() // ambiguity
                            ));
        }
        return pose_data;
    }

    /**
     * Writes simulated vision data to NetworkTables for consumption by Limelight processing code.
     */
    private void writeToTable(
            List<PhotonPipelineResult> results, NetworkTable table, PhotonCameraSim cameraSim) {
        boolean seesTarget = false;
        for (var result : results) {
            List<Double> pose_data = null;
            if (result.getMultiTagResult().isPresent()) {
                var multiTagResult = result.getMultiTagResult().get();
                Transform3d best = multiTagResult.estimatedPose.best;

                pose_data = this.getBotpose(
                        best, multiTagResult.fiducialIDsUsed.size(), result, cameraSim);
            } else if (result.hasTargets()) {
                var bestTarget = result.getBestTarget();
                Transform3d best =
                        Constants.kAprilTagLayout
                                .getTagPose(bestTarget.getFiducialId())
                                .get()
                                .minus(Pose3d.kZero)
                                .plus(bestTarget.bestCameraToTarget.inverse());

                pose_data = this.getBotpose(best, 1, result, cameraSim);
            }

            if (pose_data != null) {
                table.getEntry("botpose_wpiblue")
                        .setDoubleArray(
                                pose_data.stream().mapToDouble(Double::doubleValue).toArray());
                table.getEntry("botpose_orb_wpiblue")
                        .setDoubleArray(
                                pose_data.stream().mapToDouble(Double::doubleValue).toArray());
                // [MT1x, MT1y, MT1z, MT1roll, MT1pitch, MT1Yaw, MT2x, MT2y, MT2z, MT2roll,
                // MT2pitch,
                // MT2yaw]
                table.getEntry("stddevs")
                        .setDoubleArray(
                                new Double[] {
                                    0.3, 0.3, 0.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
                                });
                seesTarget = true;
            }
            table.getEntry("cl").setDouble(result.metadata.getLatencyMillis());
        }
        table.getEntry("tv").setInteger(seesTarget ? 1 : 0);
    }
}
