package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class AprilTagCamera extends SubsystemBase{

    private static boolean doEstimation = true;
    private static Drivetrain drivetrain = RobotContainer.getDrivetrain();

    private static AprilTagFieldLayout fieldLayout;
    private final PhotonCamera camera;
    private final String cameraName;
    private final Transform3d robotToCam;
    private final PhotonPoseEstimator poseEstimator;

    public AprilTagCamera(String cameraName, Transform3d robotToCam) {
        try {
            fieldLayout = new AprilTagFieldLayout(Path.of("src\\main\\deploy\\ApriltagJSON\\2025-reefscape.json"));
        } catch(IOException e) {
            System.out.println(e);
        }

        this.cameraName = cameraName;
        this.camera = new PhotonCamera(cameraName);
        this.robotToCam = robotToCam;
        this.poseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
    }

    @Override
    public void periodic() {
        if (doEstimation) {
            
            var results = camera.getAllUnreadResults();
            if (!results.isEmpty()) {
                for (PhotonPipelineResult result : results) {
                    if (result.getMultiTagResult().isPresent()) {
                        Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(result);
                        if (estimatedPose.isPresent()) {
                            drivetrain.addVisionMeasurement(new Pose2d(estimatedPose.get().estimatedPose.toPose2d().getTranslation(), drivetrain.getRotation3d().toRotation2d()));
                        }
                    }
                }
            }
        }
    }

    public static void turnOffApriltags() {
        AprilTagCamera.doEstimation = false;
    }

    public static void turnOnApriltags() {
        AprilTagCamera.doEstimation = true;
    }

}
