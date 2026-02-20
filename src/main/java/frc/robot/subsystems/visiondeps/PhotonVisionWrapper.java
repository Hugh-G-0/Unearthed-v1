package frc.robot.subsystems.visiondeps;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * PhotonVision wrapper that exposes:
 *  - getLatestResult() to read the camera result (cheap)
 *  - getEstimatedGlobalPose(prevPose) convenience method (reads latest result internally)
 *  - getEstimatedGlobalPose(prevPose, result) preferred overload that uses a provided result
 *
 * Keep this class in its own file: PhotonVisionWrapper.java
 */
public class PhotonVisionWrapper {

    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;

    public PhotonVisionWrapper() throws IOException {
        camera = new PhotonCamera(VisionConstants.CAMERA_NAME);

        AprilTagFieldLayout fieldLayout =
            AprilTagFields.k2026RebuiltAndymark.loadAprilTagLayoutField();

        Transform3d robotToCam = VisionConstants.ROBOT_TO_CAM;

        // Use the constructor available in your PhotonVision version.
        // If your PhotonVision library requires a different signature, adjust accordingly.
        poseEstimator = new PhotonPoseEstimator(
            fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToCam
        );
    }

    /** Cheap, non-blocking read of the latest pipeline result. */
    public PhotonPipelineResult getLatestResult() {
        return camera.getLatestResult();
    }

    /** Convenience: read latest result internally and run pose estimation. */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimate) {
        PhotonPipelineResult result = getLatestResult();
        return getEstimatedGlobalPose(prevEstimate, result);
    }

    /**
     * Preferred: use the PhotonPipelineResult you already read this loop to avoid races.
     * Returns Optional.empty() if no targets or estimator cannot produce a pose.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimate, PhotonPipelineResult result) {
        if (result == null || !result.hasTargets()) {
            return Optional.empty();
        }

        poseEstimator.setReferencePose(prevEstimate);
        return poseEstimator.update(result);
    }
}