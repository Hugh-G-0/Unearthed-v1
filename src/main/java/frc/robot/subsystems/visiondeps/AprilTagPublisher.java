package frc.robot.subsystems.visiondeps;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class AprilTagPublisher {

    private final NetworkTable table;

    // Track the last-seen id and the previous set of ids so we can determine "last detected"
    private long lastSeenId = -1;
    private long[] prevIds = new long[] {};

    public AprilTagPublisher() {
        table = NetworkTableInstance.getDefault().getTable("apriltags");
    }

    /** Helper: round a double to 3 decimals */
    private double r3(double v) {
        return Math.round(v * 1000.0) / 1000.0;
    }

    /** Helper: round an array to 3 decimals */
    private double[] r3(double[] arr) {
        double[] out = new double[arr.length];
        for (int i = 0; i < arr.length; i++) {
            out[i] = r3(arr[i]);
        }
        return out;
    }

    public void publishNoTarget() {
        table.getEntry("has_target").setBoolean(false);
        table.getEntry("tags").setIntegerArray(new long[] {});
        // Do not overwrite lastSeenId here so the "last detected" id persists when no targets are visible.
        table.getEntry("last_distance").setDouble(-1);
        table.getEntry("last_yaw").setDouble(0);
        table.getEntry("last_pitch").setDouble(0);
        table.getEntry("ambiguity").setDouble(1.0);
        table.getEntry("confidence").setDouble(0.0);
        table.getEntry("target_count").setInteger(0);
        table.getEntry("vision_ok").setBoolean(false);
        table.getEntry("cam_to_tag").setDoubleArray(new double[] {});
        table.getEntry("robot_pose_array").setDoubleArray(new double[] {});
        table.getEntry("latency_ms").setDouble(0);
        table.getEntry("last_filter_reason").setString("NO_TARGETS");
        // NOTE: intentionally do NOT touch tuning entries under "tune/*" here so dashboard sliders persist
        // Publish the persisted lastSeenId so dashboard still shows it
        table.getEntry("last_id").setInteger((int) lastSeenId);
    }

    public void publishDetections(PhotonPipelineResult result) {

        List<PhotonTrackedTarget> targets = result.getTargets();
        long[] ids = targets.stream().mapToLong(t -> t.getFiducialId()).toArray();

        table.getEntry("has_target").setBoolean(result.hasTargets());
        table.getEntry("tags").setIntegerArray(ids);
        table.getEntry("target_count").setInteger(targets.size());

        if (!result.hasTargets()) {
            // keep lastSeenId as-is (persist last detected)
            publishNoTarget();
            // update prevIds to empty for next comparison
            prevIds = new long[] {};
            return;
        }

        // Determine "last detected" semantics:
        // - If only one id is present, that's the lastSeenId.
        // - If multiple ids are present, prefer an id that newly appeared since prevIds.
        // - If none newly appeared, fall back to PhotonVision's best target.
        Set<Long> prevSet = new HashSet<>();
        for (long id : prevIds) {
            prevSet.add(id);
        }

        Set<Long> currSet = new HashSet<>();
        for (long id : ids) {
            currSet.add(id);
        }

        // If only one tag visible, make it the lastSeenId
        if (ids.length == 1) {
            lastSeenId = ids[0];
        } else {
            // Look for any id that is in current but not in previous (a newly detected tag)
            Long newlyDetected = null;
            for (long id : ids) {
                if (!prevSet.contains(id)) {
                    newlyDetected = id;
                    break;
                }
            }

            if (newlyDetected != null) {
                lastSeenId = newlyDetected;
            } else {
                // No newly detected tag. Use the best target from PhotonVision as a deterministic fallback.
                PhotonTrackedTarget best = result.getBestTarget();
                if (best != null) {
                    lastSeenId = best.getFiducialId();
                }
            }
        }

        // Publish the chosen lastSeenId
        table.getEntry("last_id").setInteger((int) lastSeenId);

        // Now publish telemetry for the "best" target used for other fields.
        // Use the PhotonVision best target for yaw/pitch/distance/ambiguity/confidence/cam_to_tag
        PhotonTrackedTarget bestForTelemetry = result.getBestTarget();
        if (bestForTelemetry != null) {
            table.getEntry("last_yaw").setDouble(r3(bestForTelemetry.getYaw()));
            table.getEntry("last_pitch").setDouble(r3(bestForTelemetry.getPitch()));

            double distance = bestForTelemetry.getBestCameraToTarget().getTranslation().getNorm();
            table.getEntry("last_distance").setDouble(r3(distance));

            double ambiguity = bestForTelemetry.getPoseAmbiguity();
            table.getEntry("ambiguity").setDouble(r3(ambiguity));

            double confidence = 1.0 / (1.0 + ambiguity);
            table.getEntry("confidence").setDouble(r3(confidence));

            Transform3d camToTag = bestForTelemetry.getBestCameraToTarget();
            double[] camToTagRaw = new double[] {
                camToTag.getX(),
                camToTag.getY(),
                camToTag.getZ(),
                camToTag.getRotation().getX(),
                camToTag.getRotation().getY(),
                camToTag.getRotation().getZ()
            };
            table.getEntry("cam_to_tag").setDoubleArray(r3(camToTagRaw));
        } else {
            // Clear telemetry if no best target
            table.getEntry("last_yaw").setDouble(0);
            table.getEntry("last_pitch").setDouble(0);
            table.getEntry("last_distance").setDouble(-1);
            table.getEntry("ambiguity").setDouble(1.0);
            table.getEntry("confidence").setDouble(0.0);
            table.getEntry("cam_to_tag").setDoubleArray(new double[] {});
        }

        double maxDistance = table.getEntry("tune/max_distance_m").getDouble(5.0);
        double maxAmbiguity = table.getEntry("tune/max_ambiguity").getDouble(0.25);
        double minConfidence = table.getEntry("tune/min_confidence").getDouble(0.5);

        boolean visionOK = result.hasTargets()
            && (bestForTelemetry != null)
            && bestForTelemetry.getBestCameraToTarget().getTranslation().getNorm() < maxDistance
            && bestForTelemetry.getPoseAmbiguity() < maxAmbiguity
            && (1.0 / (1.0 + bestForTelemetry.getPoseAmbiguity())) >= minConfidence;

        table.getEntry("vision_ok").setBoolean(visionOK);

        // Save current ids for next loop comparison
        prevIds = Arrays.copyOf(ids, ids.length);
    }

    public void publishPose(EstimatedRobotPose est) {
        Pose2d pose = est.estimatedPose.toPose2d();

        table.getEntry("robot_pose").setString(
            String.format("(%.2f, %.2f, %.1f°)",
                pose.getX(), pose.getY(), pose.getRotation().getDegrees())
        );

        table.getEntry("robot_pose_array").setDoubleArray(new double[] {
            r3(pose.getX()),
            r3(pose.getY()),
            r3(pose.getRotation().getDegrees())
        });

        table.getEntry("last_filter_reason").setString(est.strategy.toString());

        // Compute latency in milliseconds between now and the pose timestamp
        double latencyMs = (System.currentTimeMillis() / 1000.0 - est.timestampSeconds) * 1000.0;
        table.getEntry("latency_ms").setDouble(r3(latencyMs));
        table.getEntry("timestamp_seconds").setDouble(est.timestampSeconds);
    }
}
