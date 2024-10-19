package frc.robot;

import static frc.robot.Constants.Vision.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.util.FieldData;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class Vision {
    private final List<PhotonCamera> cameras = new ArrayList<>();
    private final List<PhotonPoseEstimator> photonEstimators = new ArrayList<>();
    private final List<Double> lastEstTimestamps = new ArrayList<>();

    public Vision() {
        for (CameraConfig config : CAMERAS) {
            PhotonCamera camera = new PhotonCamera(config.name);
            PhotonPoseEstimator estimator = new PhotonPoseEstimator(
                    kTagLayout,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    camera,
                    config.robotToCamera);
            estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
            cameras.add(camera);
            photonEstimators.add(estimator);
            lastEstTimestamps.add(0.0);
        }
    }

    /**
     * Returns a list of vision measurements from all cameras that can be used with
     * a SwerveDrivePoseEstimator. Only includes measurements where AprilTags are
     * detected.
     * During autonomous, discards measurements where the closest tag is more than
     * 0.4 meters away.
     *
     * @return A list of VisionMeasurement instances containing estimated poses and
     *         standard deviations.
     */
    public List<VisionMeasurement> getVisionMeasurements() {
        List<VisionMeasurement> measurements = new ArrayList<>();
        for (int i = 0; i < photonEstimators.size(); i++) {
            PhotonPoseEstimator estimator = photonEstimators.get(i);
            Optional<EstimatedRobotPose> visionEst = estimator.update();
            if (visionEst.isPresent()) {
                double latestTimestamp = cameras.get(i).getLatestResult().getTimestampSeconds();
                boolean newResult = Math.abs(latestTimestamp - lastEstTimestamps.get(i)) > 1e-5;
                if (newResult) {
                    lastEstTimestamps.set(i, latestTimestamp);

                    EstimatedRobotPose estimatedPose = visionEst.get();
                    PhotonPipelineResult result = cameras.get(i).getLatestResult();
                    Matrix<N3, N1> stdDevs = getEstimationStdDevs(result, estimatedPose);

                    // Check if we need to discard the measurement during autonomous
                    boolean discardMeasurement = false;
                    if (FieldData.getIsAuto()) {
                        double minDist = Double.MAX_VALUE;
                        for (var target : result.getTargets()) {
                            var tagPoseOptional = kTagLayout.getTagPose(target.getFiducialId());
                            if (tagPoseOptional.isEmpty())
                                continue;
                            var tagPose = tagPoseOptional.get().toPose2d();
                            double distance = tagPose.getTranslation().getDistance(
                                    estimatedPose.estimatedPose.toPose2d().getTranslation());
                            if (distance < minDist) {
                                minDist = distance;
                            }
                        }
                        if (minDist > 0.4) {
                            discardMeasurement = true;
                        }
                    }

                    if (!discardMeasurement) {
                        measurements.add(new VisionMeasurement(
                                estimatedPose.estimatedPose.toPose2d(), stdDevs, latestTimestamp));
                    }
                }
            }
        }
        return measurements;
    }

    /**
     * Calculates the standard deviations for an estimated pose based on the number
     * of visible targets
     * and their average distance. This helps in adjusting the confidence level of
     * the measurements.
     *
     * @param result        The latest pipeline result from the camera.
     * @param estimatedPose The estimated robot pose.
     * @return A Matrix containing the standard deviations.
     */
    public Matrix<N3, N1> getEstimationStdDevs(PhotonPipelineResult result, EstimatedRobotPose estimatedPose) {
        var estStdDevs = kSingleTagStdDevs;
        var targets = result.getTargets();
        int numTags = 0;
        double avgDist = 0;

        for (var tgt : targets) {
            var tagPose = kTagLayout.getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty())
                continue;
            numTags++;
            avgDist += tagPose.get()
                    .toPose2d()
                    .getTranslation()
                    .getDistance(estimatedPose.estimatedPose.toPose2d().getTranslation());
        }

        if (numTags == 0)
            return estStdDevs;
        avgDist /= numTags;

        // Adjust standard deviations based on the number of tags and average distance
        if (numTags > 1) {
            estStdDevs = kMultiTagStdDevs;
        }
        if (numTags == 1 && avgDist > 4) {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        }

        return estStdDevs;
    }

    /**
     * A helper class to hold the estimated robot pose and its associated standard
     * deviations.
     */
    public static class VisionMeasurement {
        public final Pose2d estimatedPose;
        public final Matrix<N3, N1> stdDevs;
        public final double timestamp;

        public VisionMeasurement(Pose2d estimatedPose, Matrix<N3, N1> stdDevs, double timestamp) {
            this.estimatedPose = estimatedPose;
            this.stdDevs = stdDevs;
            this.timestamp = timestamp;
        }
    }
}