package frc.robot;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Vision.VisionMeasurement;
import frc.robot.subsystems.Swerve;

public class PoseEstimator extends SubsystemBase {
    private static PoseEstimator instance;
    private SwerveDrivePoseEstimator poseEstimator;
    private Vision vision;

    public static PoseEstimator getInstance() {
        if (instance == null) {
            instance = new PoseEstimator();
        }
        return instance;
    }

    private PoseEstimator() {
        this.poseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics,
                Swerve.getInstance().getGyroYaw(), Swerve.getInstance().getModulePositions(), new Pose2d());
        this.vision = new Vision();
    }

    public void resetPose() {
        poseEstimator.resetPosition(new Rotation2d(), Swerve.getInstance().getModulePositions(), new Pose2d());
    }

    public void setPose(Rotation2d rotation, SwerveModulePosition[] states, Pose2d pose) {
        poseEstimator.resetPosition(rotation, states, pose);
    }

    public void update() {
        poseEstimator.update(Swerve.getInstance().getGyroYaw(), Swerve.getInstance().getModulePositions());
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    @Override
    public void periodic() {
        update();
        for (VisionMeasurement measurement : vision.getVisionMeasurements()) {
            poseEstimator.setVisionMeasurementStdDevs(measurement.stdDevs);
            poseEstimator.addVisionMeasurement(measurement.estimatedPose, measurement.timestamp);
        }
    }
}
