package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import frc.lib.math.Conversions;
import frc.robot.Constants;

public class ShotCalculator {
    // Constants
    private static final double g = 9.81; // Gravity (m/s^2)
    private static final double L_SHOOTER = 0.5; // Length of the shooter arm (meters)
    private static final double H_PIVOT = 0.6; // Height of the shooter pivot point (meters)
    private static final double SHOOTER_VELOCITY_MAX = 20.0; // Max shooter velocity (m/s)
    private static final double THETA_MIN = Units.degreesToRadians(Constants.Swerve.Shooter.downAngle); // Min shooter
                                                                                                        // angle
                                                                                                        // (radians)
    private static final double THETA_MAX = Units.degreesToRadians(Constants.Swerve.Shooter.upAngle); // Max shooter
                                                                                                      // angle (radians)
    private static final double ROBOT_LENGTH = 0.9; // Robot length (meters)
    private static final double ROBOT_WIDTH = 0.7; // Robot width (meters)

    // Target (Speaker) Properties
    private static final double X_TARGET = 10.0; // Target x-position (meters)
    private static final double Y_TARGET_MIN = 1.9812; // Lower edge of target (meters)
    private static final double Y_TARGET_MAX = 2.1128; // Upper edge of target (meters)
    private static final double Y_TARGET = (Y_TARGET_MIN + Y_TARGET_MAX) / 2.0; // Average y-position

    // Method to calculate the shooter parameters using exact equations
    public static ShooterSolution calculateOptimalShooterParameters(Pose2d robotPose) {
        // Convert yaw to radians
        double yawRad = robotPose.getRotation().getRadians();

        // Calculate launch position based on shooter angle and robot position
        double xLaunch = robotPose.getX() + L_SHOOTER * Math.cos(yawRad);
        double yLaunch = robotPose.getY() + L_SHOOTER * Math.sin(yawRad) + H_PIVOT;

        // Horizontal and vertical distances to target
        double dx = X_TARGET - xLaunch;
        double dy = Y_TARGET - yLaunch;

        // Check if the target is reachable horizontally
        if (dx <= 0) {
            return null;
        }

        // Calculate optimal angle
        double optimalTheta = 0.9 * Math.atan2(dy, dx + Constants.Swerve.trackWidth / 2) + (0.1 / 2 * Math.PI) * dy;

        // Ensure the angle is within physical constraints
        if (optimalTheta < THETA_MIN || optimalTheta > THETA_MAX) {
            return null;
        }

        // Calculate required initial velocity using the exact formula
        double cosTheta = Math.cos(optimalTheta);
        double tanTheta = Math.tan(optimalTheta);

        double denominator = 2 * (dx * tanTheta - dy);
        if (denominator <= 0) {
            return null; // Invalid trajectory
        }

        double v0 = (dx / cosTheta) * Math.sqrt(g / denominator);

        // Check if the required velocity is within the shooter's capabilities
        if (v0 > SHOOTER_VELOCITY_MAX) {
            return null;
        }

        // Return the solution
        return new ShooterSolution(v0, Math.toDegrees(optimalTheta));
    }

    // Method to check if the shot is feasible
    public static boolean isShotFeasible(Pose2d robotPose) {
        ShooterSolution solution = calculateOptimalShooterParameters(robotPose);
        return solution != null;
    }

    // Class to hold the shooter solution
    public static class ShooterSolution {
        public double velocity;
        public double angle;

        public ShooterSolution(double velocity, double angle) {
            this.velocity = velocity;
            this.angle = angle;
        }
    }
}