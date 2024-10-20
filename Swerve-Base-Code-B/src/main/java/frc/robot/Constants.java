package frc.robot;

import java.util.List;
import java.util.Map;
import java.util.TreeMap;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 18;
        public static final String shooterLimeLightID = "limelight-a";
        public static final String intakeLimeLightID = "limelight-b";

        public static final COTSTalonFXSwerveConstants chosenModule = // TODO: This must be tuned to specific
                                                                      // robot
                COTSTalonFXSwerveConstants.SDS.MK4i
                        .Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L3);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21.25); // TODO: This must be tuned to
                                                                             // specific
                                                                             // robot
        public static final double wheelBase = Units.inchesToMeters(21.25); // TODO: This must be tuned to
                                                                            // specific
                                                                            // robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 35;
        public static final int angleCurrentThreshold = 80;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 80;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 1.5; // TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.00;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; // TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 5000.3; // TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10000.0; // TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Coast;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { // Front Left Module
            public static final int driveMotorID = 4; // Left Front Go motor ID
            public static final int angleMotorID = 3; // Left Front Turn motor ID
            public static final int canCoderID = 12; // Left Front Encoder CAN ID, assuming it acts as the
                                                     // canCoder for
                                                     // this module
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.466797); // Adjusted to
            // match the
            // left
            // front encoder
            // offset
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                    angleMotorID,
                    canCoderID, angleOffset);
        }

        public static final class Mod1 { // Front Right Module
            public static final int driveMotorID = 6; // Right Front Go motor ID
            public static final int angleMotorID = 5; // Right Front Turn motor ID
            public static final int canCoderID = 13; // Right Front Encoder CAN ID, assuming it acts as the
                                                     // canCoder for
                                                     // this module
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.467285); // Adjusted to
            // match the
            // right front
            // encoder
            // offset
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                    angleMotorID,
                    canCoderID, angleOffset);
        }

        public static final class Mod2 { // Back Left Module
            public static final int driveMotorID = 2; // Left Back Go motor ID
            public static final int angleMotorID = 1; // Left Back Turn motor ID
            public static final int canCoderID = 11; // Left Back Encoder CAN ID, assuming it acts as the
                                                     // canCoder for
                                                     // this module
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.316162); // Adjusted to
            // match the
            // left back
            // encoder offset
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                    angleMotorID,
                    canCoderID, angleOffset);
        }

        public static final class Mod3 { // Back Right Module
            public static final int driveMotorID = 8; // Right Back Go motor ID
            public static final int angleMotorID = 7; // Right Back Turn motor ID
            public static final int canCoderID = 14; // Right Back Encoder CAN ID, assuming it acts as the
                                                     // canCoder for
                                                     // this module
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.254150);
            // right
            // back encoder
            // offset
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                    angleMotorID,
                    canCoderID, angleOffset);
        }

        public static final class Intake { // Intake
            public static final int intakeMotorID = 21; // Intake motor ID
            public static final boolean intakeMotorInverted = true; // Intake motor inverted
            public static final double kS = 0.1; // Intake kS
            public static final double kV = 0.05; // Intake kV
            public static final double kA = 0.01; // Intake kA
            public static final NeutralModeValue intakeNeutralMode = NeutralModeValue.Brake; // Intake neutral mode
            public static final double intakeKP = 0.5; // Intake kP
            public static final double intakeKI = 0.0; // Intake kI
            public static final double intakeKD = 0.01; // Intake kD
            public static final double intakeVelocity = 0.5; // Intake velocity
            public static final double outtakeVelocity = -70; // Intake velocity
        }

        public static final class Carriage { // Carriage
            public static final int carriageMotorID = 20; // Carriage motor ID
            public static final int carriageSensorPort = 9; // Carriage sensor ID
            public static final boolean carriageMotorInverted = false; // Carriage motor inverted
            public static final double kS = 0.15; // Carriage kS
            public static final double kV = 0.08; // Carriage kV
            public static final double kA = 0.02; // Carriage kA
            public static final NeutralModeValue carriageNeutralMode = NeutralModeValue.Brake; // Carriage neutral mode
            public static final double carriageKP = 0.5; // Carriage kP
            public static final double carriageKI = 0.0; // Carriage kI
            public static final double carriageKD = 0.02; // Carriage kD

            public static final double intakeVelocity = 0.5; // Carriage intake velocity
            public static final double outtakeVelocity = -70; // Carriage outtake velocity
            public static final double prepShotVelocity = 0.3; // Carriage prep shot velocity
            public static final double fireVelocity = 100.0; // Carriage fire velocity
            public static final double intakeSlowVelocity = 30.0; // Carriage intake slow velocity
        }

        public static final class Shooter { // Shooter
            public static final int angleMotor1ID = 24; // Angling motor 1 ID
            public static final boolean angleMotor1Inverted = false; // Angling motor 1 inverted
            public static final int angleMotor2ID = 26; // Angling motor 2 ID
            public static final boolean angleMotor2Inverted = true; // Angling motor 2 inverted
            public static final int fireMotor1ID = 22; // Fire motor 1 ID
            public static final boolean fireMotor1Inverted = false; // Fire motor 1 inverted
            public static final int fireMotor2ID = 23; // Fire motor 2 ID
            public static final boolean fireMotor2Inverted = true; // Fire motor 2 inverted
            public static final double anglekS = 0.0; // Angle kS
            public static final double anglekG = 0.0; // Angle kG
            public static final double anglekV = 0.0; // Angle kV
            public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Brake; // Angle neutral mode
            public static final double angleKP = 1.3; // Angle kP
            public static final double angleKI = 0.0; // Angle kI
            public static final double angleKD = 0.00; // Angle kD

            public static final double shootkS = 0.0; // Shoot kS
            public static final double shootkV = 0.0; // Shoot kV
            public static final double shootkA = 0.00; // Shoot kA
            public static final NeutralModeValue fireNeutralMode = NeutralModeValue.Coast; // Shoot neutral mode
            public static final double shootKP = 0.0; // Shoot kP
            public static final double shootKI = 0.0; // Shoot kI
            public static final double shootKD = 0.00; // Shoot kD

            public static final double shootVelocity = 85.0; // Shoot velocity (RPM)
            public static final double passVelocity = 6.0; // Pass velocity (RPM)
            public static final double outtakeVelocity = -100.0; // Outtake velocity (RPM)
            public static final double ampVelocity = 20; // Amp velocity (RPM)

            public static final double downAngle = 0; // Down angle (degrees)
            public static final double upAngle = 17; // Up angle (degrees)

            public static final int angleSensorPort = 7; // Angle sensor port

            public static class state {
                public double angle;

                public double speed_l;
                public double speed_r;

                public state(double a, double l, double r) {
                    angle = a;
                    speed_l = l;
                    speed_r = r;
                }
            }

            public static final double rollerSpeed = 100;
            public static final double rollerSpeedA = 100;

            public static final double globalOffset = -0.05;

            public static final TreeMap<Double, state> distToState = new TreeMap<Double, state>() {
                {
                    put(1.5, new state(17.30 + globalOffset, 100, 100));
                    put(2.0, new state(16.30 + globalOffset, 100, 100));
                    put(2.5, new state(15.40 + globalOffset, 100, 100));
                    put(3.0, new state(14.60 + globalOffset, rollerSpeed, rollerSpeedA));
                    put(3.5, new state(13.30 + globalOffset, rollerSpeed, rollerSpeedA));
                    put(4.0, new state(12.90 + globalOffset, rollerSpeed, rollerSpeedA));
                    put(4.5, new state(10.65 + globalOffset, rollerSpeed, rollerSpeedA));
                    put(5.0, new state(8.40 + globalOffset, rollerSpeed, rollerSpeedA));
                    put(5.5, new state(6.30 + globalOffset, rollerSpeed, rollerSpeedA));
                    put(6.0, new state(4.30 + globalOffset, rollerSpeed, rollerSpeedA));

                    put(12., new state(0.4, rollerSpeed, rollerSpeedA));
                }
            };

            public static final state adjustedState(double dist) {
                dist = Util.clamp(dist, Constants.Swerve.Shooter.distToState.firstKey() + 0.00001,
                        Constants.Swerve.Shooter.distToState.lastKey() - 0.00001);
                Map.Entry<Double, state> lower = Constants.Swerve.Shooter.distToState.floorEntry(dist);
                Map.Entry<Double, state> higher = Constants.Swerve.Shooter.distToState.ceilingEntry(dist);

                double t = (dist - lower.getKey()) / (higher.getKey() - lower.getKey());

                return new state(
                        Util.lerp(t, lower.getValue().angle, higher.getValue().angle),
                        Util.lerp(t, lower.getValue().speed_l, higher.getValue().speed_l),
                        Util.lerp(t, lower.getValue().speed_r, higher.getValue().speed_r));
            }

        }
    }

    public static final class Field {
        public static final class RED {
            public static final Translation2d Speaker = new Translation2d(16.57, 5.54);
            public static final Pose2d Amp = new Pose2d(14.7, 7.8, new Rotation2d(Math.PI / 2));
            public static final Pose2d Source = new Pose2d(1, 0.5, Rotation2d.fromDegrees(-135));
            public static final Translation2d Corner = new Translation2d(14.57, 7.);

        }

        public static final class BLUE {
            public static final Translation2d Speaker = new Translation2d(-0.04, 5.54);
            public static final Pose2d Amp = new Pose2d(1.7, 7.8, new Rotation2d(Math.PI / 2));
            public static final Pose2d Source = new Pose2d(15.15, 1.5, Rotation2d.fromDegrees(135));
            public static final Translation2d Corner = new Translation2d(2., 7.);

        }
    }

    public static final class AutoConstants { // TODO: The below constants are used in the example auto, and must be
                                              // tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 5;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        private static PIDConstants translationConstants = new PIDConstants(1.1, 0.0, 0.0);
        private static PIDConstants rotationConstants = new PIDConstants(1.9, 0, 0.0);
        private static ReplanningConfig replanningConfig = new ReplanningConfig();
        private static SendableChooser<String> autoChooser = new SendableChooser<String>();

        public static SendableChooser<String> getAutoChooser() {
            return autoChooser;
        }

        public static void configureAutos() {
            autoChooser.setDefaultOption("Source Side", "Source Side");
            autoChooser.addOption("Amp Side", "Splean Time");
            autoChooser.addOption("Source Side", "Source Side");

        }

        public static HolonomicPathFollowerConfig getPathFollowerConfig() {
            return new HolonomicPathFollowerConfig(translationConstants, rotationConstants,
                    Constants.Swerve.maxSpeed,
                    Constants.Swerve.trackWidth / 2.0, replanningConfig);
        }

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class Vision {

        public static final class CameraConfig {
            public final String name;
            public final Transform3d robotToCamera;

            public CameraConfig(String name, Transform3d robotToCamera) {
                this.name = name;
                this.robotToCamera = robotToCamera;
            }
        }

        public static final List<CameraConfig> CAMERAS = List.of(
                new CameraConfig("Camera1", new Transform3d(
                        new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0))),
                new CameraConfig("Camera2", new Transform3d(
                        new Translation3d(-0.5, 0.0, 0.5), new Rotation3d(0, Math.PI, 0))),
                new CameraConfig("Camera3", new Transform3d(
                        new Translation3d(0.0, 0.5, 0.5), new Rotation3d(0, Math.PI / 2, 0))),
                new CameraConfig("Camera4", new Transform3d(
                        new Translation3d(0.0, -0.5, 0.5), new Rotation3d(0, -Math.PI / 2, 0))),
                new CameraConfig("Camera5", new Transform3d(
                        new Translation3d(0.0, 0.0, 1.0), new Rotation3d(-Math.PI / 2, 0, 0))),
                new CameraConfig("Camera6", new Transform3d(
                        new Translation3d(0.0, 0.0, 0.0), new Rotation3d(Math.PI / 2, 0, 0))));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

        // The standard deviations of our vision estimated poses, which affect
        // correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }
}
