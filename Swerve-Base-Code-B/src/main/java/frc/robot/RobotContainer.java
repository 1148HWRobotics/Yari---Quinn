package frc.robot;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private static final boolean isDrifting = false;
    private final PS4Controller driver = new PS4Controller(0);
    private final PS4Controller operator = new PS4Controller(1);
    /* Driver8Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver,
            XboxController.Button.kLeftBumper.value);

    private final JoystickButton intakeButton = new JoystickButton(driver, PS4Controller.Button.kL2.value);
    private final JoystickButton shootClose = new JoystickButton(driver, PS4Controller.Button.kR1.value);
    private final JoystickButton outTakeButton = new JoystickButton(driver, PS4Controller.Button.kL1.value);
    private final JoystickButton forceFire = new JoystickButton(driver, PS4Controller.Button.kCross.value);
    private final JoystickButton passButton = new JoystickButton(driver, PS4Controller.Button.kR2.value);
    private final JoystickButton shootFar = new JoystickButton(operator, PS4Controller.Button.kR1.value);
    private final JoystickButton shootVeryFar = new JoystickButton(operator, PS4Controller.Button.kR2.value);
    private final JoystickButton highPass = new JoystickButton(operator, PS4Controller.Button.kL1.value);
    private final JoystickButton forceFireOperator = new JoystickButton(operator, PS4Controller.Button.kCross.value);

    public static boolean isDrifting() {
        return isDrifting;
    }

    /* Subsystems */
    private final Swerve s_Swerve = Swerve.getInstance();
    private final Intake intake = Intake.getInstance();
    private final Carriage carriage = Carriage.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    // private final PoseEstimator poseEstimator = PoseEstimator.getInstance();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // s_Swerve.setDefaultCommand(
        // new TeleopSwerve(
        // s_Swerve,
        // () -> -driver.getRawAxis(translationAxis),
        // () -> -driver.getRawAxis(strafeAxis),
        // () -> -driver.getRawAxis(rotationAxis),
        // () -> robotCentric.getAsBoolean()));
        s_Swerve.setDefaultCommand(new TeleopSwerve(
                s_Swerve,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX(), robotCentric));

        AutoBuilder.configureHolonomic(s_Swerve::getPose,
                s_Swerve::setPose,
                s_Swerve::getRobotVelocity,
                s_Swerve::fromChassisSpeeds, Constants.AutoConstants.getPathFollowerConfig(), RobotContainer::getIsRed,
                s_Swerve);
        NamedCommands.registerCommand("shoot", new AutoShootCommand().withTimeout(4));
        SmartDashboard.putData("zero heading", new Command() {
            @Override
            public void initialize() {
                Swerve.getInstance().zeroHeading();
            }
        });

        // Configure the button bindings
        configureButtonBindings();
        Constants.AutoConstants.configureAutos();
        SmartDashboard.putData(Constants.AutoConstants.getAutoChooser());
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        intakeButton.whileTrue(new IntakeCommand()).onFalse(new InstantCommand(() -> {
            Intake.getInstance().stop();
            Carriage.getInstance().stop();
        }));
        outTakeButton.whileTrue(new OutTakeCommand());
        shootClose.whileTrue(new ShooterPresetCommand(14));
        forceFire.whileTrue(new InstantCommand(() -> {
            Intake.getInstance().setIntakeSpeed(0.5);
            Carriage.getInstance().setVelocity(1);
        })).onFalse(new InstantCommand(() -> {
            Intake.getInstance().stop();
            Carriage.getInstance().stop();
        }));
        passButton.whileTrue(new InstantCommand(() -> {
            Shooter.getInstance().setAngleTarget(3);
            Shooter.getInstance().setVelocity(0.9999);
        })).onFalse(new InstantCommand(() -> {
            Shooter.getInstance().setAngleTarget(1.5);
            Shooter.getInstance().stop();
        }));
        highPass.whileTrue(new InstantCommand(() -> {
            Shooter.getInstance().setAngleTarget(13);
            Shooter.getInstance().setVelocity(0.55);
        })).onFalse(new InstantCommand(() -> {
            Shooter.getInstance().setAngleTarget(1.5);
            Shooter.getInstance().stop();
        }));
        forceFireOperator.whileTrue(new InstantCommand(() -> {
            Intake.getInstance().setIntakeSpeed(0.5);
            Carriage.getInstance().setVelocity(1);
        })).onFalse(new InstantCommand(() -> {
            Intake.getInstance().stop();
            Carriage.getInstance().stop();
        }));
        shootFar.whileTrue(new ShooterPresetCommand(9.5));
        shootVeryFar.whileTrue(new ShooterPresetCommand(6.3));

        // isDrifting.onTrue(new InstantCommand((() -> s_Swerve.isDrifting =
        // !s_Swerve.isDrifting)));
    }

    public static boolean getIsRed() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new PathPlannerAuto(Constants.AutoConstants.getAutoChooser().getSelected());
    }
}
