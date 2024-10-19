package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.BinarySensor;

public class Carriage extends SubsystemBase {
    private static Carriage instance;
    private TalonFX motor;
    private BinarySensor noteSensor;

    public BinarySensor getNoteSensor() {
        return noteSensor;
    }

    private MotionMagicVelocityTorqueCurrentFOC motorControl;
    private SimpleMotorFeedforward feedforward;
    boolean prepShot = false;
    boolean isFiring = false;
    boolean hasNote = false;

    public boolean isHasNote() {
        return hasNote;
    }

    private SendableChooser<NeutralModeValue> neutralModeChooser = new SendableChooser<>();
    private double velocity = 0.0;
    private NeutralModeValue currentNeutralMode = Constants.Swerve.Carriage.carriageNeutralMode;

    public static Carriage getInstance() {
        if (instance == null) {
            instance = new Carriage();
        }
        return instance;
    }

    private Carriage() {
        this.motor = new TalonFX(Constants.Swerve.Carriage.carriageMotorID);
        this.motor.setInverted(Constants.Swerve.Carriage.carriageMotorInverted);
        this.motor.setNeutralMode(Constants.Swerve.Carriage.carriageNeutralMode);
        this.feedforward = new SimpleMotorFeedforward(Constants.Swerve.Carriage.kS,
                Constants.Swerve.Carriage.kV, Constants.Swerve.Carriage.kA);
        this.motor.getConfigurator().apply(new Slot0Configs().withKP(Constants.Swerve.Carriage.carriageKP)
                .withKI(Constants.Swerve.Carriage.carriageKI).withKD(Constants.Swerve.Carriage.carriageKD));
        this.motorControl = new MotionMagicVelocityTorqueCurrentFOC(0, 0, false, 0, 0, false, false, false);
        this.motor.setControl(motorControl);
        this.noteSensor = new BinarySensor(Constants.Swerve.Carriage.carriageSensorPort);
        neutralModeChooser.setDefaultOption("Brake", NeutralModeValue.Brake);
        neutralModeChooser.addOption("Coast", NeutralModeValue.Coast);
    }

    public void setVelocity(double speed) {
        velocity = speed;
    }

    public void setPrepShot(boolean prepShot) {
        this.prepShot = prepShot;
    }

    public void setFiring(boolean isFiring) {
        this.isFiring = isFiring;
    }

    public void setHasNote(boolean hasNote) {
        this.hasNote = hasNote;
    }

    public void setMotorControl(MotionMagicTorqueCurrentFOC control) {
        motor.setControl(control);
    }

    public double getVelocity() {
        return motor.getVelocity().getValueAsDouble();
    }

    public void secureNote(double originalPosition) {
        Command secureCommand = new Command() {
            @Override
            public void initialize() {

            }

            double accumulatedPosition = 0.0;

            @Override
            public void execute() {
                setVelocity(Constants.Swerve.Carriage.outtakeVelocity / 2);
                Intake.getInstance().setVelocity(Constants.Swerve.Carriage.outtakeVelocity / 2);
                accumulatedPosition -= getVelocity() * 0.02;
                if (originalPosition - accumulatedPosition <= 0) {
                    cancel();
                }
            }

            @Override
            public boolean isFinished() {
                if (originalPosition - accumulatedPosition <= 0) {
                    return true;
                } else {
                    return false;
                }
            }

            @Override
            public void end(boolean interrupted) {
                Intake.getInstance().stop();
                stop();
            }
        }.withTimeout(0.5);
        secureCommand.schedule();
    }

    public void resetMotor() {
        motor.setPosition(0);
    }

    public double getEncoderValue() {
        return motor.getPosition().getValueAsDouble();
    }

    public void intake() {
        // if (!hasNote) {
        velocity = Constants.Swerve.Carriage.intakeVelocity;
        // } else {

        // }

    }

    public void outtake() {
        velocity = -Constants.Swerve.Carriage.intakeVelocity;
        hasNote = false;
    }

    public NeutralModeValue getNeutralMode() {
        return currentNeutralMode;
    }

    public void shoot() {
        Command shootCommand = new Command() {
            @Override
            public void initialize() {
            }

            @Override
            public void execute() {
                velocity = Constants.Swerve.Carriage.fireVelocity;
            }

            @Override
            public void end(boolean interrupted) {
                stop();
            }
        }.withTimeout(0.5);
        shootCommand.schedule();
    }

    public void stop() {
        velocity = 0.0;
    }

    public void intakeSlow() {
        velocity = Constants.Swerve.Carriage.intakeSlowVelocity;
    }

    public void outtakeSlow() {
        velocity = -Constants.Swerve.Carriage.intakeSlowVelocity;
    }

    @Override

    public void periodic() {
        motorControl = new MotionMagicVelocityTorqueCurrentFOC(velocity, 0.0, true, feedforward.calculate(velocity), 0,
                false, false, false);
        motor.setControl(motorControl);
        motor.set(velocity);

        SmartDashboard.putBoolean("Has Note", hasNote);
        SmartDashboard.putData("Carriage Neutral Mode", neutralModeChooser);
        if (neutralModeChooser.getSelected() != currentNeutralMode) {
            currentNeutralMode = neutralModeChooser.getSelected();
            motor.setNeutralMode(currentNeutralMode);
        }
    }

}
