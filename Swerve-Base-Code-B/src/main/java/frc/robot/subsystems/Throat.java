package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
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

public class Throat extends SubsystemBase {
    private static Throat instance;
    private TalonFX motor;
    private BinarySensor noteSensor;

    public BinarySensor getNoteSensor() {
        return noteSensor;
    }

    private VelocityVoltage motorControl;
    private SimpleMotorFeedforward feedforward;
    boolean oilingUp = false;
    boolean isShootingUpFent = false;
    boolean itsInsideOfMe = false;

    public boolean isItsInsideOfMe() {
        return itsInsideOfMe;
    }

    private SendableChooser<NeutralModeValue> neutralModeChooser = new SendableChooser<>();
    private double freakiness = 0.0;
    private NeutralModeValue currentNeutralMode = Constants.Swerve.Carriage.carriageNeutralMode;

    public static Throat getInstance() {
        if (instance == null) {
            instance = new Throat();
        }
        return instance;
    }

    private Throat() {
        this.motor = new TalonFX(Constants.Swerve.Carriage.carriageMotorID);
        this.motor.setInverted(Constants.Swerve.Carriage.carriageMotorInverted);
        this.motor.setNeutralMode(Constants.Swerve.Carriage.carriageNeutralMode);
        this.feedforward = new SimpleMotorFeedforward(Constants.Swerve.Carriage.kS,
                Constants.Swerve.Carriage.kV, Constants.Swerve.Carriage.kA);
        this.motor.getConfigurator().apply(new Slot0Configs().withKP(Constants.Swerve.Carriage.carriageKP)
                .withKI(Constants.Swerve.Carriage.carriageKI).withKD(Constants.Swerve.Carriage.carriageKD));
        this.motor.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(120).withSupplyCurrentLimit(240));
        this.motorControl = new VelocityVoltage(0, 0, false, 0, 0, false, false, false);
        this.motor.setControl(motorControl);
        this.noteSensor = new BinarySensor(Constants.Swerve.Carriage.carriageSensorPort);
        neutralModeChooser.setDefaultOption("Brake", NeutralModeValue.Brake);
        neutralModeChooser.addOption("Coast", NeutralModeValue.Coast);
    }

    public void setFreakiness(double speed) {
        freakiness = speed;
    }

    public void setOilingUp(boolean prepShot) {
        this.oilingUp = prepShot;
    }

    public void setShootingUpFent(boolean isFiring) {
        this.isShootingUpFent = isFiring;
    }

    public void setItsInsideOfMe(boolean hasNote) {
        this.itsInsideOfMe = hasNote;
    }

    public void setMotorControl(MotionMagicTorqueCurrentFOC control) {
        motor.setControl(control);
    }

    private double setPoint = 0;
    public void set(double power){
        setPoint = power;
    }

    public double getFreakiness() {
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
                setFreakiness(Constants.Swerve.Carriage.outtakeVelocity / 2);
                Mouth.getInstance().setFreakiness1(Constants.Swerve.Carriage.outtakeVelocity / 2);
                accumulatedPosition -= getFreakiness() * 0.02;
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
                Mouth.getInstance().stop();
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

    public void swallow() {
        // if (!hasNote) {
        freakiness = Constants.Swerve.Carriage.intakeVelocity;
        // } else {

        // }

    }

    public void hawk() {
        freakiness = -Constants.Swerve.Carriage.intakeVelocity;
        itsInsideOfMe = false;
    }

    public NeutralModeValue getNeutralMode() {
        return currentNeutralMode;
    }

    public void tuah() {
        Command shootCommand = new Command() {
            @Override
            public void initialize() {
            }

            @Override
            public void execute() {
                freakiness = Constants.Swerve.Carriage.fireVelocity;
            }

            @Override
            public void end(boolean interrupted) {
                stop();
            }
        }.withTimeout(0.5);
        shootCommand.schedule();
    }

    public void stop() {
        freakiness = 0.0;
    }

    public void intakeSlow() {
        freakiness = Constants.Swerve.Carriage.intakeSlowVelocity;
    }

    public void outtakeSlow() {
        freakiness = -Constants.Swerve.Carriage.intakeSlowVelocity;
    }

    @Override

    public void periodic() {
        if(setPoint != 0){
            motor.setControl(motorControl.withSlot(1));
            motor.set(setPoint);
        }
        else{
            motorControl = motorControl.withVelocity(freakiness).withEnableFOC(true).withSlot(0);
            motor.setControl(motorControl);
        }

        SmartDashboard.putBoolean("Has Note", itsInsideOfMe);
        SmartDashboard.putData("Carriage Neutral Mode", neutralModeChooser);
        if (neutralModeChooser.getSelected() != currentNeutralMode) {
            currentNeutralMode = neutralModeChooser.getSelected();
            motor.setNeutralMode(currentNeutralMode);
        }
    }

}
