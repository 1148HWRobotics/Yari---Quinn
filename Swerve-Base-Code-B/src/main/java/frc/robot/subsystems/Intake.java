package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.BinarySensor;

public class Intake extends SubsystemBase {
    private static Intake instance;
    private TalonFX intakeMotor1;
    private TalonFX intakeMotor2;
    private double velocity1 = 0.0;
    private double velocity2 = 0.0;
    private final SimpleMotorFeedforward feedForward;
    private MotionMagicVelocityVoltage intakeControl;

    public double getIntakeSpeed() {
        return intakeMotor1.getVelocity().getValueAsDouble();
    }

    public void setIntakeSpeed(double intakeSpeed) {
        this.setVelocity1(intakeSpeed);
    }

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    private Intake() {
        this.intakeMotor1 = new TalonFX(Constants.Swerve.Intake.intakeMotorLowID);
        this.intakeMotor2 = new TalonFX(Constants.Swerve.Intake.intakeMotorHighID);
        this.intakeMotor1.getConfigurator().apply(new Slot0Configs().withKP(Constants.Swerve.Intake.intakeKP)
                .withKI(Constants.Swerve.Intake.intakeKI).withKD(Constants.Swerve.Intake.intakeKD));
        this.intakeMotor2.getConfigurator().apply(new Slot0Configs().withKP(Constants.Swerve.Intake.intakeKP)
                .withKI(Constants.Swerve.Intake.intakeKI).withKD(Constants.Swerve.Intake.intakeKD));
        this.intakeMotor1.setInverted(Constants.Swerve.Intake.intakeMotorInverted);
        this.intakeMotor2.setInverted(Constants.Swerve.Intake.intakeMotorInverted2);
        this.feedForward = new SimpleMotorFeedforward(Constants.Swerve.Intake.kS, Constants.Swerve.Intake.kV,
                Constants.Swerve.Intake.kA);
        this.intakeMotor1.setNeutralMode(Constants.Swerve.Intake.intakeNeutralMode);
        this.intakeControl = new MotionMagicVelocityVoltage(0, 0, true, 0, 0, false, false, false);
        this.intakeMotor1.setControl(intakeControl);
        this.intakeMotor2.setControl(intakeControl.clone());

    }

    public void setVelocity1(double speed) {
        velocity1 = speed;
        velocity2 = speed;
    }

    public void intake() {
        velocity1 = 1;
        velocity2 = 0.5;
    }

    public void outtake() {
        velocity1 = Constants.Swerve.Intake.outtakeVelocity;
        velocity2 = Constants.Swerve.Intake.outtakeVelocity;

    }

    public void stop() {
        velocity2 = 0.0;
        velocity1 = 0.0;
    }

    @Override
    public void periodic() {
        intakeControl = new MotionMagicVelocityVoltage(velocity1, 0.0, true,
                feedForward.calculate(velocity1), 0, false, true, false);
        this.intakeMotor1.setControl(intakeControl);
        this.intakeMotor2.setControl(intakeControl);
        this.intakeMotor1.set(velocity1);
        this.intakeMotor2.set(velocity2);
    }
}
