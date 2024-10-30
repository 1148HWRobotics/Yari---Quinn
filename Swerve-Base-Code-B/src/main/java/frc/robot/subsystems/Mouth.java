package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.BinarySensor;

public class Mouth extends SubsystemBase {
    private static Mouth instance;
    private TalonFX intakeMotor1;
    private TalonFX intakeMotor2;
    private double freakiness1 = 0.0;
    private double freakiness2 = 0.0;
    private final SimpleMotorFeedforward feedForward;
    private VelocityVoltage intakeControl1;
    private VelocityVoltage intakeControl2;

    public double getFreaky() {
        return intakeMotor1.getVelocity().getValueAsDouble();
    }

    public void setFreaky(double intakeSpeed) {
        this.setFreakiness1(intakeSpeed);
    }

    public static Mouth getInstance() {
        if (instance == null) {
            instance = new Mouth();
        }
        return instance;
    }

    private Mouth() {
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
        this.intakeControl1 = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
        this.intakeControl2 = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
        this.intakeMotor1.setControl(intakeControl1);
        this.intakeMotor2.setControl(intakeControl2);

    }

    public void setFreakiness1(double speed) {
        freakiness1 = speed;
        freakiness2 = speed;
    }

    public void eat() {
        freakiness1 = 20000;
        freakiness2 = 10000;
    }

    public void excrete() {
        freakiness1 = Constants.Swerve.Intake.outtakeVelocity;
        freakiness2 = Constants.Swerve.Intake.outtakeVelocity;

    }

    public void stop() {
        freakiness2 = 0.0;
        freakiness1 = 0.0;
    }

    @Override
    public void periodic() {
        intakeControl1 = new VelocityVoltage(freakiness1, 0.0, true,
                feedForward.calculate(freakiness1), 0, false, false, false);
                        intakeControl2 = new VelocityVoltage(freakiness2, 0.0, true,
                feedForward.calculate(freakiness2), 0, false, false, false);
        this.intakeMotor1.setControl(intakeControl1);
        this.intakeMotor2.setControl(intakeControl2);

    }
}
