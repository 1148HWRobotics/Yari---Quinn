package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.BinarySensor;

public class Intake extends SubsystemBase {
    private static Intake instance;
    private TalonFX intakeMotor;
    private double velocity = 0.0;
    private final SimpleMotorFeedforward feedForward;
    private MotionMagicVelocityTorqueCurrentFOC intakeControl;

    public double getIntakeSpeed() {
        return intakeMotor.getVelocity().getValueAsDouble();
    }

    public void setIntakeSpeed(double intakeSpeed) {
        this.setVelocity(intakeSpeed);
    }

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    private Intake() {
        this.intakeMotor = new TalonFX(Constants.Swerve.Intake.intakeMotorID);
        this.intakeMotor.getConfigurator().apply(new Slot0Configs().withKP(Constants.Swerve.Intake.intakeKP)
                .withKI(Constants.Swerve.Intake.intakeKI).withKD(Constants.Swerve.Intake.intakeKD));
        this.intakeMotor.setInverted(Constants.Swerve.Intake.intakeMotorInverted);
        this.feedForward = new SimpleMotorFeedforward(Constants.Swerve.Intake.kS, Constants.Swerve.Intake.kV,
                Constants.Swerve.Intake.kA);
        this.intakeMotor.setNeutralMode(Constants.Swerve.Intake.intakeNeutralMode);
        this.intakeControl = new MotionMagicVelocityTorqueCurrentFOC(0, 0, true, 0, 0, false, false, false);
        this.intakeMotor.setControl(intakeControl);

    }

    public void setVelocity(double speed) {
        velocity = speed;
    }

    public void intake() {
        velocity = Constants.Swerve.Intake.intakeVelocity;
    }

    public void outtake() {
        velocity = Constants.Swerve.Intake.outtakeVelocity;
    }

    public void stop() {
        velocity = 0.0;
    }

    @Override
    public void periodic() {
        intakeControl = new MotionMagicVelocityTorqueCurrentFOC(velocity, 0.0, true,
                feedForward.calculate(velocity), 0, false, true, false);
        this.intakeMotor.setControl(intakeControl);
        this.intakeMotor.set(velocity);
    }
}
