package frc.robot.subsystems;

import javax.swing.text.Position;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CTREConfigs;
import frc.robot.Constants;
import frc.robot.util.BinarySensor;

public class Shooter extends SubsystemBase {
    private static Shooter instance;
    private TalonFX shooterMotor1;
    private TalonFX shooterMotor2;
    private TalonFX angleMotor1;
    private TalonFX angleMotor2;
    private double freakiness1 = 0.0;
    private double freakiness2 = 0.0;
    public double getVelocity() {
        return freakiness1;
    }

    private double thicknessCurrent = 0.0;
    private double thicknessTarget = 0.0;
    public double getThicknessTarget() {
        return thicknessTarget;
    }

    private NeutralModeValue angleNeutralMode = Constants.Swerve.Shooter.angleNeutralMode;
    private NeutralModeValue velocityNeutralMode = Constants.Swerve.Shooter.fireNeutralMode;
    
    private VelocityVoltage freakyControl1;
    private VelocityVoltage freakyControl2;
    private PositionVoltage angleControl;

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    private Shooter() {
        this.shooterMotor1 = new TalonFX(Constants.Swerve.Shooter.fireMotor1ID);
        this.shooterMotor2 = new TalonFX(Constants.Swerve.Shooter.fireMotor2ID);
        this.angleMotor1 = new TalonFX(Constants.Swerve.Shooter.angleMotor1ID);
        this.angleMotor2 = new TalonFX(Constants.Swerve.Shooter.angleMotor2ID);
        this.shooterMotor1.setInverted(Constants.Swerve.Shooter.fireMotor1Inverted);
        this.shooterMotor2.setInverted(Constants.Swerve.Shooter.fireMotor2Inverted);
        this.angleMotor1.setInverted(Constants.Swerve.Shooter.angleMotor1Inverted);
        this.angleMotor2.setInverted(Constants.Swerve.Shooter.angleMotor2Inverted);
        this.shooterMotor1.setNeutralMode(Constants.Swerve.Shooter.fireNeutralMode);
        this.shooterMotor2.setNeutralMode(Constants.Swerve.Shooter.fireNeutralMode);
        this.angleMotor1.setNeutralMode(Constants.Swerve.Shooter.angleNeutralMode);
        this.angleMotor2.setNeutralMode(Constants.Swerve.Shooter.angleNeutralMode);
        shooterMotor1.getConfigurator().apply(new MotionMagicConfigs());
        shooterMotor2.getConfigurator().apply(new MotionMagicConfigs());
        angleMotor1.getConfigurator().apply(new MotionMagicConfigs());
        angleMotor2.getConfigurator().apply(new MotionMagicConfigs());
        this.shooterMotor1.getConfigurator().apply(new Slot0Configs().withKP(Constants.Swerve.Shooter.shootKP)
                .withKI(Constants.Swerve.Shooter.shootKI).withKD(Constants.Swerve.Shooter.shootKD).withKS(Constants.Swerve.Shooter.shootkS).withKV(Constants.Swerve.Shooter.shootkV).withKA(Constants.Swerve.Shooter.shootkA));
        this.shooterMotor2.getConfigurator().apply(new Slot0Configs().withKP(Constants.Swerve.Shooter.shootKP).withKI(thicknessCurrent).withKD(Constants.Swerve.Shooter.shootKD).withKS(Constants.Swerve.Shooter.shootkS).withKV(Constants.Swerve.Shooter.shootkV).withKA(Constants.Swerve.Shooter.shootkA));
        this.angleMotor1.getConfigurator().apply(new Slot0Configs().withKP(Constants.Swerve.Shooter.angleKP)
                .withKI(Constants.Swerve.Shooter.angleKI).withKD(Constants.Swerve.Shooter.angleKD).withKS(Constants.Swerve.Shooter.anglekS).withKG(Constants.Swerve.Shooter.anglekG).withKV(Constants.Swerve.Shooter.anglekV));
        this.angleMotor2.getConfigurator().apply(new Slot0Configs().withKP(Constants.Swerve.Shooter.angleKP).withKI(thicknessCurrent).withKD(Constants.Swerve.Shooter.angleKD).withKS(Constants.Swerve.Shooter.anglekS).withKG(Constants.Swerve.Shooter.anglekG).withKV(Constants.Swerve.Shooter.anglekV));
        this.freakyControl1 = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
        this.freakyControl2 = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
        this.angleControl = new PositionVoltage(0.0).withEnableFOC(true);
        this.shooterMotor1.setControl(freakyControl1);
        this.shooterMotor2.setControl(freakyControl2);
        this.angleMotor1.setControl(angleControl);
        this.angleMotor2.setControl(angleControl);
        this.angleMotor1.setPosition(0.0);
        this.angleMotor2.setPosition(0.0);
    }

    public void setFreakiness(double speed) {
        freakiness1 = speed;
        freakiness2 = speed;
    }

    public void setThicknessTarget(double angle) {
        thicknessTarget = angle;
    }

    public void stop() {
        freakiness1 = 0;
        freakiness2 = 0;
    }

    public void goHome() {
        thicknessTarget = 0;
    }

    public double getThicknessCurrent() {
        return angleMotor1.getPosition().getValueAsDouble();
    }

    public void amp() {
        thicknessTarget = Constants.Swerve.Shooter.upAngle;
        
    }

    @Override
    public void periodic() {
        freakyControl1 = new VelocityVoltage(freakiness1, 0, true, 0, 0, false, false, false);
        freakyControl2 = new VelocityVoltage(freakiness2, 0, true, 0, 0, false, false, false);
        angleControl = new PositionVoltage(thicknessTarget).withEnableFOC(true);
        this.shooterMotor1.setControl(freakyControl1);
        this.shooterMotor2.setControl(freakyControl2);
        this.angleMotor1.setControl(angleControl);
        this.angleMotor2.setControl(angleControl);
    }

}