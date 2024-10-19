package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.util.ShotCalculator;

public class ShooterPresetCommand extends Command {
    private double angle;

    public ShooterPresetCommand(double angle) {
        this.angle = angle;
    }

    @Override
    public void initialize() {

    }

    // hi

    @Override
    public void execute() {
        // Shooter.getInstance()
        // .setAngleTarget(Constants.Swerve.Shooter.adjustedState(Swerve.getInstance().getDistToSpeaker()).angle);
        // Shooter.getInstance()
        // .setVelocity(Constants.Swerve.Shooter.adjustedState(Swerve.getInstance().getDistToSpeaker()).speed_l);
        Shooter.getInstance().setVelocity(Constants.Swerve.Shooter.shootVelocity);
        Shooter.getInstance().setAngleTarget(angle);
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.getInstance().stop();
        Shooter.getInstance().setAngleTarget(1.5);
    }

}
