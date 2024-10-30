package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Throat;
import frc.robot.subsystems.Mouth;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.util.ShotCalculator;

public class AutoShootCommand extends Command {
    private double angle;

    public AutoShootCommand() {
        this.angle = 14;
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
        Shooter.getInstance().setFreakiness(100);
        Shooter.getInstance().setThicknessTarget(angle);

    }

    @Override
    public void end(boolean interrupted) {
        Command shootCommand = new Command() {
            @Override
            public void execute() {
                Throat.getInstance().set(1);
                Mouth.getInstance().eat();
            }
            @Override
            public void end(boolean interrupted) {
                Shooter.getInstance().stop();
                Shooter.getInstance().setThicknessTarget(1.5);
                Mouth.getInstance().stop();
                Throat.getInstance().stop();
            }
        };
        shootCommand.withTimeout(8.5).schedule();
    }
}
