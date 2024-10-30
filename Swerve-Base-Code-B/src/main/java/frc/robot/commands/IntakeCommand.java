package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Throat;
import frc.robot.subsystems.Mouth;
import frc.robot.subsystems.Shooter;

public class IntakeCommand extends Command {
    // todo Implement PathPlanner Intake Command
    public IntakeCommand() {
    }

    @Override
    public void initialize() {

    }

    private double interceptPosition;

    @Override
    public void execute() {
        Shooter.getInstance().setThicknessTarget(0);
        if (!Throat.getInstance().getNoteSensor().get()) {
            Mouth.getInstance().stop();
            Throat.getInstance().stop();
            Throat.getInstance().setItsInsideOfMe(true);
        } else {
            Mouth.getInstance().eat();
            Throat.getInstance().swallow();
        }
    }

    @Override
    public void end(boolean interrupted) {
        Mouth.getInstance().stop();
        Throat.getInstance().stop();
    }

    @Override
    public boolean isFinished() {
        if (!Throat.getInstance().getNoteSensor().get())
            return true;
        return false;
    }
}
