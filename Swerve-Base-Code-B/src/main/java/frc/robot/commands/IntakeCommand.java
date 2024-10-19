package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Intake;
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
        Shooter.getInstance().setAngleTarget(1.5);
        if (!Carriage.getInstance().getNoteSensor().get()) {
            Intake.getInstance().stop();
            Carriage.getInstance().stop();
            Carriage.getInstance().setHasNote(true);
        } else {
            Intake.getInstance().intake();
            Carriage.getInstance().intake();
        }
    }

    @Override
    public void end(boolean interrupted) {
        Intake.getInstance().stop();
        Carriage.getInstance().stop();
    }

    @Override
    public boolean isFinished() {
        if (!Carriage.getInstance().getNoteSensor().get())
            return true;
        return false;
    }
}
