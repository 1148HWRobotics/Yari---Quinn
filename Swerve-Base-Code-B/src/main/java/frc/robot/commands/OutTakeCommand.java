package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class OutTakeCommand extends Command {
    public OutTakeCommand() {

    }

    @Override
    public void initialize() {
        Shooter.getInstance().setAngleTarget(1.5);
        Carriage.getInstance().setHasNote(false);
        Intake.getInstance().outtake();
        Carriage.getInstance().outtake();
    }

    @Override
    public void end(boolean interrupted) {
        Intake.getInstance().stop();
        Carriage.getInstance().stop();
    }
}
