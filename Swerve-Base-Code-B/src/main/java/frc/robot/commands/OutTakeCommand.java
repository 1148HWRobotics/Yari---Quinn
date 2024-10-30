package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Throat;
import frc.robot.subsystems.Mouth;
import frc.robot.subsystems.Shooter;

public class OutTakeCommand extends Command {
    public OutTakeCommand() {

    }

    @Override
    public void initialize() {
        Shooter.getInstance().setThicknessTarget(4);
        Throat.getInstance().setItsInsideOfMe(false);
        Mouth.getInstance().excrete();
        Throat.getInstance().hawk();
    }

    @Override
    public void end(boolean interrupted) {
        Mouth.getInstance().stop();
        Throat.getInstance().stop();
    }
}
