package frc.robot.commands;

import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.util.ShotCalculator;
import frc.robot.util.ShotCalculator.ShooterSolution;

public class SpinShooterCommand extends Command {
    Pose2d currentPose;
    ShooterSolution solution;
    MotionMagicVelocityTorqueCurrentFOC originalFireControl;
    // todo Implement PathPlanner Shoot Command
    public SpinShooterCommand() {
        currentPose = Swerve.getInstance().getPose();
        solution = new ShooterSolution(Shooter.getInstance().getVelocity(), Shooter.getInstance().getAngleCurrent());
    }

    @Override
    public void initialize() {
        if(solution != null){
            Shooter.getInstance().setAngleTarget(solution.angle);
            Shooter.getInstance().setVelocity(solution.velocity);
        }
    }

    @Override
    public void execute() {
        currentPose = Swerve.getInstance().getPose();
        solution = ShotCalculator.calculateOptimalShooterParameters(currentPose);
        if(solution != null) {
            if(Math.abs(solution.velocity - Shooter.getInstance().getVelocity()) > 0.5) {
                Shooter.getInstance().setVelocity(solution.velocity);
            }
            if(Math.abs(solution.angle - Shooter.getInstance().getAngleCurrent()) > 0.5) {
                Shooter.getInstance().setAngleCurrent(solution.angle);
            }
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.getInstance().setFireControl(originalFireControl);
        Shooter.getInstance().setVelocity(0);
        Shooter.getInstance().setAngleTarget(0);
    }
}
