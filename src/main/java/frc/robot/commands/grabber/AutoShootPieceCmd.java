package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RollerSubsystem;

public class AutoShootPieceCmd extends CommandBase {

    private RollerSubsystem rollerSubsystem;

    private double robotTime;
    private double shootOutDuration;
    private double finishedShootingTimestamp;
    private boolean isDone;

    public AutoShootPieceCmd(RollerSubsystem rollerSubsystem, double shootOutDuration) {
        this.rollerSubsystem = rollerSubsystem;
        this.shootOutDuration = shootOutDuration;

        addRequirements(rollerSubsystem);
    }

    @Override
    public void initialize() {
        robotTime = 0;
        isDone = false;

        robotTime = Timer.getFPGATimestamp();
        finishedShootingTimestamp = robotTime + shootOutDuration;
    }

    @Override
    public void execute() {
        robotTime = Timer.getFPGATimestamp();
        if(robotTime < shootOutDuration){
        rollerSubsystem.setRollerSpeedPercentage(.1);
        }
        else
        {
            isDone = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        rollerSubsystem.setRollerSpeedPercentage(0);
    }

    @Override
    public boolean isFinished() {
      return isDone;
    }
}