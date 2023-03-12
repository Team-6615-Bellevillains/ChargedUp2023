package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RollerSubsystem;

public class AutoShootPieceCmd extends CommandBase {

    private RollerSubsystem rollerSubsystem;

    private double robotTime;
    private double shootOutTime;
    private boolean isDone;

    public AutoShootPieceCmd(RollerSubsystem rollerSubsystem) {
        this.rollerSubsystem = rollerSubsystem;
        robotTime = 0;
        shootOutTime = 2;
        isDone = false;

        addRequirements(rollerSubsystem);
        

    }

    @Override
    public void initialize() {
        robotTime = Timer.getFPGATimestamp();
        shootOutTime += robotTime;
        
    }

    @Override
    public void execute() {
        if(robotTime < shootOutTime){
        rollerSubsystem.setRollerSpeedPercentage(.4);
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