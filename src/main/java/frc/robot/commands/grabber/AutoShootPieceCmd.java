package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RollerSubsystem;

public class AutoShootPieceCmd extends CommandBase {

    private RollerSubsystem rollerSubsystem;

    private double robotTime;
    private double shootDuration;
    private boolean isDone;
    private double shootOutTime;
    private double power;

    public AutoShootPieceCmd(RollerSubsystem rollerSubsystem, double shootDuration, double power) {
        this.rollerSubsystem = rollerSubsystem;
        this.shootDuration = shootDuration;
        this.power = power;

        addRequirements(rollerSubsystem);
    }

    @Override
    public void initialize() {
        robotTime = 0;
        isDone = false;

        robotTime = Timer.getFPGATimestamp();
        shootOutTime = shootDuration + robotTime;
    }

    @Override
    public void execute() {
        robotTime = Timer.getFPGATimestamp();
        if(robotTime < shootOutTime){
            rollerSubsystem.setRollerSpeedPercentage(power);
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