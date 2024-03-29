package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RollerSubsystem;

public class AutoSuckPieceCmd extends CommandBase {

    private RollerSubsystem rollerSubsystem;

    private double robotTime;
    private double shootOutTime;
    private boolean isDone;

    public AutoSuckPieceCmd(RollerSubsystem rollerSubsystem) {
        this.rollerSubsystem = rollerSubsystem;

        addRequirements(rollerSubsystem);
    }

    @Override
    public void initialize() {
        robotTime = 0;
        shootOutTime = 3.5;
        isDone = false;

        robotTime = Timer.getFPGATimestamp();
        shootOutTime += robotTime;
    }

    @Override
    public void execute() {
        robotTime = Timer.getFPGATimestamp();
        if (robotTime < shootOutTime) {
            rollerSubsystem.setRollerSpeedPercentage(-.15);
        } else {
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
