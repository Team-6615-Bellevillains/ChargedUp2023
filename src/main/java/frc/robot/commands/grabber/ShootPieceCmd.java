package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RollerSubsystem;

public class ShootPieceCmd extends CommandBase {

    private RollerSubsystem rollerSubsystem;

    public ShootPieceCmd(RollerSubsystem rollerSubsystem) {
        this.rollerSubsystem = rollerSubsystem;

        addRequirements(rollerSubsystem);
    }

    @Override
    public void execute() {
        rollerSubsystem.setRollerSpeedPercentage(.2);
    }

    @Override
    public void end(boolean interrupted) {
        rollerSubsystem.setRollerSpeedPercentage(0);
    }

}