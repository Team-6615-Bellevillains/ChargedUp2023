package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RollerSubsystem;

public class ShootPieceCmd extends CommandBase {

    private RollerSubsystem rollerSubsystem;
    private double rollerSpeedPercentage;

    public ShootPieceCmd(RollerSubsystem rollerSubsystem) {
        this(rollerSubsystem, 0.25);
    }

    public ShootPieceCmd(RollerSubsystem rollerSubsystem, double rollerSpeedPercentage) {
        this.rollerSubsystem = rollerSubsystem;
        this.rollerSpeedPercentage = rollerSpeedPercentage;

        addRequirements(rollerSubsystem);
    }

    @Override
    public void execute() {
        rollerSubsystem.setRollerSpeedPercentage(rollerSpeedPercentage);
    }

    @Override
    public void end(boolean interrupted) {
        rollerSubsystem.setRollerSpeedPercentage(0);
    }

}
