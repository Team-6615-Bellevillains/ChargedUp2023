package frc.robot.commands;

import java.util.function.Supplier;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Constants;

public final ArmSubsystem armSybsystem;

public class ArmCommand extends CommandBase {

    public ArmdCommand (ArmSubsystem armSubsystem){
        this.armSubsystem = armSubsystem;


    }


    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
       
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
