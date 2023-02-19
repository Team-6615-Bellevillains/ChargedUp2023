package frc.robot.commands.group;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.grabber.SuckObjectCmd;
import frc.robot.commands.grabber.OpenGrabberCmd;
import frc.robot.commands.grabber.ClampGrabberCmd;
import frc.robot.subsystems.GrabberSubsystem;

public class IntakeCone extends CommandBase {

    private GrabberSubsystem grabberSubsystem;

    private OpenGrabberCmd openGrabberCmd = new OpenGrabberCmd(grabberSubsystem);
    private SuckObjectCmd suckObjectCmd = new SuckObjectCmd(grabberSubsystem, 500);
    private ClampGrabberCmd clampGrabberCmd = new ClampGrabberCmd(grabberSubsystem);

    public IntakeCone(GrabberSubsystem grabberSubsystem) {
        this.grabberSubsystem = grabberSubsystem;

        addRequirements(grabberSubsystem);
    }

    @Override
    public void initialize() {
        openGrabberCmd
                .andThen(new WaitCommand(0.5))
                .andThen(suckObjectCmd)
                .andThen(new WaitCommand(0.5))
                .andThen(clampGrabberCmd).schedule();
    }
}