package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

public class WaitUntilConditionForTimeCmd extends CommandBase {

    private final Supplier<Boolean> conditionSupplier;
    private final double duration;
    private double startTS;
    private boolean counting;

    public WaitUntilConditionForTimeCmd(Supplier<Boolean> conditionSupplier, double duration) {
        this.conditionSupplier = conditionSupplier;
        this.duration = duration;
    }

    @Override
    public void initialize() {
        counting = false;
    }

    @Override
    public void execute() {
        if (!conditionSupplier.get()) {
            counting = false;
        }
        if (conditionSupplier.get()) {
            if (!counting) {
                counting = true;
                startTS = Timer.getFPGATimestamp();
            }
        }

        SmartDashboard.putBoolean("Counting", counting);
        SmartDashboard.putNumber("Time", startTS + duration);

    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("Wait command status", "WAITING HAS FINISHED");
    }

    @Override
    public boolean isFinished() {
        return counting && (startTS + duration) <= Timer.getFPGATimestamp();
    }

}
