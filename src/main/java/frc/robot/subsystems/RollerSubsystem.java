package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GrabberConstants;

public class RollerSubsystem extends SubsystemBase {

    private CANSparkMax leftMotorRoller;
    private CANSparkMax rightMotorRoller;

    public RollerSubsystem() {
        leftMotorRoller = new CANSparkMax(GrabberConstants.kLeftRollerMotorPort, MotorType.kBrushless);
        rightMotorRoller = new CANSparkMax(GrabberConstants.kRightRollerMotorPort, MotorType.kBrushless);

        leftMotorRoller.setInverted(true);
    }

    public void setRollerSpeedPercentage(double speedPercentage) {
        leftMotorRoller.set(speedPercentage);
        rightMotorRoller.set(speedPercentage);
    }

}
