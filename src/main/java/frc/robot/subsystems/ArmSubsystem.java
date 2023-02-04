package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.Encoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem {

private final WPI_TalonSRX armMotor;

private final Encoder armEncoder;

public ArmSubsystem(){

    this.armMotor = new WPI_TalonSRX(ArmConstants.armMotor);

    this.armEncoder = new armEncoder(ArmConstants.armEncoderPort1,ArmConstants.armEncoderPort2);
    this.armEncoder.reset();
    this.armEncoder.setDistancePerPulse(ArmConstants.armEncoderRateInDegrees);
}

public double armEncoderDistance(){
    return armEncoder.getDistance();
}

public void armDown(){
    armMotor.set(.25);
}

public void armUp(){
    armMotor.set(-.25);
}

public void stop(){
    armMotor.stop();
}

    
}
