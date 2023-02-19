package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

    private final WPI_TalonSRX verticalMotorA;
    private final WPI_TalonSRX verticalMotorB;
    private final WPI_TalonSRX hElevatorMotor;

    private final Encoder verticalEncoder;
    private final Encoder horizontalEncoder;

    public ElevatorSubsystem() {

        this.verticalMotorA = new WPI_TalonSRX(ElevatorConstants.verticalMotorAPort); // only one
                                                                                      // encoder
        // for the two
        // motors
        this.verticalMotorB = new WPI_TalonSRX(ElevatorConstants.verticalMotorBPort);
        this.hElevatorMotor = new WPI_TalonSRX(ElevatorConstants.horizontalMotorPort);

        this.verticalEncoder = new Encoder(ElevatorConstants.verticalEncoderPortA,
                ElevatorConstants.verticalEncoderPortB);
        this.horizontalEncoder = new Encoder(ElevatorConstants.horizonalEncoderPortA,
                ElevatorConstants.horizonalEncoderPortB);
    }

    // VERITCAL
    public double vElevatorEncoderPosition() {
        return verticalMotorA.getSelectedSensorPosition();
    }

    public void vElevatorEncoderReset() {
        verticalMotorA.setSelectedSensorPosition(0);
    }

    public void vElevatorSpeed(double speed) {
        verticalMotorA.set(speed);
        verticalMotorB.set(speed);
    }
    /*
     * public void vElevatorDown(){
     * 
     * verticalElevatorMotorA.set(.25);
     * verticalElevatorMotorB.set(.25);
     * }
     * 
     * public void vElevatorUp(double speed) {
     * verticalElevatorMotorA.set(speed);
     * verticalElevatorMotorB.set(speed);
     * }
     * 
     * public void stopVElevator(){
     * verticalElevatorMotorA.set(0);
     * verticalElevatorMotorB.set(0);
     * }
     */

    // HORRIZONTAL

    public double hElevatorEncoderPosition() {
        return hElevatorMotor.getSelectedSensorPosition();
    }

    public void hElevatorEncoderReset() {
        hElevatorMotor.setSelectedSensorPosition(0);
    }

    public void hElevatorSpeed(double speed) {
        hElevatorMotor.set(speed);
    }

    /*
     * public void hElevatorOut(){
     * hElevatorMotor.set(.25);
     * }
     * 
     * public void hElevatorIn(){
     * hElevatorMotor.set(-.25);
     * }
     * 
     * public void stopHElevator(){
     * hElevatorMotor.set(0);
     * }
     */

}