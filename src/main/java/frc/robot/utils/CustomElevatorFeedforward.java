package frc.robot.utils;

import frc.robot.Constants.ElevatorConstants;

public class CustomElevatorFeedforward {
    public static double feedforward(double velocity) {
        return ElevatorConstants.kGVerticalElevator + ElevatorConstants.kSVerticalElevator * Math.signum(velocity) + ElevatorConstants.kVVerticalElevator * velocity;
    }
}
