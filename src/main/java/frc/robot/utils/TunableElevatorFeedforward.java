package frc.robot.utils;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.ArrayList;

public class TunableElevatorFeedforward {

    private final static NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
    private final static NetworkTable tuningTable = networkTableInstance.getTable("tuning");
    private final static double updateInterval = 3;
    private final static ArrayList<TunableElevatorFeedforward> feedforwards = new ArrayList<>();

    private ElevatorFeedforward elevatorFeedforward;
    private String identifier;

    private double lastUpdatedTS = Timer.getFPGATimestamp();

    public TunableElevatorFeedforward(String identifier, double ks, double kg, double kv) {
        this(identifier, ks, kg, kv, 0);
    }

    public TunableElevatorFeedforward(String identifier, double ks, double kg, double kv, double ka) {
        elevatorFeedforward = new ElevatorFeedforward(ks, kg, kv, ka);

        this.identifier = identifier;

        tuningTable.getEntry(appendIdentifier("ks")).setDouble(ks);
        tuningTable.getEntry(appendIdentifier("kg")).setDouble(kg);
        tuningTable.getEntry(appendIdentifier("kv")).setDouble(kv);
        tuningTable.getEntry(appendIdentifier("ka")).setDouble(ka);

        feedforwards.add(this);
    }

    public ElevatorFeedforward getController() {
        return elevatorFeedforward;
    }

    public String appendIdentifier(String input) {
        return "eFF" + input + identifier;
    }

    public void updateConstantsIfOutdated() {
        if (Timer.getFPGATimestamp() - lastUpdatedTS < updateInterval)
            return;

        double tableKS = tuningTable.getValue(appendIdentifier("ks")).getDouble();
        double tableKG = tuningTable.getValue(appendIdentifier("kg")).getDouble();
        double tableKV = tuningTable.getValue(appendIdentifier("kv")).getDouble();
        double tableKA = tuningTable.getValue(appendIdentifier("ka")).getDouble();

        if (elevatorFeedforward.ks != tableKS
                || elevatorFeedforward.kg != tableKG
                || elevatorFeedforward.kv != tableKV
                || elevatorFeedforward.ka != tableKA) {
            SmartDashboard.putNumber(String.format("Last update to %s", identifier), Timer.getFPGATimestamp());

            elevatorFeedforward = new ElevatorFeedforward(tableKS, tableKG, tableKV, tableKA);
        }

        lastUpdatedTS = Timer.getFPGATimestamp();
    }

    public static void updateControllersIfOutdated() {
        for (TunableElevatorFeedforward feedforward : feedforwards) {
            feedforward.updateConstantsIfOutdated();
        }
    }

}
