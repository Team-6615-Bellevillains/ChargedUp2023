package frc.robot.utils;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

public class TunableSimpleMotorFeedforward {

    private final static NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
    private final static NetworkTable tuningTable = networkTableInstance.getTable("tuning");
    private final static double updateInterval = 3;
    private final static ArrayList<TunableSimpleMotorFeedforward> feedforwards = new ArrayList<>();

    private SimpleMotorFeedforward simpleMotorFeedforward;
    private String identifier;

    private double lastUpdatedTS = Timer.getFPGATimestamp();

    public TunableSimpleMotorFeedforward(String identifier, double ks, double kv) {
        this(identifier, ks, kv, 0);
    }

    public TunableSimpleMotorFeedforward(String identifier, double ks, double kv, double ka) {
        simpleMotorFeedforward = new SimpleMotorFeedforward(ks, kv, ka);

        this.identifier = identifier;

        tuningTable.getEntry(appendIdentifier("ks")).setDouble(ks);
        tuningTable.getEntry(appendIdentifier("kv")).setDouble(kv);
        tuningTable.getEntry(appendIdentifier("ka")).setDouble(ka);

        feedforwards.add(this);
    }

    public SimpleMotorFeedforward getController() {
        return simpleMotorFeedforward;
    }

    public String appendIdentifier(String input) {
        return "smFF" + input + identifier;
    }

    public void updateConstantsIfOutdated() {
        if (Timer.getFPGATimestamp() - lastUpdatedTS < updateInterval)
            return;

        double tableKS = tuningTable.getValue(appendIdentifier("ks")).getDouble();
        double tableKV = tuningTable.getValue(appendIdentifier("kv")).getDouble();
        double tableKA = tuningTable.getValue(appendIdentifier("ka")).getDouble();

        if (simpleMotorFeedforward.ks != tableKS
                || simpleMotorFeedforward.kv != tableKV
                || simpleMotorFeedforward.ka != tableKA) {
            SmartDashboard.putNumber(String.format("Last update to %s", identifier), Timer.getFPGATimestamp());

            simpleMotorFeedforward = new SimpleMotorFeedforward(tableKS, tableKV, tableKA);
        }

        lastUpdatedTS = Timer.getFPGATimestamp();
    }

    public static void updateControllersIfOutdated() {
        for (TunableSimpleMotorFeedforward feedforward : feedforwards) {
            feedforward.updateConstantsIfOutdated();
        }
    }

}
