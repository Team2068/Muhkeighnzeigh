package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;

public class DebugValue {

    public static java.lang.Object Get(String DataName) {
        NetworkTable Table = NetworkTableInstance.getDefault().getTable(DataName);
        NetworkTableValue tabEntry = Table.getValue(DataName);
        return tabEntry.getValue();
    };

    public static void Set(String DataName, double Data) {
        NetworkTable Table = NetworkTableInstance.getDefault().getTable(DataName);
        Table.putValue(DataName, NetworkTableValue.makeDouble(Data));
    };
}
