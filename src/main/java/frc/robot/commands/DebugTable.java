package frc.robot.commands;

import java.lang.Object;
import edu.wpi.first.networktables.NetworkTableInstance;

public class DebugTable {
    static NetworkTableInstance NetTabInst = NetworkTableInstance.getDefault();
    public static Object get(String DataName) {
        return NetTabInst.getTable(DataName).getValue(DataName);
    };

    public static void set(String DataName, Object Data) {
        NetTabInst.getEntry(DataName).setValue(Data);
    };
}
