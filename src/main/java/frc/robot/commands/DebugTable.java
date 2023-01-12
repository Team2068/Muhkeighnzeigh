package frc.robot.commands;

import java.lang.Object;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class DebugTable {
    static NetworkTable NetTabInst = NetworkTableInstance.getDefault().getTable("Debug");
    public static Object get(String DataName) {
        return NetTabInst.getValue(DataName);
    };

    public static void set(String DataName, Object Data) {
        NetTabInst.getEntry(DataName).setValue(Data);
    };
}
