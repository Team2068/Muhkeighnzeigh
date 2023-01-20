package frc.robot.utilities;

import java.lang.Object;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;

public class DebugTable {
    static NetworkTable NetTabInst = NetworkTableInstance.getDefault().getTable("Debug");
    public static NetworkTableValue get(String DataName) {
        return NetTabInst.getValue(DataName);
    };

    public static void set(String DataName, Object Data) {
        NetTabInst.getEntry(DataName).setValue(Data);
    };
}
