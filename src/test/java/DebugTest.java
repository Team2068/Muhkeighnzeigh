import frc.robot.utilities.DebugTable;
// Optional Test for DebugValue located in commands directory. (Can be removed as well as the directory if not wanted)
public class DebugTest {

    public void Test() {
        String DataName = "TestValue";
        System.out.println(DebugTable.get(DataName));
    
        DebugTable.set(DataName, 2);
        System.out.println(DebugTable.get(DataName));
    }
}
