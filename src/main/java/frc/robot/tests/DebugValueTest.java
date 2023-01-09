package frc.robot.tests;

import frc.robot.commands.DebugValue;
// Optional Test for DebugValue located in commands directory. (Can be removed as well as the directory if not wanted)
public class DebugValueTest {
    public void Test() {
        String DataName = "TestValue";
        System.out.println(DebugValue.get(DataName));
    
        DebugValue.set(DataName, 2);
        System.out.println(DebugValue.get(DataName));
    }
}
