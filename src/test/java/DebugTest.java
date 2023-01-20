import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import org.junit.jupiter.api.Test;

import frc.robot.utilities.DebugTable;

public class DebugTest {
    @Test
    public void createDoubleTest() {
        DebugTable.set("AAA", 1.1);
        assertEquals(DebugTable.get("AAA").getDouble(), 1.1);
    }

    @Test
    public void getNoSetTest() {
        assertNull(DebugTable.get("BBB").getValue());
    }

    @Test
    public void getSetGetTest() {
        assertNull(DebugTable.get("CCC").getValue());
        DebugTable.set("CCC", "test");
        assertEquals(DebugTable.get("CCC").getString(), "test");
    }
}
