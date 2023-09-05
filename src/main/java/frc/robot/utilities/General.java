package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class General {
    public static InstantCommand Instant(Runnable... funcs){
        return new InstantCommand(() -> {
            for (Runnable func : funcs) func.run();
        });
    }
}
