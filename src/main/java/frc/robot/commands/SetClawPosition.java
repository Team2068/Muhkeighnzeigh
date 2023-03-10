package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class SetClawPosition extends CommandBase {
    private final PIDController clawController = new PIDController(0.8, 0, 0);
    private final ClawSubsystem clawSubsystem;
    private double lastClawPosition = 0;

    public SetClawPosition(ClawSubsystem clawSubsystem, double angleDegrees) {
        this.clawSubsystem = clawSubsystem;
        addRequirements(clawSubsystem);
        clawController.setSetpoint(angleDegrees);
        clawController.setTolerance(1);
        clawController.enableContinuousInput(0, 360);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double currentClawPosition = clawSubsystem.getClawPosition();
        double clawPidOutput = clawController.calculate(currentClawPosition);
        double cffOutput = clawSubsystem.calculateClawFeedforward(Math.toRadians(clawController.getSetpoint()),
                (Math.toRadians(currentClawPosition) - Math.toRadians(lastClawPosition)) / clawController.getPeriod());
        double newClawOutput = (clawPidOutput / 180 * 8) + cffOutput;
        SmartDashboard.putNumber("Claw PID", clawPidOutput);
        SmartDashboard.putNumber("Claw FeedForwad", cffOutput);
        SmartDashboard.putNumber("Claw Voltage", newClawOutput);

        clawSubsystem.setWristVoltage(MathUtil.clamp(newClawOutput, -12, 12));
        lastClawPosition = currentClawPosition;
    }

    @Override
    public void end(boolean interrupted) {
        clawSubsystem.setIntakeSpeed(0);
        clawSubsystem.setWristSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return clawController.atSetpoint();
    }
}