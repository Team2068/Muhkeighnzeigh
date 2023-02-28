package frc.robot.commands;

public class SetClawPosition extends CommandBase{
    private final PIDController clawController = new PIDController(0,0,0);
    private final ClawSubsystem clawSubsystem;
    private double lastClawPosition = 0;

    public SetClawPosition(ClawSubsystem clawSubsystem, double angleDegrees){
        this.clawSubsystem = clawSubsystem;
        addRequirements(clawSubsystem);
        clawController.setSetpoint(angleDegrees);
        clawController.setTolerance(1);
        clawController.enableContinuousInput(0,360);

    }
@Override
public void initialize(){

}
@Override
public void execute(){
 var clawSetpoint = clawController.getSetpoint();
 var currentClawPosition = clawSubsystem.getClawPosition();
    double clawPidOutput = clawController.calculate(currentClawPosition);
    double cffOutput = clawSubsystem.calculateClawFeedforward(Math.toRadians(clawSetpoint),
    (Math.toRadians(currentClawPosition) - Math.toRadians(lastClawPosition)) / clawController.getPeriod());
    double newClawOutput = (clawPidOutput / 180 * 8) +cffOutput;
SmartDashboard.putNumber("Claw PID", clawPidOutput);
SmartDashboard.putNumber("Claw FeedForwad", cffOutput);
SmartDashboard.putNumber("Claw Voltage", newClawOutput);

clawSubsystem.setClawVoltage(newClawOutput);
lastClawPosition = currentClawPosition;

}
@Override
public void end(boolean interrupted){
    clawSubsystem.set(0);
}
@Override
public boolean isFinished(){
    return clawController.atSetpoint();
}
}