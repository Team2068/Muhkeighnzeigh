package frc.robot.utilities;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.PhotonConstants;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.SetArmProfiled;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Photonvision;
import frc.robot.subsystems.TelescopeSubsystem;

public class IO {
    final CommandXboxController driveController = new CommandXboxController(0);
    final CommandXboxController mechController = new CommandXboxController(1);

    public final ArmSubsystem arm = new ArmSubsystem();
    public final LEDSubsystem leds = new LEDSubsystem();
    public final ClawSubsystem claw = new ClawSubsystem();
    public final Photonvision photon = new Photonvision();
    public final DriveSubsystem driveSubsystem = new DriveSubsystem();
    public final TelescopeSubsystem telescope = new TelescopeSubsystem();

    public SetArmProfiled armCommand = new SetArmProfiled(73, arm, telescope, photon::rotateMount, true);

    public void configGlobal() {
        photon.camera.setPipelineIndex(1);
        photon.mount.setAngle(PhotonConstants.FORWARD_ANGLE);
        
        driveSubsystem.setDefaultCommand(new DefaultDriveCommand(driveSubsystem,
                () -> -modifyAxis(driveController.getLeftY()) * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                () -> -modifyAxis(driveController.getLeftX()) * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                () -> -modifyAxis(driveController.getRightX())* DriveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));
        arm.setDefaultCommand(armCommand);
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    public void configTeleop() {
        mechController.a().onTrue(General.Instant(armCommand::stop));
        mechController.b().onTrue(General.Instant(() -> armCommand.setAngle(0)));
        mechController.x().onTrue(General.Instant(() -> armCommand.setAngle(-60)));
        mechController.y().onTrue(General.Instant(() -> armCommand.setAngle(60)));

        mechController.leftBumper().onTrue(General.Instant(claw::closeClaw, () -> leds.setAllLeds(new Color(0, 0, 0.25))));
        mechController.rightBumper().onTrue(General.Instant(claw::openClaw, () -> leds.setAllLeds(new Color(0.2, 0.15, 0))));

        mechController.leftTrigger()
                .whileTrue(General.Instant(telescope::extendTelescope))
                .whileFalse(General.Instant(telescope::stopTelescope));
        mechController.povDown()
                .whileTrue(General.Instant(telescope::retractTelescope))
                .whileFalse(General.Instant(telescope::stopTelescope));
        mechController.povUp().onTrue(General.Instant(telescope::resetPosition));

        claw.setDefaultCommand(new InstantCommand(
            () -> claw.setWristVoltage(MathUtil.clamp(modifyAxis(mechController.getLeftY()) * ClawConstants.WRIST_VOLTAGE,
                -ClawConstants.WRIST_VOLTAGE, ClawConstants.WRIST_VOLTAGE)),
                claw).alongWith(General.Instant(() -> claw.setIntakeSpeed(mechController.getRightY() * 2))));

        driveController.b().onTrue(General.Instant(driveSubsystem::syncEncoders));
        driveController.y().onTrue(General.Instant(driveSubsystem::resetOdometry));
        driveController.a().onTrue(General.Instant(driveSubsystem::syncEncoders));
        driveController.x().onTrue(General.Instant(driveSubsystem::zeroGyro));
    }

    public void configTesting() {
        SmartDashboard.putData("Kill LEDs", General.Instant(leds::killLeds));
        PathPlannerServer.startServer(5811);

        driveController.a().onTrue(runSystemsCheck());
        driveController.b().onTrue(General.Instant(driveSubsystem::syncEncoders));
        driveController.y().onTrue(General.Instant(driveSubsystem::resetOdometry));

        driveController.leftBumper().onTrue(General.Instant(armCommand::stop));
        driveController.rightBumper().onTrue(General.Instant(()->armCommand.setAngle(60)));

        driveController.leftTrigger().onTrue(General.Instant(claw::closeClaw, () -> leds.setAllLeds(new Color(0, 0, 0.25))));
        driveController.rightTrigger().onTrue(General.Instant(claw::openClaw, () -> leds.setAllLeds(new Color(0.2, 0.15, 0))));
    }

    public Command runSystemsCheck(){
        driveSubsystem.syncEncoders();
        driveSubsystem.resetOdometry();

        return new SequentialCommandGroup(
          new PrintCommand("Starting..."),
            // TODO: Extend the drive sys tests to cover moving forward, back, left, right, rotating clock-wise & counter clock-wise
          General.Instant(()->driveSubsystem.drive(new ChassisSpeeds(10,0,0)),
                          ()->armCommand.setAngle(60)),
          new WaitCommand(0.5),
          General.Instant(()-> armCommand.setAngle(-60)),
          new WaitCommand(0.5),
          General.Instant(claw::intake, claw::openClaw, telescope::extendTelescope),
          new WaitCommand(0.5),
          General.Instant(claw::output, claw::closeClaw, telescope::retractTelescope),
          new WaitCommand(0.6),
          General.Instant(armCommand::stop, claw::stopClaw, telescope::stopTelescope, driveSubsystem::stop)
        );
    }

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) <= deadband)
            return 0.0;
        deadband *= (value > 0.0) ? 1 : -1;
        return (value + deadband) / (1.0 + deadband);
    }

    private static double modifyAxis(double value) {
        value = deadband(value, 0.05); // Deadband
        value = Math.copySign(value * value, value); // Square the axis
        return value;
    }
}
