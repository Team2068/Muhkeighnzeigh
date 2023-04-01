// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.Constants.PhotonConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Photonvision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Aimbot extends PIDCommand {
  /** Creates a new Aimbot. */
  DriveSubsystem driveSubsystem;
  Photonvision photonvision;

  public Aimbot(Photonvision photonvision, DriveSubsystem driveSubsystem, boolean rotating) {
    super(
        // The controller that the command will use
        new PIDController(Constants.AimbotConstants.kP, Constants.AimbotConstants.kI, Constants.AimbotConstants.kD),
        // This should return the measurement
        () -> photonvision.data.targetYaw,
        // This should return the setpoint (can also be a constant)
        () -> ((photonvision.isFlipped()) ? PhotonConstants.AIMBOT_OFFSET_BACKWARD : PhotonConstants.AIMBOT_OFFSET_FORWARD),
        // This uses the output
        output -> {
          driveSubsystem.drive(new ChassisSpeeds(0, 
                                                  ((photonvision.isFlipped()) ? -1 : 1) * output * Constants.AimbotConstants.speed * Constants.DRIVE_MAX_VELOCITY_METERS_PER_SECOND, 
                                                  0));
        });

    this.driveSubsystem = driveSubsystem;
    this.photonvision = photonvision;
    addRequirements(driveSubsystem, photonvision);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(getController().getPositionError()) < Constants.AimbotConstants.minimumAdjustment || !photonvision.camera.getLatestResult().hasTargets();
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(new ChassisSpeeds(0, 0, 0));
  }
}
// lol hahahah uheguyavfgyavsyhaevgfyhsaebj WEINER 