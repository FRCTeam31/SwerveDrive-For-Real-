// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DriveCommand extends CommandBase {
  /** Creates a new DriveCommand. */
  public DriveCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    RobotContainer.swerveDrive.reset(); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("Drive Command", "ACTIVE");
    RobotContainer.cancelAllExcept(this);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // X is left to right [-1, 1], Y is front to back [1, -1], Z is CCW to CW [-1, 1]
    RobotContainer.swerveDrive.driveWithDeadZone(RobotContainer.joystick.getX(), RobotContainer.joystick.getY(), RobotContainer.joystick.getZ(), 0.1);
    //RobotContainer.swerveDrive.fieldOrientedDriveWithDeadZone(RobotContainer.joystick.getX(), -1 * RobotContainer.joystick.getY(), RobotContainer.joystick.getZ(), 0.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("THE DRIVE COMMAND HAS ENDED___________");
    RobotContainer.swerveDrive.stopModulesMotion();
    SmartDashboard.putString("Drive Command", "NOT ACTIVE");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;

  }
}
