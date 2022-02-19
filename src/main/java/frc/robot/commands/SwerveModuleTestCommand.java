// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import PursellJaques.FalconFXSwerveModule;
import PursellJaques.Vector;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class SwerveModuleTestCommand extends CommandBase {
  // Instance Variables
  FalconFXSwerveModule swerveModule;

  /** Creates a new SwerveModuleTestCommand. */
  public SwerveModuleTestCommand(FalconFXSwerveModule sm) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerveModule = sm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.cancelAllExcept(this);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Vector input = new Vector(RobotContainer.joystick.getX(), 1 * RobotContainer.joystick.getY());
    swerveModule.drive(input);

    System.out.println("(X: " + RobotContainer.joystick.getX() + " Y: " + -1 * RobotContainer.joystick.getY() + " Encoder: " + swerveModule.angleEncoder.getVoltage() + ")");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
