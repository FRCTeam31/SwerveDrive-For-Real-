// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import java.util.Properties;

import PursellJaques.FalconFXSwerveDrive;
import PursellJaques.FalconFXSwerveModule;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;

public class CalibrateSwerve extends CommandBase {

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Properties wheelAlignmentProperties = new Properties();

    for (FalconFXSwerveModule module: RobotContainer.swerveDrive.swerveModules) {
      wheelAlignmentProperties.put(module.name, Double.toString(module.calculateAngleConstant()));
    }
    wheelAlignmentProperties.put("TURRET", Double.toString(RobotContainer.turret.getZeroAngleConstant()));

    try {
      wheelAlignmentProperties.store(new FileOutputStream(Constants.ALIGNMENT_FILE_PATH), null);
    } catch(IOException e){
      e.printStackTrace();
      System.out.println("Could not store properties");
    }
    System.out.println("CALIBRATED WHEEL CONSTANTS");
    RobotContainer.cancelAllExcept(this);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
