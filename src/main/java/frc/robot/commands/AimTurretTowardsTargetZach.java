// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.swing.RootPaneContainer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/**
 * A command that continually sets the shooter to face towards the target
 */
public class AimTurretTowardsTargetZach extends CommandBase {
  //Instance Variables
  Double prevX = null;
  Double prevX2 = null;
  Double avgX = null;
  Integer noTargetCount = null;

  /** Creates a new AimTurretTowardsTarget. */
  public AimTurretTowardsTargetZach() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("AIM TURRET TOWARDS TARGET COMMAND ACTIVE");
    prevX = 0.0;
    prevX2 = 0.0;
    avgX = 0.0;
    noTargetCount = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double tv = RobotContainer.limelightTable.getEntry("tv").getDouble(0.0);
    double tx = RobotContainer.limelightTable.getEntry("tx").getDouble(0.0);
    boolean driveTurret = false;

    if(tv == 0.0){
      // No Valid target found
      noTargetCount ++;
      if(noTargetCount > Constants.AIM_TURRET_TOWARDS_TARGET_COMMNAND_MAX_NO_TARGET_COUNT){
        // If no target is found for too long, set shooter to normal position
        RobotContainer.turret.setShooterAbsoluteAngle(0.0);
      }
      else{
        // Not target is found, but we will interpolate the current value
        tx = prevX + (prevX - prevX2);
        driveTurret = true;
      }
    }
    else{
      noTargetCount = 0;
      driveTurret = true;
    }

    if(driveTurret){
      // If the turret needs to be driven
      // Calculate target value in order to smooth data
      avgX += Constants.AIM_TURRET_TOWARDS_TARGET_COMMAND_ALPHA * (tx - avgX);
      RobotContainer.turret.setShooterRelativeAngle(avgX);

      // Update values
      prevX2 = prevX;
      prevX = tx;
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.turret.stopAngleMotor();
    System.out.println("AIM TURRET TOWARDS TARGET COMMAND INACTIVE");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
