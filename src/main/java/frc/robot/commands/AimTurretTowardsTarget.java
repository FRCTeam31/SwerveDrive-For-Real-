// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

/**
 * A command that continually sets the shooter to face towards the target
 */
public class AimTurretTowardsTarget extends CommandBase {
  /** Creates a new AimTurretTowardsTarget. */
  public AimTurretTowardsTarget() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   

    if (RobotContainer.limelightTable.getEntry("tv").getDouble(0.0) == 0){
      // If not target is found set the shooter to face forward
      RobotContainer.turret.setShooterAbsoluteAngle(0);
    }
    else{
      // If there is a target set the shooter to face the target
      RobotContainer.turret.setShooterRelativeAngle(RobotContainer.limelightTable.getEntry("tx").getDouble(0.0));
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.turret.stopAngleMotor();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
