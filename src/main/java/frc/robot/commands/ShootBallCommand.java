// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

/**
 * Powers the shooter motors at a speed controlled by the shooter motor tree map for a set time
 */
public class ShootBallCommand extends CommandBase {
  // Instance Variables
  private Boolean endCommand = false;
  private int maxTime;
  private int time;

  /** Creates a new ShootBallCommand. */
  public ShootBallCommand(int maxTime) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.maxTime = maxTime;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.time = 0;
    // Turn off driveCommand
    RobotContainer.turnOnDriveCommands();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    time ++;
    double tv = RobotContainer.limelightTable.getEntry("tv").getDouble(0.0);
    double ty = RobotContainer.limelightTable.getEntry("ty").getDouble(0.0);

    if(tv==0.0){
      // No Valid Target
    }
    else{
      // Valid target
      // Calculate powers with tree maps
      double topMotorPower = RobotContainer.topMotorTreeMap.getInterpolatedKey(ty);
      double bottomMotorPower = RobotContainer.bottomMotorTreeMap.getInterpolatedKey(ty);
      // Set motor speeds
      RobotContainer.turret.setShooterSpeeds(topMotorPower, bottomMotorPower);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.turret.stopShooterMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (time > maxTime);
  }
}
