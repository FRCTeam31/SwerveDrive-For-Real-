// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class TeleopTurnTurretCommand extends CommandBase {
  /** Creates a new TelleopTurnTurretCommand. */
  public TeleopTurnTurretCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.joystick.getPOV() == 90){
        RobotContainer.turret.setShooterRelativeAngle(20);
    }
    else if(RobotContainer.joystick.getPOV() == 270){
        RobotContainer.turret.setShooterRelativeAngle(-20);
    }
    else{
      RobotContainer.turret.setShooterRelativeAngle(0);
    }
    
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
