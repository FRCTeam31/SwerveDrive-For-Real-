// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ClimbingCommand extends CommandBase {
  /** Creates a new ClimbingCommand. */
  public ClimbingCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.joystick.getPOV() == 0){
      RobotContainer.climbingMechanism.powerClimbMotor(Constants.SAFE_CLIMBING_SPEED);
    }
    else if(RobotContainer.joystick.getPOV() == 180){
      RobotContainer.climbingMechanism.powerClimbMotor(-1 * Constants.SAFE_CLIMBING_SPEED);
    }
    else{
      RobotContainer.raiseIntakeMotor.stopMotor();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.climbingMechanism.climbingMotor.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
