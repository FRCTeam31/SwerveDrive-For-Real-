// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import PursellJaques.N_PID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class TurnAngleCommand extends CommandBase {
  //instance variables
  N_PID anglePIDController;
  double targetAngle;
  double inputAngle;
  double counter = 0;
  /** Creates a new TurnAngleCommand. */
  public TurnAngleCommand(double inputAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.inputAngle = inputAngle;
    anglePIDController = new N_PID(Constants.TTA_KP, Constants.TTA_KI, Constants.TTA_KD);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetAngle = RobotContainer.navX.getAngle() + inputAngle;
    anglePIDController.reset(0);
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.swerveDrive.drive(0, 0, anglePIDController.run(RobotContainer.navX.getAngle(), targetAngle));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentAngle = RobotContainer.navX.getAngle();
    if(currentAngle < targetAngle + Constants.TTA_ERROR & currentAngle > targetAngle - Constants.TTA_ERROR){
      counter ++;
    }
    else{
      counter = 0;
    }

    return counter > Constants.TTA_MAX_COUNTER;
  }
}
