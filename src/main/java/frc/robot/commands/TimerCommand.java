// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TimerCommand extends CommandBase {
  double waitTime;
  double endTime;
  /** Creates a new TimerCommand. 
   * @param waitTime The time to wait in seconds
  */
  public TimerCommand(double waitTime){
    // Use addRequirements() here to declare subsystem dependencies.
    this.waitTime = waitTime;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    endTime = (double) System.currentTimeMillis() + waitTime * 1000;
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
    return (double) System.currentTimeMillis() > endTime;
  }
}
