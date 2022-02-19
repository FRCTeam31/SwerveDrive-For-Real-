// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class BallSuckCommand extends CommandBase {
  // the times in nanoseconds to spin the intake motor and drive straight forward
  double suckDuration, driveDuration;
  double startTime;

  public BallSuckCommand(double suckDuration, double driveDuration) {
    this.suckDuration = suckDuration;
    this.driveDuration = driveDuration;
    startTime = (double)System.currentTimeMillis();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.swerveDrive.drive(0, Constants.BALL_SUCK_SPEED, 0);
    RobotContainer.intakeMotor.set(ControlMode.Velocity, Constants.SAFE_INTAKE_SPEED * Constants.FALCON_MAX_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double timeElapsed = (double)System.currentTimeMillis() - startTime;
    return timeElapsed > suckDuration && timeElapsed > driveDuration;
  }
}
