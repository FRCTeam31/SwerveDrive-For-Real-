// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import PursellJaques.N_PID;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class TurnAngleCommand extends CommandBase {
  //instance variables
  public PIDController anglePIDController;
  public double targetAngle;
  public double inputAngle;
  public double counter = 0;
  public Timer timer;
  public double maxTime;


  /** Creates a new TurnAngleCommand. */
  public TurnAngleCommand(double inputAngle, double maxTime) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.inputAngle = inputAngle;
    anglePIDController = new PIDController(Constants.TTA_KP, Constants.TTA_KI, Constants.TTA_KD);
    this.maxTime = maxTime;
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetAngle = RobotContainer.navX.getAngle() + inputAngle;
    anglePIDController.reset();
    counter = 0;
    RobotContainer.swerveDrive.setNeutralMode(NeutralMode.Brake);
    SmartDashboard.putString("Current Command", "Turn angle command");

    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.swerveDrive.drive(0, 0, anglePIDController.calculate(RobotContainer.navX.getAngle(), targetAngle));
    SmartDashboard.putNumber("Current Robot Angle", RobotContainer.navX.getAngle());

    if(timer.get() > maxTime){
      this.end(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.swerveDrive.stopModulesMotion();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentAngle = RobotContainer.navX.getAngle();
    if(currentAngle < targetAngle + Constants.TTA_ERROR && currentAngle > targetAngle - Constants.TTA_ERROR){
      counter ++;
    }
    else{
      counter = 0;
    }

    return counter > Constants.TTA_MAX_COUNTER;
  }
}
