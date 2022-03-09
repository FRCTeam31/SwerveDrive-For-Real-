// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import com.ctre.phoenix.motorcontrol.NeutralMode;

import PursellJaques.FalconFXSwerveModule;
import PursellJaques.N_PID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveDistanceCommand extends CommandBase {
  // Instance variables
  private double targetDistance;
  private N_PID distancePIDController;
  private double counter;

  /** Creates a new DriveDistanceCommand. */
  public DriveDistanceCommand(double targetDistance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.targetDistance = targetDistance;
    distancePIDController = new N_PID(Constants.DDC_KP, Constants.DDC_KI, Constants.DDC_KD);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    for(FalconFXSwerveModule swerveModule: RobotContainer.swerveDrive.swerveModules){
      swerveModule.driveMotor.setSelectedSensorPosition(0.0);
    }
    distancePIDController.reset(0);
    counter = 0;
    RobotContainer.cancelAllExcept(this);
    RobotContainer.swerveDrive.setNeutralMode(NeutralMode.Brake);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Calculate average sensor position
    double sensorAccumulator = 0;
    for(FalconFXSwerveModule swerveModule: RobotContainer.swerveDrive.swerveModules){
      sensorAccumulator += swerveModule.driveMotor.getSelectedSensorPosition();
    }
    sensorAccumulator /= RobotContainer.swerveDrive.swerveModules.length;

    // Drive forward using PID
    RobotContainer.swerveDrive.drive(0, -1 * distancePIDController.run(sensorAccumulator, targetDistance), 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.swerveDrive.stopModulesMotion();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Get average sensor position
    double sensorAccumulator = 0;
    for(FalconFXSwerveModule swerveModule: RobotContainer.swerveDrive.swerveModules){
      sensorAccumulator += swerveModule.driveMotor.getSelectedSensorPosition();
    }
    sensorAccumulator /= RobotContainer.swerveDrive.swerveModules.length;

    if(sensorAccumulator < targetDistance * (1 + Constants.DDC_TOLERANCE) && sensorAccumulator > targetDistance * (1 - Constants.DDC_TOLERANCE)){
      counter ++;
      System.out.println("INSIDE TARGET DISTANCE");
    }
    else{
      counter = 0;
    }
    SmartDashboard.putNumber("Current Motor Positions", sensorAccumulator);
    return (counter > Constants.DDC_MAX_COUNT);
  }
}
