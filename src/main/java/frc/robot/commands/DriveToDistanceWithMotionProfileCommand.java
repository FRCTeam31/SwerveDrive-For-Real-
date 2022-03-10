// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import com.ctre.phoenix.motorcontrol.NeutralMode;

import PursellJaques.FalconFXSwerveModule;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/**
 * A command that controls the robot so that it travels a specified distance
 * using a trapezoidal profile + PID controller
 */
public class DriveToDistanceWithMotionProfileCommand extends CommandBase {
  // Instance Variables
  public ProfiledPIDController distancePIDController;
  public double targetDistance;
  public boolean done;
  public int atTargetDistanceCounter;


  /** Creates a new DriveToDistanceWithMotionProfileCommand. 
   * @param targetDistance the distance you want the robot to travel in meters
  */
  public DriveToDistanceWithMotionProfileCommand(double targetDistance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.targetDistance = targetDistance * Constants.SWERVE_DRIVE_METERS_TO_TICKS;
    // Create a pid controller that also uses a trapezoidal profile
    distancePIDController = new ProfiledPIDController(Constants.DRIVE_DISTANCE_COMMAND_WITH_MOTION_PROFILE_KP,
                                                      Constants.DRIVE_DISTANCE_COMMAND_WITH_MOTION_PROFILE_KI,
                                                      Constants.DRIVE_DISTANCE_COMMAND_WITH_MOTION_PROFILE_KD,
                                                      new TrapezoidProfile.Constraints(Constants.DRIVE_DISTANCE_COMMAND_WITH_MOTION_MAX_VELOCITY, 
                                                                          Constants.DRIVE_DISTANCE_COMMAND_WITH_MOTION_PROFILE_MAX_ACCELERATION),
                                                      Constants.ROBOT_LOOPING_PERIOD);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Reset all swerveModule drive motors to be at distance 0
    for(FalconFXSwerveModule module: RobotContainer.swerveDrive.swerveModules){
      module.driveMotor.setSelectedSensorPosition(0.0);
    }
    RobotContainer.swerveDrive.setNeutralMode(NeutralMode.Brake);
    done = false;
    atTargetDistanceCounter = 0;
  } 

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentPos = getCurrentDistance();
    RobotContainer.swerveDrive.drive(0.0, distancePIDController.calculate(currentPos, targetDistance), 0.0);
    if(currentPos < targetDistance * (1 + Constants.DRIVE_DISTANCE_COMMAND_WITH_MOTION_PROFILE_TOLERANCE) &&
    currentPos > targetDistance * (1 - Constants.DRIVE_DISTANCE_COMMAND_WITH_MOTION_PROFILE_TOLERANCE)){
      // Current pos is inside target tolerances
      atTargetDistanceCounter ++;
    }
    else{
      atTargetDistanceCounter = 0;
    }
    done = atTargetDistanceCounter > Constants.DRIVE_DISTANCE_COMMAND_WITH_MOTION_PROFILE_MAX_COUNTER;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.swerveDrive.stopModulesMotion();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }

  /**
   * 
   * @return the average distance a wheel has traveled in ticks
   */
  public double getCurrentDistance(){
    double sensorAccumulator = 0.0;
    for(FalconFXSwerveModule module: RobotContainer.swerveDrive.swerveModules){
      sensorAccumulator += module.driveMotor.getSelectedSensorPosition();
    }
    
    double avgPosition = sensorAccumulator / RobotContainer.swerveDrive.swerveModules.length;
    return avgPosition;
  }
}
