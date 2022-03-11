// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import PursellJaques.N_PID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class TrackBallCommand extends CommandBase {
  /** Creates a new TrackBallCommand. */
  
  // Instance Variables
  private N_PID distancePID;
  private N_PID anglePID;
  private int checkCount;

  public TrackBallCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    this.distancePID = new N_PID(Constants.TRACK_BALL_COMMAND_DISTANCE_P, Constants.TRACK_BALL_COMMAND_DISTANCE_I, Constants.TRACK_BALL_COMMAND_DISTANCE_D);
    this.anglePID = new N_PID(Constants.TRACK_BALL_COMMAND_ANGLE_P, Constants.TRACK_BALL_COMMAND_ANGLE_I, Constants.TRACK_BALL_COMMAND_ANGLE_D);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Reset PID controller
    distancePID.reset(0);
    anglePID.reset(0);
    // Reset check count
    checkCount = 0;
    // Turn off driveCommand
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get current values
    double x = RobotContainer.limelightTable.getEntry("tx").getDouble(0.0);
    double v = RobotContainer.limelightTable.getEntry("tv").getDouble(0.0);
    double y = RobotContainer.limelightTable.getEntry("ty").getDouble(0.0);

    if(v == 0.0){
      // No valid target found, rotate to 'seek'
      System.out.println("No Target Found");
      RobotContainer.swerveDrive.drive(0, 0, Constants.SAFE_TURN_RATE); 
      anglePID.reset(0);
      distancePID.reset(0);
    }
    else {
      System.out.println("TARGET FOUND!!");
      // Use pid to calculate motor outputs
      double distanceOutput = distancePID.run(y, Constants.TARGET_TY) * Constants.TRACK_BALL_COMMAND_DISTANCE_INVERTER;
      double angleOutput = anglePID.run(x, 0.0) * Constants.TRACK_BALL_COMMAND_ANGLE_INVERTER;
      
      // Drive motors
      RobotContainer.swerveDrive.drive(0, distanceOutput, angleOutput);
    }

    // Printouts
    System.out.println("TY: " + y);
    // This is a change to commit
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Set motors to resting state
    RobotContainer.swerveDrive.stopModulesMotion();
    System.out.println("THE COMMAND IS OVERRRRRRR");
    // Turn on Drive Command
    RobotContainer.turnOnDriveCommands();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Finished when ta is within tolerance for specified number of checks 
    double y = RobotContainer.limelightTable.getEntry("ty").getDouble(0.0);
    double x = RobotContainer.limelightTable.getEntry("tx").getDouble(0.0);
    System.out.println("(" + x + ", " + y + ")");
    if (y < Constants.TARGET_TY + Constants.Y_TOLERANCE && y > Constants.TARGET_TY - Constants.Y_TOLERANCE){
      System.out.println("INSIDE TARGET Y");
      if(x < Constants.ANGLE_TOLERANCE & x > -1 * Constants.ANGLE_TOLERANCE){
        System.out.println("INSIDE TARGET ANGLE ");
        checkCount += 1;
      }

    }
    else{
      checkCount = 0;
    }
    return (checkCount > Constants.CHECKCOUNT);
  }
}