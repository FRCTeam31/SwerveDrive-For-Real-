// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.fasterxml.jackson.annotation.JsonTypeInfo.None;

import PursellJaques.N_PID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;

/**
 * A command to make the robot track a ball with the pixy
 */
public class TrackBallWithPixyCommand extends CommandBase {
  // Instance Variables
  int signature;
  N_PID anglePID;
  N_PID distancePID;
  int validPostionCounter = 0;
  int noTargetCounter = 0;
  Double prevX = null;
  Double prevY = null;
  Double avgX = null;
  Double avgY = null;
  Double prevX2 = null;
  Double prevY2 = null;


  /** Creates a new TrackBallWithPixyCommand. 
   * @param signature the signature (profile) you want to track
  */
  public TrackBallWithPixyCommand(int signature) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.signature = signature;

    // Set up PID
    anglePID = new N_PID(Constants.TRACK_BALL_WITH_PIXY_COMMAND_ANGLE_KP, Constants.TRACK_BALL_WITH_PIXY_COMMAND_ANGLE_KI, Constants.TRACK_BALL_WITH_PIXY_COMMAND_ANGLE_KD);
    distancePID = new N_PID(Constants.TRACK_BALL_WITH_PIXY_COMMAND_DISTANCE_KP, Constants.TRACK_BALL_WITH_PIXY_COMMAND_DISTANCE_KI, Constants.TRACK_BALL_WITH_PIXY_COMMAND_DISTANCE_ANGLE_KD);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Reset PID
    anglePID.reset(0);
    distancePID.reset(0);
    validPostionCounter = 0;
    noTargetCounter = 0;
    avgX = Constants.TRACK_BALL_WITH_PIXY_COMMAND_TARGET_TX;
    avgY = Constants.TRACK_BALL_WITH_PIXY_COMMAND_TARGET_TY;
    prevX = avgX;
    prevY = avgY;
    prevX2 = prevX;
    prevY2 = prevY;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Declare instance variables
    Double tx = null;
    Double ty = null;
    boolean driveBot = false;

    // Get image data from pixy
    Block targetBlock = RobotContainer.pixy.getBiggestBlock(signature);
    // Display block data to smart dashboard
    RobotContainer.pixy.displayBlock(targetBlock);
    
    if(targetBlock == null){
      // No target found, rotate robot in "seeking" mode
      noTargetCounter ++;

      if(noTargetCounter < Constants.ACCECPTABLE_NO_TARGET_COUNT){
        // Target may have missed a frame, act like normal with interpolated data
        tx = prevX + (prevX - prevX2);
        ty = prevY + (prevY - prevY2);
        driveBot = true;

      }
      else{
        RobotContainer.swerveDrive.drive(0.0, 0.0, Constants.SAFE_TURN_RATE);
        // Reset validPostionCounter
        validPostionCounter = 0;
      }
  
    }
    else{
      // Target is found
      tx = (double) targetBlock.getX();
      ty = (double) targetBlock.getY();
      driveBot = true;
      noTargetCounter = 0;
    }

    if(driveBot){
      // Calculate avgX and avgY (this will be used as the current values in order to smooth out the data)
      avgX += Constants.TRACK_BALL_WITH_PIXY_ALPHA * (tx - avgX);
      avgY += Constants.TRACK_BALL_WITH_PIXY_ALPHA * (ty - avgY);
      SmartDashboard.putNumber("AVGX", avgX);
      SmartDashboard.putNumber("AVGY", avgY);

      // Update validPostionCounter
      if(avgX < Constants.TRACK_BALL_WITH_PIXY_COMMAND_TARGET_TX * (1 + Constants.TRACK_BALL_WITH_PIXY_COMMAND_ANGLE_TOLERANCE) && avgX > Constants.TRACK_BALL_WITH_PIXY_COMMAND_TARGET_TX * (1 - Constants.TRACK_BALL_WITH_PIXY_COMMAND_ANGLE_TOLERANCE) &&
      avgY < Constants.TRACK_BALL_WITH_PIXY_COMMAND_TARGET_TY * (1 + Constants.TRACK_BALL_WITH_PIXY_COMMAND_TY_TOLERANCE) && avgY > Constants.TRACK_BALL_WITH_PIXY_COMMAND_TARGET_TY * (1 - Constants.TRACK_BALL_WITH_PIXY_COMMAND_TY_TOLERANCE)){
        // Target is within tolerance for angle and distance, increment validPostionCounter
        validPostionCounter ++; 
      }
      else{
        // Target is not within loterance, reset validPostionCounter
        validPostionCounter = 0;
      }
      // Calculate swerve outputs
      double anglePower = anglePID.run(avgX, Constants.TRACK_BALL_WITH_PIXY_COMMAND_TARGET_TX);
      double distancePower = distancePID.run(avgY, Constants.TRACK_BALL_WITH_PIXY_COMMAND_TARGET_TY);
      // Drive Swerve drive
      RobotContainer.swerveDrive.drive(0.0, -1 * distancePower, -1 * anglePower);
      SmartDashboard.putNumber("PIXY distance power", distancePower);
      SmartDashboard.putNumber("PIXY angle power", anglePower);
      SmartDashboard.putNumber("Valid Pos Counter", validPostionCounter);

      // Update values
      prevX2 = prevX;
      prevX = avgX;
      prevY2 = prevY;
      prevY = avgY;
    }


  };

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.swerveDrive.stopModulesMotion();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (validPostionCounter > Constants.TRACK_BALL_WITH_PIXY_COMMAND_MAX_COUNT);
  }
}
