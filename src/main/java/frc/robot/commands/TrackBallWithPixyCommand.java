// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
  int counter = 0;

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
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get image data from pixy
    Block targetBlock = RobotContainer.pixy.getBiggestBlock(signature);
    // Display block data to smart dashboard
    RobotContainer.pixy.displayBlock(targetBlock);
    
    if(targetBlock == null){
      // No target found, rotate robot in "seeking" mode
      RobotContainer.swerveDrive.drive(0.0, 0.0, Constants.SAFE_TURN_RATE);
      // Reset counter
      counter = 0;
    }
    else{
      // Target is found
      double tx = targetBlock.getX();
      double ty = targetBlock.getY();
      // Update counter
      if(tx < 1 * Constants.TRACK_BALL_WITH_PIXY_COMMAND_ANGLE_TOLERANCE && tx > -1 * Constants.TRACK_BALL_WITH_PIXY_COMMAND_ANGLE_TOLERANCE &&
      ty < Constants.TRACK_BALL_WITH_PIXY_COMMAND_TARGET_TY * (1 + Constants.TRACK_BALL_WITH_PIXY_COMMAND_TY_TOLERANCE) && ty > Constants.TRACK_BALL_WITH_PIXY_COMMAND_TARGET_TY * (1 - Constants.TRACK_BALL_WITH_PIXY_COMMAND_TY_TOLERANCE)){
        // Target is within tolerance for angle and distance, increment counter
        counter ++; 
      }
      else{
        // Target is not within loterance, reset counter
        counter = 0;
      }
      // Calculate swerve outputs
      double anglePower = anglePID.run(tx, Constants.TRACK_BALL_WITH_PIXY_COMMAND_TARGET_TX);
      double distancePower = distancePID.run(ty, Constants.TRACK_BALL_WITH_PIXY_COMMAND_TARGET_TY);
      // Drive Swerve drive
      RobotContainer.swerveDrive.drive(0.0, -1 * distancePower, anglePower);
      SmartDashboard.putNumber("PIXY distance power", distancePower);
      SmartDashboard.putNumber("PIXY angle power", anglePower);
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
    return (counter > Constants.TRACK_BALL_WITH_PIXY_COMMAND_MAX_COUNT);
  }
}
