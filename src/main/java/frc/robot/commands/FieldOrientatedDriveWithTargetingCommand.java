// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import PursellJaques.N_PID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class FieldOrientatedDriveWithTargetingCommand extends CommandBase {
  // Instance Variables
  private N_PID anglePID;
  private double prevAngle;

  /** Creates a new FieldOrientatedDriveWithTargetingCommand. */
  public FieldOrientatedDriveWithTargetingCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    anglePID = new N_PID(Constants.FODWTKP, Constants.FODWTKI, Constants.FODWTKD);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    anglePID.reset(0);
    prevAngle = RobotContainer.navX.getAngle();
    RobotContainer.cancelAllExcept(this);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double tv = RobotContainer.limelightTable.getEntry("tv").getDouble(0.0); // 1 when theres a target, 0 if not
    double tx = RobotContainer.limelightTable.getEntry("tx").getDouble(0.0);
    double anglePower;
    if(tv == 0.0){
      // No valud target
      anglePower = Constants.SAFE_TURN_RATE;
    }
    else{
      // There is a valid target
      // Use feed-forward in order to calculate target angle (the angle the robot should face so that a ball shot would score)
      // Using the angular velocity as the feed-forward sensor
      double targetAngle = 0 ;//- angularVelocity * Constants.FODWT_FEED_BACK_CONSTANT; 
      
      anglePower = anglePID.run(tx, targetAngle);
    }

    // Power Swerve Drive
    RobotContainer.swerveDrive.fieldOrientedDrive(RobotContainer.joystick.getX(), RobotContainer.joystick.getY(), anglePower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("THE  FIELD ORIENTATED DRIVE WITH TARGETING COMMAND HAS ENDED___________");
    RobotContainer.swerveDrive.stopModulesMotion();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}