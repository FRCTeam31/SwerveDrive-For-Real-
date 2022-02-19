// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import PursellJaques.N_PID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AimAtTargetCommand extends CommandBase {
  // Instance Variable
   private int count;
   N_PID ballCenterPID;
   private double range = 30;

  /** Creates a new AimAtTargetCommand. */
  public AimAtTargetCommand(){
    // Use addRequirements() here to declare subsystem dependencies.
    ballCenterPID = new N_PID(Constants.GOAL_AIMING_KP, Constants.GOAL_AIMING_KI, Constants.GOAL_AIMING_KD);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    count = 0;
    ballCenterPID.reset(0);
    // Turn off driveCommand
    RobotContainer.turnOffDriveCommands();
    SmartDashboard.putString("Aim At Target Command", "ACTIVE");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double tx = RobotContainer.limelightTable.getEntry("tx").getDouble(0.0);
    double tv = RobotContainer.limelightTable.getEntry("tv").getDouble(0.0); // Valid target: 0=false, 1=true
    if(tv == 1){
      // Valid Target
      RobotContainer.swerveDrive.drive(0, 0, ballCenterPID.run(tx, 0));
    }
    else{
      // No Valid Target
      RobotContainer.swerveDrive.drive(0, 0, Constants.SAFE_TURN_RATE);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.swerveDrive.stopModulesMotion();
    // Turn on driveCommand
    RobotContainer.turnOnDriveCommands();
    SmartDashboard.putString("Aim At Target Command", "NOT ACTIVE");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double tx = RobotContainer.limelightTable.getEntry("tx").getDouble(0.0);

    if(tx < Constants.GOAL_AIM_TOLERANCE * range && tx > Constants.GOAL_AIM_TOLERANCE * range * -1){
      count ++;
    }
    else{
      count = 0;
    }
    return (count > Constants.GOAL_CENTER_TIMER);
  }
}