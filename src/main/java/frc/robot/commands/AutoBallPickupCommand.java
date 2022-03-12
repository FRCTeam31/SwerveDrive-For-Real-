// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBallPickupCommand extends SequentialCommandGroup {
  /** Creates a new AutoBallPickupCommand. */
  public AutoBallPickupCommand() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new TrackBallWithPixyCommand(Constants.BALL_PROFILE, 5),
      new BallSuckCommand(1500, 1000)
    );
  }
  public AutoBallPickupCommand(double driveTime, double suckTime) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new TrackBallWithPixyCommand(Constants.BALL_PROFILE, 5),
      new BallSuckCommand(driveTime, suckTime)
    );
  }
}
