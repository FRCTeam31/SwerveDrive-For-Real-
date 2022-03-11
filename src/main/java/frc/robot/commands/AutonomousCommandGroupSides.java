// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import PursellJaques.AutonOrientation;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousCommandGroupSides extends SequentialCommandGroup {
  /** Creates a new AutonomousCommandGroup. */
  private int direction;
  public AutonomousCommandGroupSides(AutonOrientation orientation) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    if (orientation == AutonOrientation.LEFT) {
      direction = -1;
    } else if (orientation == AutonOrientation.RIGHT) {
      direction = 1;
    } else {
      System.out.println("stop using this command \nuse the other auton cmd");
      direction = 1/0;
    }
    

    addCommands(
      new WaitCommand(1),
      new BallSuckCommand(1000, 0),
      new TurnAngleCommand(180, 3),
      new AutoBallPickupCommand(),
      new TurnAngleCommand(180, 3),
      new BallSuckCommand(1000, 0)
    );
  }
}
