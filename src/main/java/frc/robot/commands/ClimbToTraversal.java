// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbToTraversal extends SequentialCommandGroup {
  /** Creates a new ClimbToTraversal. */
  public ClimbToTraversal(Climber s_climber, Joystick joystick) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MidLift(s_climber),
      new WaitUntilCommand(s_climber::getMidClawEngaged),
      new MidGrab(s_climber),
      new WaitUntilCommand(s_climber::getJoystickButton),
      new MidLockWinch(s_climber),
      new WaitUntilCommand(s_climber::getJoystickButton),
      new MidPull(s_climber),
      new WaitUntilCommand(s_climber::getJoystickButton),
      new TraverseLift(s_climber),
      new WaitUntilCommand(s_climber::getTraverseClawEngaged),
      new TraverseGrab(s_climber),
      new WaitUntilCommand(s_climber::getJoystickButton),
      new ReleaseMidClaw(s_climber)
    );
  }
}
