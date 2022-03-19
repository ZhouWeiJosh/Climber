// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class Climb extends CommandBase {
  Climber s_climber;
  /** Creates a new Climb. */
  public Climb(Climber s_climber) {
    this.s_climber = s_climber;
    addRequirements(s_climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   // s_climber.resetEncoderValues();
    s_climber.incrementCount();  
    s_climber.resetBooleans();

  }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_climber.climbToTraversal();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //s_climber.resetEncoderValues();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return s_climber.getActionFinished();
  }
}
