// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.RomiDrivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class Backwards extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final RomiDrivetrain m_subsystem;
  
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Backwards(RomiDrivetrain subsystem, int backwards) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    //m_subsystem.getRightDistanceInch()
    addRequirements(subsystem);

    m_backwards = backwards;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //it runs before the command the starts
    //reset encoders
    m_subsystem.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //runs the command
    //move backwards
    m_subsystem.arcadeDrive(-.5, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //runs when the command ends
    //STOP
    m_subsystem.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //constantly checking if a statement is true or false
    //return true when encoders read 5 inches
    return (m_subsystem.getAverageDistanceInch() >= 5);
  }
}
