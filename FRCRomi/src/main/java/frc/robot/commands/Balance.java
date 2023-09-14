// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.RomiDrivetrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;

/** An example command that uses an example subsystem. */
public class Balance extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final RomiDrivetrain m_subsystem;
  private final PIDController m_PidBalance = new PIDController(0, 0, 0);
  private boolean backwards;
  
  private boolean onRamp;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Balance(RomiDrivetrain subsystem, boolean backwards) {
    m_subsystem = subsystem;
    this.backwards = backwards;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(backwards == false && m_suubsystem.getAngleY() >= 11){
      onRamp = true;
    } 
    else if(backwards == true && m_sbsystem.getAngleY() >= -11){
      onRamp = true;
    }
    if(onRamp){
      m_subsystem.arcadeDrive(m_PidBalance.calculate(m_subsystem.getAngleY(), 0), m_PidBalance.calculate(m_subsystem.getAngleZ(), 0));
    }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
