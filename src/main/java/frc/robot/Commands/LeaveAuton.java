// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LeaveAuton extends Command {
  /** Creates a new LeaveAuton. */
  DriveSubsystem driveSubsystem;
  Timer driveTimer;
  public LeaveAuton(DriveSubsystem driveSubsystem) {
    addRequirements(driveSubsystem);
    this.driveSubsystem = driveSubsystem;
    driveTimer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTimer.reset();
    driveTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    while(driveTimer.hasElapsed(5)){
      driveSubsystem.drive(0, 0.15, 0, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return driveTimer.hasElapsed(5);
  }
}
