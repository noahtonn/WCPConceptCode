// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.pivotConstants.PivotPosition;
import frc.robot.Subsystems.Index;
import frc.robot.Subsystems.Pivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCoral extends Command {

  Index indexSubsystem;
  Pivot pivotSubsystem;
  int tick;

  public IntakeCoral(Index indexSubsystem, Pivot pivotSubsystem) {
    addRequirements(indexSubsystem, pivotSubsystem);
    this.indexSubsystem = indexSubsystem;
    this.pivotSubsystem = pivotSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(pivotSubsystem.getPosition() == PivotPosition.CORALINTAKE){
      if(!indexSubsystem.getSensor()){
        indexSubsystem.runMotor(true);
      }else{
        indexSubsystem.stopMotor();
      }
    }else{
      indexSubsystem.runMotor(false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexSubsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
