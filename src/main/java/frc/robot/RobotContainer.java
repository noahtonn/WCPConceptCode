// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.Commands.IntakeCoral;
import frc.robot.Commands.OuttakeCoral;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.controllerPorts;
import frc.robot.Constants.elevatorConstants.ElevatorHeight;
import frc.robot.Constants.pivotConstants.PivotPosition;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Index;
import frc.robot.Subsystems.Pivot;

public class RobotContainer {

  XboxController driveController;
  XboxController operatorController;

  private final DriveSubsystem driveSubsystem;
  private final Elevator elevatorSubsystem;
  private final Index indexSubsystem;
  private final Pivot pivotSubsystem;

  boolean toggleTrue = false;

  public RobotContainer() {
    driveController = new XboxController(controllerPorts.driveController);
    operatorController = new XboxController(controllerPorts.operatorController);
    driveSubsystem = new DriveSubsystem();
    elevatorSubsystem = new Elevator();
    indexSubsystem = new Index();
    pivotSubsystem = new Pivot();
    configureBindings();

    driveSubsystem.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> driveSubsystem.drive(
                -MathUtil.applyDeadband(driveController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driveController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driveController.getRightX(), OIConstants.kDriveDeadband),
                true),
                driveSubsystem));
  }

  private void configureBindings() {
    new Trigger(() -> operatorController.getAButton())
      .onTrue(
        new ConditionalCommand(
          new SequentialCommandGroup( //If True
            new InstantCommand(() -> pivotSubsystem.switchPivot(PivotPosition.CORALSCORE)),
            new WaitCommand(0.2),
            new InstantCommand(() -> elevatorSubsystem.switchElevator(ElevatorHeight.INTAKE)),
            new WaitCommand(0.2),
            new InstantCommand(() -> pivotSubsystem.switchPivot(PivotPosition.CORALINTAKE))), 

            new InstantCommand(() -> elevatorSubsystem.switchElevator(ElevatorHeight.INTAKE)), //If false

            () -> pivotSubsystem.getPosition() != PivotPosition.ALGAE)); //Condition

    new Trigger(() -> operatorController.getBButton())
    .onTrue(
      new ConditionalCommand( 
        new SequentialCommandGroup( //If true
          new InstantCommand(() -> pivotSubsystem.switchPivot(PivotPosition.CORALSCORE)),
          new InstantCommand(() -> elevatorSubsystem.switchElevator(ElevatorHeight.LEVELTWO))
        ), 
        
        new InstantCommand(() -> elevatorSubsystem.switchElevator(ElevatorHeight.LEVELTWO)), //If false
        
        () -> pivotSubsystem.getPosition() != PivotPosition.ALGAE)); //Condition

    new Trigger(() -> operatorController.getYButton())
    .onTrue(
      new ConditionalCommand( 
        new SequentialCommandGroup( //If true
          new InstantCommand(() -> pivotSubsystem.switchPivot(PivotPosition.CORALSCORE)),
          new InstantCommand(() -> elevatorSubsystem.switchElevator(ElevatorHeight.LEVELTHREE))
        ), 
        
        new InstantCommand(() -> elevatorSubsystem.switchElevator(ElevatorHeight.LEVELTHREE)), //If false
        
        () -> pivotSubsystem.getPosition() != PivotPosition.ALGAE)); //Condition
    
    new Trigger(() -> operatorController.getXButton())
    .onTrue(
      new ConditionalCommand( 
        new SequentialCommandGroup( //If true
          new InstantCommand(() -> pivotSubsystem.switchPivot(PivotPosition.CORALSCORE)),
          new InstantCommand(() -> elevatorSubsystem.switchElevator(ElevatorHeight.LEVELFOUR))
        ), 
        
        new InstantCommand(() -> elevatorSubsystem.switchElevator(ElevatorHeight.LEVELFOUR)), //If false
        
        () -> pivotSubsystem.getPosition() != PivotPosition.ALGAE)); //Condition

    new Trigger(() -> operatorController.getLeftTriggerAxis() > 0.2)
      .whileTrue(new IntakeCoral(indexSubsystem, pivotSubsystem));
    
    new Trigger(() -> operatorController.getRightTriggerAxis() > 0.2)
      .whileTrue(new OuttakeCoral(indexSubsystem, pivotSubsystem));

    new Trigger(() -> operatorController.getRightBumperButton())
      .onTrue(
        new ConditionalCommand(
          new InstantCommand(() -> pivotSubsystem.switchPivot(PivotPosition.ALGAE)), //If true
          new InstantCommand(() -> pivotSubsystem.switchPivot(PivotPosition.CORALINTAKE)), //If false
          () -> pivotSubsystem.getPosition() != PivotPosition.ALGAE) //Condition
      );

    new Trigger(() -> driveController.getRightBumperButton())
      .whileTrue(new InstantCommand(() -> driveSubsystem.setX()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
