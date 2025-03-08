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
import frc.robot.Commands.IntakeCoral;
import frc.robot.Commands.OuttakeCoral;
import frc.robot.Commands.moveElevator;
import frc.robot.Commands.safeElevator;
import frc.robot.Commands.switchCoralIntake;
import frc.robot.Commands.switchPivot;
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

  public Command switchPivot(){
    if(pivotSubsystem.getPosition() != PivotPosition.ALGAE){
      return new InstantCommand(() -> pivotSubsystem.switchPivot(PivotPosition.ALGAE));
    }else{
      return new InstantCommand(() -> pivotSubsystem.switchPivot(PivotPosition.CORALINTAKE));
    }
  }

  public Command changeElevator(ElevatorHeight height){
    if(pivotSubsystem.getPosition() != PivotPosition.ALGAE){
      return new SequentialCommandGroup(
        new InstantCommand(() -> pivotSubsystem.switchPivot(PivotPosition.CORALSCORE)),
        new WaitCommand(0.15),
        new InstantCommand(() -> elevatorSubsystem.switchElevator(height)));
    }else{
      return new InstantCommand(() -> elevatorSubsystem.switchElevator(height));
    }
  }

  public Command changeElevatorIntake(){
    if(pivotSubsystem.getPosition() != PivotPosition.ALGAE){
      return new SequentialCommandGroup(
        new InstantCommand(() -> this.pivotSubsystem.switchPivot(PivotPosition.CORALSCORE)),
        new WaitCommand(0.15),
        new InstantCommand(() -> this.elevatorSubsystem.switchElevator(ElevatorHeight.INTAKE)),
        new WaitCommand(0.2),
        new InstantCommand(() -> this.pivotSubsystem.switchPivot(PivotPosition.CORALINTAKE)));
    }else{
      return new InstantCommand(() -> elevatorSubsystem.switchElevator(ElevatorHeight.INTAKE));
    }
  }

  private void configureBindings() {
    new Trigger(() -> operatorController.getAButton())
      .onTrue(new SequentialCommandGroup(
        new safeElevator(pivotSubsystem, elevatorSubsystem),
        new WaitCommand(0.2),
        new moveElevator(elevatorSubsystem, ElevatorHeight.INTAKE),
        new WaitCommand(0.2),
        new switchCoralIntake(pivotSubsystem)
      ));

    new Trigger(() -> operatorController.getBButton())
      .onTrue(new SequentialCommandGroup(
        new safeElevator(pivotSubsystem, elevatorSubsystem),
        new WaitCommand(0.2),
        new moveElevator(elevatorSubsystem, ElevatorHeight.LEVELTWO)
      ));

    new Trigger(() -> operatorController.getYButton())
      .onTrue(this.changeElevator(ElevatorHeight.LEVELTHREE));
    
    new Trigger(() -> operatorController.getXButton())
      .onTrue(this.changeElevator(ElevatorHeight.LEVELFOUR));

    new Trigger(() -> operatorController.getLeftTriggerAxis() > 0.2)
      .whileTrue(new IntakeCoral(indexSubsystem, pivotSubsystem));
    
    new Trigger(() -> operatorController.getRightTriggerAxis() > 0.2)
      .whileTrue(new OuttakeCoral(indexSubsystem, pivotSubsystem));

    new Trigger(() -> operatorController.getRightBumperButton())
      .onTrue(new switchPivot(pivotSubsystem));

    new Trigger(() -> driveController.getRightBumperButton())
      .whileTrue(new InstantCommand(() -> driveSubsystem.setX()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
