// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.elevatorConstants;
import frc.robot.Constants.motorPorts;
import frc.robot.Constants.elevatorConstants.ElevatorHeight;

public class Elevator extends SubsystemBase {

  SparkFlex frontElevatorMotor;
  SparkFlex backElevatorMotor;
  PIDController elevatorPID;
  double targetPosition;
  ElevatorHeight position;
  SparkClosedLoopController elevatorController;

  public Elevator() {
    frontElevatorMotor = new SparkFlex(motorPorts.frontElevatorMotor, MotorType.kBrushless);
    backElevatorMotor = new SparkFlex(motorPorts.backElevatorMotor, MotorType.kBrushless);
    

    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(60);
    config.closedLoopRampRate(0.5);
    config.idleMode(IdleMode.kBrake);
    config.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(elevatorConstants.p, elevatorConstants.i, elevatorConstants.d)
      .velocityFF(0.018)
      .outputRange(-1, 1);

    frontElevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    config.follow(motorPorts.frontElevatorMotor);
    backElevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    elevatorController = frontElevatorMotor.getClosedLoopController();
    
    switchElevator(ElevatorHeight.INTAKE);
  } 

  public ElevatorHeight getState(){
    return position;
  }

  public void switchElevator(ElevatorHeight position){
    this.position = position;
    switch (position) {
      case INTAKE:
        elevatorController.setReference(elevatorConstants.intake, ControlType.kPosition);
        break;
      case LEVELTWO:
        elevatorController.setReference(elevatorConstants.levelTwo, ControlType.kPosition);
        break;
      case LEVELTHREE:
        elevatorController.setReference(elevatorConstants.levelThree, ControlType.kPosition);
        break;
      case LEVELFOUR:
        elevatorController.setReference(elevatorConstants.levelFour, ControlType.kPosition);
        break;
      default:
        elevatorController.setReference(elevatorConstants.intake, ControlType.kPosition);
    }
  }
  @Override
  public void periodic() {
    SmartDashboard.putString("Elevator Target", position.toString());
    SmartDashboard.putNumber("Applied Output", frontElevatorMotor.getAppliedOutput());
  }
}
