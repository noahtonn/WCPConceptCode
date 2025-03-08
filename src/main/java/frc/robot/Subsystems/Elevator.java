// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

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

  public Elevator() {
    frontElevatorMotor = new SparkFlex(motorPorts.frontElevatorMotor, MotorType.kBrushless);
    backElevatorMotor = new SparkFlex(motorPorts.backElevatorMotor, MotorType.kBrushless);
    

    SparkMaxConfig sConfig = new SparkMaxConfig();
    sConfig.smartCurrentLimit(60);
    frontElevatorMotor.configure(sConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    sConfig.follow(motorPorts.frontElevatorMotor);
    backElevatorMotor.configure(sConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    elevatorPID = new PIDController(elevatorConstants.p, elevatorConstants.i, elevatorConstants.d);
    elevatorPID.setTolerance(elevatorConstants.elevatorTolerance);

    switchElevator(ElevatorHeight.INTAKE);
  } 

  public ElevatorHeight getState(){
    return position;
  }

  public void switchElevator(ElevatorHeight position){
    this.position = position;
    switch (position) {
      case INTAKE:
        targetPosition = elevatorConstants.intake;
        break;
      case LEVELTWO:
        targetPosition = elevatorConstants.levelTwo;
        break;
      case LEVELTHREE:
        targetPosition = elevatorConstants.levelThree;
        break;
      case LEVELFOUR:
        targetPosition = elevatorConstants.levelFour;
        break;
      default:
        targetPosition = elevatorConstants.intake;
    }
  }
  @Override
  public void periodic() {
    double input = elevatorPID.calculate(frontElevatorMotor.getEncoder().getPosition(), targetPosition);
    input = input >= 1 ? 1 : input;
    SmartDashboard.putNumber("Elevator Power", input);
    frontElevatorMotor.set(input);
  }
}
