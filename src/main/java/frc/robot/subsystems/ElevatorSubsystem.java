/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */

  private final CANSparkMax elevatorMotor1 = new CANSparkMax(ElevatorConstants.kElevatorMotorPort1, MotorType.kBrushless);
  private final CANSparkMax elevatorMotor2= new CANSparkMax(ElevatorConstants.kElevatorMotorPort2, MotorType.kBrushless);

  private final CANEncoder m_motor1Encoder = new CANEncoder(elevatorMotor1);
  private final CANEncoder m_motor2Encoder = new CANEncoder(elevatorMotor2);

  private final SpeedControllerGroup elevatorMotors = new SpeedControllerGroup(elevatorMotor1, elevatorMotor2);

  private static PIDController kElevatorPIDController = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);

  private static boolean pidEnabled = true;
  private static double setpoint = 0.0;

  public ElevatorSubsystem() {
    if (pidEnabled) {
      elevatorMotors.set(kElevatorPIDController.calculate(getAvgEncoderPos(), setpoint));
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void moveManual(double speed) {
    if (inLimits(speed)) {
      elevatorMotors.set(speed * ElevatorConstants.speedMultiplier);
    }

  }

  public boolean inLimits(double speed) {
    //TODO: Add velocity check to elevator limit calculation
    if (speed >= 0) {
      if (getAvgEncoderPos() < ElevatorConstants.maxHeight) {
        return true;
      }
    } else {
      if (getAvgEncoderPos() > ElevatorConstants.minHeight) {
        return true;
      }
    }

    return false;
  }

  public void setPos(double climbHieght) {
    setpoint = climbHieght;
  }

  private double getAvgEncoderPos() {
    return (m_motor1Encoder.getPosition() + m_motor2Encoder.getPosition()) / 2.0;
  }

}
