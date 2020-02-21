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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */

  private CANSparkMax elevatorMotor1 = new CANSparkMax(ElevatorConstants.kElevatorLeftMotor, MotorType.kBrushless);
  private CANSparkMax elevatorMotor2 = new CANSparkMax(ElevatorConstants.kElevatorRightMotor, MotorType.kBrushless);

  private final CANEncoder m_motor1Encoder = new CANEncoder(elevatorMotor1);
  private final CANEncoder m_motor2Encoder = new CANEncoder(elevatorMotor2);

//   private final SpeedControllerGroup elevatorMotors;

  private static PIDController kElevatorPIDController = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);

  private static boolean pidEnabled = false;
  private static double setpoint = 0.0;

  public ElevatorSubsystem() {
      elevatorMotor1.restoreFactoryDefaults();
      elevatorMotor2.restoreFactoryDefaults();

      elevatorMotor2.setInverted(true);
      // elevatorMotor2.setInverted(true);

      elevatorMotor1.setOpenLoopRampRate(0.5);
      elevatorMotor2.setOpenLoopRampRate(0.5);

      
    //   elevatorMotors = new SpeedControllerGroup(elevatorMotor1, elevatorMotor2);

      if (pidEnabled) {
        // elevatorMotors.set(kElevatorPIDController.calculate(getAvgEncoderPos(), setpoint));
      }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Elevator Height", getAvgEncoderPos());
  }

  public void moveManual(double speed) {
    // if (inLimits(speed)) {
    //   elevatorMotors.set(speed * ElevatorConstants.speedMultiplier);
    // }
        elevatorMotor1.set(speed);
        elevatorMotor2.set(speed);
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

  public void setPos(double climbHeight) {
    setpoint = climbHeight;
  }

  private double getAvgEncoderPos() {
    return (m_motor1Encoder.getPosition() + m_motor2Encoder.getPosition()) / 2.0;
  }

  public void resetPosition()
  {
      m_motor1Encoder.setPosition(0);
      m_motor2Encoder.setPosition(0);
  }

}
