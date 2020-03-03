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

import java.util.EnumSet;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.SmartRunner;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    /**
     * Creates a new ExampleSubsystem.
     */

    private CANSparkMax elevatorMotor1 = new CANSparkMax(ElevatorConstants.kElevatorLeftMotor, MotorType.kBrushless);
    private CANSparkMax elevatorMotor2 = new CANSparkMax(ElevatorConstants.kElevatorRightMotor, MotorType.kBrushless);

    private final CANEncoder m_motor1Encoder = new CANEncoder(elevatorMotor1);
    private final CANEncoder m_motor2Encoder = new CANEncoder(elevatorMotor2);

    // private final SpeedControllerGroup elevatorMotors;

    private PIDController kElevatorPIDController = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);

    private boolean pidEnabled = false;
    // private double setpoint = 0.0;

    private boolean zeroed = false;
    private boolean overridden = false;

    public ElevatorSubsystem() {
        elevatorMotor1.restoreFactoryDefaults();
        elevatorMotor2.restoreFactoryDefaults();

        elevatorMotor1.setInverted(true);
        // elevatorMotor2.setInverted(true);

        elevatorMotor1.setOpenLoopRampRate(0.5);
        // elevatorMotor2.setOpenLoopRampRate(0.5);

        elevatorMotor2.follow(elevatorMotor1, true);
       
        kElevatorPIDController.setSetpoint(0);
        kElevatorPIDController.setTolerance(0.1);
        kElevatorPIDController.setIntegratorRange(-0.1, 0.1);
    }

    @Override
    public void periodic() {
        SmartRunner.Logger.getNumber("Elevator P", (value) -> {
            kElevatorPIDController.setP(value);
        }, EnumSet.of(SmartRunner.RunLevel.ELEVATOR_TUNING), ElevatorConstants.kP);

        SmartRunner.Logger.getNumber("Elevator I", (value) -> {
            kElevatorPIDController.setI(value);
        }, EnumSet.of(SmartRunner.RunLevel.ELEVATOR_TUNING), ElevatorConstants.kI);

        SmartRunner.Logger.getNumber("Elevator D", (value) -> {
            kElevatorPIDController.setD(value);
        }, EnumSet.of(SmartRunner.RunLevel.ELEVATOR_TUNING), ElevatorConstants.kD);

        if(!overridden)
        {
            moveManual(kElevatorPIDController.calculate(getAvgEncoderPos()));
        }

        SmartRunner.Logger.put("Elevator Height", getAvgEncoderPos(), EnumSet.of(SmartRunner.RunLevel.DEBUG, SmartRunner.RunLevel.ELEVATOR_TUNING));
        SmartRunner.Logger.put("Elevator Speed", elevatorMotor1.get(), EnumSet.of(SmartRunner.RunLevel.DEBUG, SmartRunner.RunLevel.ELEVATOR_TUNING));
    }

    public void moveElevator(double speed) {
        if(speed != 0)
        {
            overridden = true;
            moveManual(speed);
        }
        else
        {
            if(overridden)
            {
                kElevatorPIDController.setSetpoint(getAvgEncoderPos());//when the controller is let off the PID loop tries to keep the current height
            }
            overridden = false;
        }
    }

    private void moveManual(double speed) {
        if (inLimits(speed)) {
            elevatorMotor1.set(speed * (speed > 0 ? ElevatorConstants.forwardSpeedMultiplier : ElevatorConstants.reverseSpeedMultiplier));
        }
        else
        {
            elevatorMotor1.set(0);
        }

        SmartRunner.Logger.put("In Limit", inLimits(speed), EnumSet.of(SmartRunner.RunLevel.DEBUG));
    }

    public boolean inLimits(double speed) {
        // TODO: Add velocity check to elevator limit calculation
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
        kElevatorPIDController.setSetpoint(climbHeight);
    }

    private double getAvgEncoderPos() {
        return (m_motor1Encoder.getPosition() + m_motor2Encoder.getPosition()) / 2.0;
    }

    public void resetPosition() {
        if(!zeroed)
        {
            m_motor1Encoder.setPosition(0);
            m_motor2Encoder.setPosition(0);
            zeroed = true;
        }
    }

}
