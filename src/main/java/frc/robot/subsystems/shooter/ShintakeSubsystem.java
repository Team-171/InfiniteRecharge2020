/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShintakeConstants;

public class ShintakeSubsystem extends SubsystemBase {

  private boolean allowBall = true;
  private CANSparkMax topMotor = new CANSparkMax(ShintakeConstants.kUpperMotorPort, MotorType.kBrushless);
  private CANSparkMax bottomMotor = new CANSparkMax(ShintakeConstants.kLowerMotorPort, MotorType.kBrushless);
  private AnalogInput topProxSwitch = new AnalogInput(ShintakeConstants.kTopProxSwitch);
  private AnalogInput bottomProxSwitch = new AnalogInput(ShintakeConstants.kBottomProxSwitch);

  public ShintakeSubsystem() {
        topMotor.setClosedLoopRampRate(0.25);
        bottomMotor.setClosedLoopRampRate(0.25);
  }

  public void drive(double speed)
  {
      if(speed > 0)
      {
          topMotor.set(speed * .5);

          if(allowBall)
          {
              bottomMotor.set(speed);
          }
      }
      else
      {
          topMotor.set(speed * .5);
          bottomMotor.set(speed);
      }
  }

  public void driveTop(double speed){
      topMotor.set(speed * 0.4);
  }

  public void driveBottom(double speed){
      bottomMotor.set(speed);
  }

  @Override
  public void periodic() {
    //   if(bottomProxSwitch.get && allowBall)
    //   {
    //       allowBall = false;
    //   }

    //   if(topProxSwitch.get() && !allowBall)
    //   {
    //       allowBall = true;
    //   }

    //   SmartDashboard.putNumber("TopShintake", topProxSwitch.getVoltage());
  }
}
