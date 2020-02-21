/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShooterConstants;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {

    public enum ShooterAngle {
        UP, DOWN
    }

    private final CANSparkMax bottomMotor = new CANSparkMax(ShooterConstants.kBottomMotorPort, MotorType.kBrushless);
    private final CANSparkMax topMotor = new CANSparkMax(ShooterConstants.kTopMotorPort, MotorType.kBrushless);

    private final CANEncoder bottomEncoder = new CANEncoder(bottomMotor);
    private final CANEncoder topEncoder = new CANEncoder(topMotor);

    private PIDController kShooterPIDControllerBottom = new PIDController(ShooterConstants.kP, ShooterConstants.kI,
            ShooterConstants.kD);
    private PIDController kShooterPIDControllerTop = new PIDController(ShooterConstants.kP, ShooterConstants.kI,
            ShooterConstants.kD);

    private static double kRpmBottom = 0;
    private static double kRpmTop = 0;

    private DoubleSolenoid shooterSolenoid = new DoubleSolenoid(0, 1);

    public boolean pidEnabled = false;

    public ShooterSubsystem() {

        shooterSolenoid.set(DoubleSolenoid.Value.kForward);

        SmartDashboard.putNumber("SetBottomRPM", 0);
        SmartDashboard.putNumber("SetTopRPM", 0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        updateSmartdashboard();

        if (pidEnabled) {
            calculatePID();
        }
    }

    public void setRPM(double rpmBottom, double rpmTop) {
        kRpmBottom = rpmBottom;
        kRpmTop = rpmTop;
    }

    public void setManual(double bottomValue, double topValue) {
        bottomMotor.set(bottomValue);
        topMotor.set(topValue);
    }

    public void toggleShooterAngle(ShooterAngle angle) {
        switch (angle) {
        case UP:
            shooterSolenoid.set(DoubleSolenoid.Value.kForward);
            break;

        case DOWN:
            shooterSolenoid.set(DoubleSolenoid.Value.kReverse);
            break;
        }
    }

    public void calculatePID() {
        double errBottom = kShooterPIDControllerBottom.calculate(bottomEncoder.getVelocity(), kRpmBottom);
        double errTop = kShooterPIDControllerTop.calculate(topEncoder.getVelocity(), -kRpmTop); // Top runs backwards,
                                                                                                // so target is negative

        double ffBottom = ShooterConstants.kFF * kRpmBottom;
        double ffTop = ShooterConstants.kFF * -kRpmTop; // Top runs backwards

        bottomMotor.set(errBottom + ffBottom);
        topMotor.set(errTop + ffTop);

        SmartDashboard.putNumber("SetValueBottom", ffBottom);
        SmartDashboard.putNumber("SetValueTop", ffTop);
    }

    public void updateSmartdashboard() {
        SmartDashboard.putNumber("BottomRPM", bottomEncoder.getVelocity());
        SmartDashboard.putNumber("TopRPM", topEncoder.getVelocity());

        SmartDashboard.putNumber("TargetRPMBottom", kRpmBottom);
        SmartDashboard.putNumber("TargetRPMTop", kRpmTop);
    }
}
