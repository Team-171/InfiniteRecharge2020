/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SmartRunner;
import frc.robot.Constants.ShooterConstants;

import java.util.EnumSet;

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

    private PIDController kShooterPIDControllerBottom = new PIDController(ShooterConstants.kPTop, ShooterConstants.kITop,
            ShooterConstants.kDTop);
    private PIDController kShooterPIDControllerTop = new PIDController(ShooterConstants.kPTop, ShooterConstants.kITop,
            ShooterConstants.kDTop);

    private DoubleSolenoid shooterSolenoid = new DoubleSolenoid(0, 1);

    public boolean pidEnabled = false;

    public ShooterSubsystem() {
        topMotor.restoreFactoryDefaults();
        // topMotor.setOpenLoopRampRate(0.1);
        
        bottomMotor.restoreFactoryDefaults();
        bottomMotor.setInverted(true);
        // bottomMotor.setOpenLoopRampRate(0.1);

        shooterSolenoid.set(DoubleSolenoid.Value.kForward);

        kShooterPIDControllerBottom.setIntegratorRange(-0.08, 0.08);
        kShooterPIDControllerTop.setIntegratorRange(-0.08, 0.08);

        kShooterPIDControllerTop.setTolerance(ShooterConstants.rpmTolerance);
        kShooterPIDControllerBottom.setTolerance(ShooterConstants.rpmTolerance);

        kShooterPIDControllerTop.setSetpoint(ShooterConstants.rpmTop);
        kShooterPIDControllerBottom.setSetpoint(ShooterConstants.rpmBottom);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartRunner.Logger.getNumber("ShooterPTop", (value) -> {
            kShooterPIDControllerTop.setP(value);
        }, EnumSet.of(SmartRunner.RunLevel.SHOOTER_TUNING), ShooterConstants.kPTop);

        SmartRunner.Logger.getNumber("ShooterITop", (value) -> {
            kShooterPIDControllerTop.setI(value);
        }, EnumSet.of(SmartRunner.RunLevel.SHOOTER_TUNING), ShooterConstants.kITop);
        
        SmartRunner.Logger.getNumber("ShooterDTop", (value) -> {
            kShooterPIDControllerTop.setD(value);
        }, EnumSet.of(SmartRunner.RunLevel.SHOOTER_TUNING), ShooterConstants.kDTop);


        SmartRunner.Logger.getNumber("ShooterPBottom", (value) -> {
            kShooterPIDControllerBottom.setP(value);
        }, EnumSet.of(SmartRunner.RunLevel.SHOOTER_TUNING), ShooterConstants.kPBottom);

        SmartRunner.Logger.getNumber("ShooterIBottom", (value) -> {
            kShooterPIDControllerBottom.setI(value);
        }, EnumSet.of(SmartRunner.RunLevel.SHOOTER_TUNING), ShooterConstants.kIBottom);
        
        SmartRunner.Logger.getNumber("ShooterDBottom", (value) -> {
            kShooterPIDControllerBottom.setD(value);
        }, EnumSet.of(SmartRunner.RunLevel.SHOOTER_TUNING), ShooterConstants.kDBottom);
        

        SmartRunner.Logger.getNumber("SetBottomRPM", (value) -> {
            kShooterPIDControllerBottom.setSetpoint(value);
        }, EnumSet.of(SmartRunner.RunLevel.SHOOTER_TUNING), 0);
        
        SmartRunner.Logger.getNumber("SetTopRPM", (value) -> {
            kShooterPIDControllerTop.setSetpoint(value);
        }, EnumSet.of(SmartRunner.RunLevel.SHOOTER_TUNING), 0);
        

        SmartRunner.Logger.getNumber("ShooterRPMTolerance", (value) -> {
            kShooterPIDControllerTop.setTolerance(value);
            kShooterPIDControllerBottom.setTolerance(value);
        }, EnumSet.of(SmartRunner.RunLevel.SHOOTER_TUNING), ShooterConstants.rpmTolerance);

        updateSmartdashboard();

        if (pidEnabled) {
            calculatePID();
        }
    }

    public void setRPM(double rpmBottom, double rpmTop) {
        kShooterPIDControllerTop.setSetpoint(rpmTop);
        kShooterPIDControllerBottom.setSetpoint(rpmBottom);
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
        double errBottom = kShooterPIDControllerBottom.calculate(bottomEncoder.getVelocity());
        double errTop = kShooterPIDControllerTop.calculate(topEncoder.getVelocity()); // Top runs backwards,
                                                                                                // so target is negative

        double ffBottom = ShooterConstants.kFF * kShooterPIDControllerBottom.getSetpoint();
        double ffTop = ShooterConstants.kFF * kShooterPIDControllerTop.getSetpoint(); // Top runs backwards

        bottomMotor.set(errBottom + ffBottom);
        topMotor.set(errTop + ffTop);
    }

    public void updateSmartdashboard() {
        SmartRunner.Logger.put("BottomRPM", bottomEncoder.getVelocity(), EnumSet.of(SmartRunner.RunLevel.DEBUG, SmartRunner.RunLevel.SHOOTER_TUNING));
        SmartRunner.Logger.put("TopRPM", topEncoder.getVelocity(), EnumSet.of(SmartRunner.RunLevel.DEBUG, SmartRunner.RunLevel.SHOOTER_TUNING));

        SmartRunner.Logger.put("TargetRPMBottom", kShooterPIDControllerBottom.getSetpoint(), EnumSet.of(SmartRunner.RunLevel.SHOOTER_TUNING));
        SmartRunner.Logger.put("TargetRPMTop", kShooterPIDControllerTop.getSetpoint(), EnumSet.of(SmartRunner.RunLevel.SHOOTER_TUNING));

        SmartRunner.Logger.put("ShooterPIDEnabled", pidEnabled, EnumSet.of(SmartRunner.RunLevel.SHOOTER_TUNING, SmartRunner.RunLevel.DEBUG));

        SmartRunner.Logger.put("Shooter Within Tolerance", atSetpoint(), EnumSet.of(SmartRunner.RunLevel.SHOOTER_TUNING, SmartRunner.RunLevel.DEBUG));
    }

    public boolean atSetpoint(){
        return !pidEnabled || (kShooterPIDControllerTop.atSetpoint() && kShooterPIDControllerBottom.atSetpoint());
    }

    public void startShooter(){
        pidEnabled = true;
    }

    public void stopShooter(){
        pidEnabled = false;
        setManual(0, 0);
    }
}
