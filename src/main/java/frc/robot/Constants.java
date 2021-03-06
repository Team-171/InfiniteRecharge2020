/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.EnumSet;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final EnumSet<SmartRunner.RunLevel> robotLogLevel = EnumSet.of(SmartRunner.RunLevel.MATCH);

  public static final class DriveConstants {
    public static final int kLeftMotor1Port = 1;
    public static final int kLeftMotor2Port = 2;
    public static final int kRightMotor1Port = 3;
    public static final int kRightMotor2Port = 4;

    public static final double kSpeedMultiplier = 1.0;

    public static final double kRampRate = 0.25;

    public static final double turnP = 0.02;
    public static final double turnI = 0.03;
    public static final double turnD = 0.005;
    public static final double turnTolerance = 2;

    public static final double distP = 0.016;
    public static final double distI = 0.003;
    public static final double distD = 0.00;
    public static final double distTolerance = 4;

    public static final int[] kLeftEncoderPorts = new int[]{0, 1};
    public static final int[] kRightEncoderPorts = new int[]{2, 3};
    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = true;

    public static final int kEncoderCPR = 42;  // NEOs (42), but may use REV Robotics hex shaft encoders (8192 counts per revolution)
    public static final double kWheelDiameterInches = 6;
    public static final double kMotorToWheelRatio = 8.25;
    public static final double kEncoderDistancePerRotation = 
        (kMotorToWheelRatio *
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterInches * Math.PI)) 
        / kEncoderCPR;
  }

  public static final class ElevatorConstants {
    public static final int kElevatorLeftMotor = 5;
    public static final int kElevatorRightMotor = 6;

    // TODO: Get real position multiplier
    public static final double kPositionMultipier = 10;

    public static double forwardSpeedMultiplier = 1.0;
    public static double reverseSpeedMultiplier = 0.4;

    public static double kP = 0.1;
    public static double kI = 0.0;
    public static double kD = 0.0;

    // TODO: Get real climb height values
    public static double kHomePosition = -5.5;
    public static double kClimbPosition = 100.0; 
    public static double kCarryPosition = 90.0;

    //TODO: Get real min and max elevator heights
    public static double minHeight = -6.0;
    public static double maxHeight = 135.0;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kDriverYAxis = 1;
    public static final int kDriverTurnAxis = 2;
    public static final int kDriverSpeedAxis = 3;

    public static final double kmaxTurnMultiplier = .6;
    public static final double kminTurnMultiplier = .35;

    public static final int kOperatorControllerPort = 1;

    public static final int kOperatorLeftTrigger = 2;
    public static final int kOperatorRightTrigger = 3;
    public static final int kOperatorLeftJoystick = 1;
    public static final int kOperatorRightJoystick = 5;
    public static final int kOperatorLeftBumper = 5;
    public static final int kOperatorRightBumper = 6;
  }

  public static final class ShooterConstants {
    public static final int kTopMotorPort = 9;
    public static final int kBottomMotorPort = 10;

    public static double kFF = 1.0 / 5874.0;

    public static double kPTop = .00015;
    public static double kITop = 0.001;
    public static double kDTop = 0.00004;
    
    public static double kPBottom = .00025;
    public static double kIBottom = 0.001;
    public static double kDBottom = 0.00004;

    public static double rpmTolerance = 100;

    public static double rpmTop = 3000;
    public static double rpmBottom = 5000;
  }

  public static final class ShintakeConstants {
    public static final int kUpperMotorPort = 7;
    public static final int kLowerMotorPort = 8;

    public static final int kTopProxSwitch = 0;
    public static final int kBottomProxSwitch = 1;
  }

  public static final class IntakeConstants {
    public static int kMotorPort = 9;//TODO: get actual motor port
  }

}
