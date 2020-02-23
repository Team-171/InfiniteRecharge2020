package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;


public class DriveSubsystem extends SubsystemBase {

    private final CANSparkMax leftMotor1 = new CANSparkMax(DriveConstants.kLeftMotor1Port, MotorType.kBrushless);
    private final CANSparkMax leftMotor2 = new CANSparkMax(DriveConstants.kLeftMotor2Port, MotorType.kBrushless);
    private final CANSparkMax rightMotor1 = new CANSparkMax(DriveConstants.kRightMotor1Port, MotorType.kBrushless);
    private final CANSparkMax rightMotor2 = new CANSparkMax(DriveConstants.kRightMotor2Port, MotorType.kBrushless);

    // private final Talon leftMotor1 = new Talon(DriveConstants.kLeftMotor1Port);
    // private final Talon leftMotor2 = new Talon(DriveConstants.kLeftMotor2Port);
    // private final Talon rightMotor1 = new Talon(DriveConstants.kRightMotor1Port);
    // private final Talon rightMotor2 = new Talon(DriveConstants.kRightMotor2Port);

    private final SpeedControllerGroup m_leftMotors = 
        new SpeedControllerGroup(leftMotor1, leftMotor2);

    private final SpeedControllerGroup m_rightMotors = 
        new SpeedControllerGroup(rightMotor1, rightMotor2);

    // The robot's drive
    private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

    // The drive encoders
    private final CANEncoder m_leftEncoder = new CANEncoder(leftMotor1);
    private final CANEncoder m_rightEncoder = new CANEncoder(rightMotor1);

    // private final Encoder m_leftEncoder = new Encoder(DriveConstants.kLeftEncoderPorts[0], DriveConstants.kLeftEncoderPorts[1]);
    // private final Encoder m_rightEncoder = new Encoder(DriveConstants.kRightEncoderPorts[0], DriveConstants.kRightEncoderPorts[1]);

    private final AHRS navx = new AHRS(SerialPort.Port.kUSB);

    private final PIDController turnController = new PIDController(
        SmartDashboard.getNumber("DrivetrainP", DriveConstants.P), 
        SmartDashboard.getNumber("DrivetrainI", DriveConstants.I), 
        SmartDashboard.getNumber("DrivetrainD", DriveConstants.D)
    );

    public boolean autoAim = false;
    private double limelightAngle = 0;

    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry m_odometry;

    public DriveSubsystem() {
        leftMotor1.restoreFactoryDefaults();
        leftMotor2.restoreFactoryDefaults();
        rightMotor1.restoreFactoryDefaults();
        rightMotor2.restoreFactoryDefaults();

        leftMotor1.setOpenLoopRampRate(0.25);
        leftMotor2.setOpenLoopRampRate(0.25);
        rightMotor1.setOpenLoopRampRate(0.25);
        rightMotor2.setOpenLoopRampRate(0.25);

        // Sets the distance per pulse for the encoders
        m_leftEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerRotation);
        m_rightEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerRotation);

        // m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerRotation);
        // m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerRotation);

        SmartDashboard.putNumber("DrivetrainP", DriveConstants.P);
        SmartDashboard.putNumber("DrivetrainI", DriveConstants.I);
        SmartDashboard.putNumber("DrivetrainD", DriveConstants.D);

        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        // m_odometry.update(Rotation2d.fromDegrees(getHeading()), m_leftEncoder.getPosition(),
        //                   m_rightEncoder.getPosition());

        if (autoAim) {
            turnController.setPID(
                SmartDashboard.getNumber("DrivetrainP", DriveConstants.P), 
                SmartDashboard.getNumber("DrivetrainI", DriveConstants.I), 
                SmartDashboard.getNumber("DrivetrainD", DriveConstants.D));
            calculatePID();
        }

        m_odometry.update(Rotation2d.fromDegrees(getHeading()), m_leftEncoder.getPosition(),
                      m_rightEncoder.getPosition());
    }

        /**
     * Drives the robot using arcade controls.
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    public void arcadeDrive(double fwd, double rot) {
        m_drive.arcadeDrive(fwd * DriveConstants.kSpeedMultiplier, rot * DriveConstants.kSpeedMultiplier);
    }

    public void driveRobot(double forward, double rotation) {
        double rightMultiplier = OIConstants.kminTurnMultiplier 
            + ((1 - Math.abs(forward)) 
            * (OIConstants.kmaxTurnMultiplier - OIConstants.kminTurnMultiplier));

        SmartDashboard.putNumber("Right Multiplier", rightMultiplier);
        arcadeDrive(forward, rotation * rightMultiplier);
	}

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        // m_leftEncoder.reset();
        // m_rightEncoder.reset();
    }

    /**
     * Gets the average distance of the TWO encoders.
     *
     * @return the average of the TWO encoder readings
     */
    // public double getAverageEncoderDistance() {
    //   // return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
    // }

    /**
     * Gets the left drive encoder.
     *
     * @return the left drive encoder
     */
    // public Encoder getLeftEncoder() {
    //   // return m_leftEncoder;
    // }

    /**
     * Gets the right drive encoder.
     *
     * @return the right drive encoder
     */
    // public Encoder getRightEncoder() {
    //   // return m_rightEncoder;
    // }

    /**
     * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
        m_drive.setMaxOutput(maxOutput);
    }

    // public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        // return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
    // }

    public double getHeading() {
        return navx.getAngle();
    }

    public void calculatePID() {
        double currentHeading = getHeading();
        double targetHeading = currentHeading + limelightAngle;
        double err = turnController.calculate(currentHeading, targetHeading);

        m_drive.arcadeDrive(0, err);
        SmartDashboard.putNumber("Current Heading", currentHeading);
        SmartDashboard.putNumber("Target Heading", targetHeading);
        SmartDashboard.putNumber("Turn Value", err);
    }

    public void setTargetAngle(double angle) {
        limelightAngle = angle;
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
      }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        m_leftMotors.setVoltage(leftVolts);
        m_rightMotors.setVoltage(-rightVolts);
    }
}
