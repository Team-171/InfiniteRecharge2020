package frc.robot.subsystems;

import java.util.EnumSet;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableInstance;
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
import frc.robot.SmartRunner;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;


public class DriveSubsystem extends SubsystemBase {

    public enum PIDMode {
        TURN,
        DIST,
        DIST_WITH_TURN,
        NONE
    }

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

    private final PIDController turnController = new PIDController(DriveConstants.turnP, DriveConstants.turnI, DriveConstants.turnD);

    private final PIDController distController = new PIDController(DriveConstants.distP, DriveConstants.distI, DriveConstants.distD);

    private PIDMode pidMode = PIDMode.NONE;

    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry m_odometry;

    public DriveSubsystem() {
        leftMotor1.restoreFactoryDefaults();
        leftMotor2.restoreFactoryDefaults();
        rightMotor1.restoreFactoryDefaults();
        rightMotor2.restoreFactoryDefaults();

        leftMotor1.setOpenLoopRampRate(DriveConstants.kRampRate);
        leftMotor2.setOpenLoopRampRate(DriveConstants.kRampRate);
        rightMotor1.setOpenLoopRampRate(DriveConstants.kRampRate);
        rightMotor2.setOpenLoopRampRate(DriveConstants.kRampRate);

        // Sets the distance per pulse for the encoders
        m_leftEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerRotation);
        m_rightEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerRotation);

        turnController.setIntegratorRange(-0.2, 0.2);
        turnController.setTolerance(DriveConstants.turnTolerance);

        distController.setIntegratorRange(-0.2, 0.2);
        distController.setTolerance(DriveConstants.distTolerance);

        // m_leftEncoder.setInverted(DriveConstants.kLeftEncoderReversed);
        // m_rightEncoder.setInverted(DriveConstants.kRightEncoderReversed);

        // m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerRotation);
        // m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerRotation);

        // SmartDashboard.putNumber("DrivetrainP", DriveConstants.P);
        // SmartDashboard.putNumber("DrivetrainI", DriveConstants.I);
        // SmartDashboard.putNumber("DrivetrainD", DriveConstants.D);

        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

        // Make sure limelight light is off
        
    }

    @Override
    public void periodic() {

        if (pidMode != PIDMode.NONE) {
            calculatePID();
        }

        m_odometry.update(Rotation2d.fromDegrees(getHeading()), m_leftEncoder.getPosition(),
                      m_rightEncoder.getPosition());

        updateSmartDashboard();
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

        // SmartDashboard.putNumber("Right Multiplier", rightMultiplier);
        arcadeDrive(forward, rotation * rightMultiplier);
	}

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        m_leftEncoder.setPosition(0);
        m_rightEncoder.setPosition(0);
    }

    /**
     * Gets the average distance of the TWO encoders.
     *
     * @return the average of the TWO encoder readings
     */
    public double getAverageEncoderDistance() {
      return (m_leftEncoder.getPosition() + -m_rightEncoder.getPosition()) / 2.0;
    }

    /**
     * Gets the left drive encoder.
     *
     * @return the left drive encoder
     */
    // public Encoder getLeftEncoder() {
    //   return m_leftEncoder;
    // }

    /**
     * Gets the right drive encoder.
     *
     * @return the right drive encoder
     */
    // public Encoder getRightEncoder() {
    //   return m_rightEncoder;
    // }

    /**
     * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    // public void setMaxOutput(double maxOutput) {
    //     m_drive.setMaxOutput(maxOutput);
    // }

    // public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        // return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
    // }

    public void setPIDMode(PIDMode mode){
        pidMode = mode;
        
        if(mode == PIDMode.NONE)
        {
            turnController.reset();
            distController.reset();
        }
    }

    public boolean isOnTargetAngle(){
        return turnController.atSetpoint();
    }

    public boolean isOnDistanceTarget(){
        return distController.atSetpoint();
    }

    public boolean isOnTarget(){
        return isOnDistanceTarget() && isOnTargetAngle();
    }

    public double getHeading() {
        return navx.getAngle();
    }

    public void calculatePID() {
        double turnOutput = turnController.calculate(getHeading());
        double distOutput = distController.calculate(getAverageEncoderDistance());

        SmartRunner.Logger.put("Drivetrain Turn PID Output", turnOutput, EnumSet.of(SmartRunner.RunLevel.DRIVETRAIN_TUNING));
        SmartRunner.Logger.put("Drivetrain Dist PID Output", distOutput, EnumSet.of(SmartRunner.RunLevel.DRIVETRAIN_TUNING));

        switch(pidMode)
        {
            case TURN:
                m_drive.arcadeDrive(0, turnOutput);
                break;

            case DIST:
                m_drive.arcadeDrive(distOutput, 0);
                break;

            case DIST_WITH_TURN:
                m_drive.arcadeDrive(distOutput, turnOutput);
                break;
        }

        // SmartDashboard.putNumber("Current Heading", currentHeading);
        // SmartDashboard.putNumber("Target Heading", targetHeading);
        // SmartDashboard.putNumber("Turn Value", err);
    }

    public void setRelativeTargetAngle(double angle) {
        turnController.setSetpoint(angle + getHeading());
    }

    public void setTargetDistance(double distance){
        distController.setSetpoint(distance);
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
      }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        m_leftMotors.setVoltage(leftVolts);
        m_rightMotors.setVoltage(-rightVolts);
    }

    public void updateSmartDashboard(){
        SmartRunner.Logger.put("Drivetrain Position", getAverageEncoderDistance(), EnumSet.of(SmartRunner.RunLevel.DRIVETRAIN_TUNING));

        SmartRunner.Logger.getNumber("DrivetrainTurnP", (value) -> {
            turnController.setP(value);
        }, EnumSet.of(SmartRunner.RunLevel.DRIVETRAIN_TUNING), DriveConstants.turnP);

        SmartRunner.Logger.getNumber("DrivetrainTurnI", (value) -> {
            turnController.setI(value);
        }, EnumSet.of(SmartRunner.RunLevel.DRIVETRAIN_TUNING), DriveConstants.turnI);

        SmartRunner.Logger.getNumber("DrivetrainTurnD", (value) -> {
            turnController.setD(value);
        }, EnumSet.of(SmartRunner.RunLevel.DRIVETRAIN_TUNING), DriveConstants.turnD);

        SmartRunner.Logger.getNumber("Draintrain Turn Tolerance", (value) -> {
            turnController.setTolerance(value);
        }, EnumSet.of(SmartRunner.RunLevel.DRIVETRAIN_TUNING), DriveConstants.turnTolerance);

        SmartRunner.Logger.put("AutoAim Error", turnController.getPositionError(), EnumSet.of(SmartRunner.RunLevel.DRIVETRAIN_TUNING));


        SmartRunner.Logger.getNumber("DrivetrainDistP", (value) -> {
            distController.setP(value);
        }, EnumSet.of(SmartRunner.RunLevel.DRIVETRAIN_TUNING), DriveConstants.distP);

        SmartRunner.Logger.getNumber("DrivetrainDistI", (value) -> {
            distController.setI(value);
        }, EnumSet.of(SmartRunner.RunLevel.DRIVETRAIN_TUNING), DriveConstants.distI);

        SmartRunner.Logger.getNumber("DrivetrainDistD", (value) -> {
            distController.setD(value);
        }, EnumSet.of(SmartRunner.RunLevel.DRIVETRAIN_TUNING), DriveConstants.distD);

        SmartRunner.Logger.getNumber("Draintrain Dist Tolerance", (value) -> {
            distController.setTolerance(value);
        }, EnumSet.of(SmartRunner.RunLevel.DRIVETRAIN_TUNING), DriveConstants.distTolerance);

        SmartRunner.Logger.put("Drivetrain Dist Error", distController.getPositionError(), EnumSet.of(SmartRunner.RunLevel.DRIVETRAIN_TUNING));
        
    }
}
