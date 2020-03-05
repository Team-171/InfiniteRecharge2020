/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.shooter.ShintakeSubsystem;
// import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class AimAndShootByLimelight extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_driveSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private final ShintakeSubsystem m_shintakeSubsystem;

  private NetworkTable limeLightTable;

  /**
   * Creates a new ExampleCommand.
   *
   * @param driveSubsystem The subsystem used by this command.
   */
  public AimAndShootByLimelight(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem, ShintakeSubsystem shintakeSubsystem) {
    m_driveSubsystem = driveSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_shintakeSubsystem = shintakeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem, shooterSubsystem, shintakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limeLightTable = NetworkTableInstance.getDefault().getTable("limelight");
    limeLightTable.getEntry("ledMode").setNumber(3);
    m_shooterSubsystem.startShooter();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if (limeLightTable.getEntry("tv").getDouble(0) != 0) {
          m_driveSubsystem.startAutoAim();
          m_driveSubsystem.setTargetAngle(limeLightTable.getEntry("tx").getDouble(0));
      } else {
          m_driveSubsystem.arcadeDrive(0, 0);
      }

      if(m_driveSubsystem.isOnTarget() && m_shooterSubsystem.atSetpoint()){
          m_shintakeSubsystem.drive(1);
      }
      else{
          m_shintakeSubsystem.drive(0);
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.stopAutoAim();
    m_shooterSubsystem.stopShooter();
    m_shintakeSubsystem.drive(0);
    limeLightTable.getEntry("ledMode").setNumber(1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
