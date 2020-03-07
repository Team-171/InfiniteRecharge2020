/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.shooter.ShintakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class DriveForwardAndShoot extends SequentialCommandGroup {
  /**
   * Creates a new DriveForwardAndShoot.
   */
  public DriveForwardAndShoot(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem, ShintakeSubsystem shintakeSubsystem) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new DriveForward(driveSubsystem), new ShootBalls(shooterSubsystem, shintakeSubsystem));
  }
}
