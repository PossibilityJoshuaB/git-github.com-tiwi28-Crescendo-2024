package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants.DriveMode;
import frc.robot.subsystems.Drive.Swerve;

public class SnapTo extends Command {
  private final Swerve m_drive;
  private final DriveMode m_driveMode;

  public SnapTo(Swerve drive, DriveMode mode) {
    m_drive = drive;
    m_driveMode = mode;
  }

  @Override
  public void initialize() {
    m_drive.setDriveMode(m_driveMode);
  }

  // @Override
  // public void execute() {
  // m_drive.setDriveMode(m_driveMode);
  // }

  @Override
  public void end(boolean interrupted) {
    // TODO: blip 3
    if (!interrupted) {
      m_drive.setDriveMode(DriveMode.NORMAL);
    }
  }
}
