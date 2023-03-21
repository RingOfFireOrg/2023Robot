package frc.robot.commands;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.linearSlideArm;
import frc.robot.subsystems.outtakeTransfer;

public class AutoFactory {

    private final LoggedDashboardChooser<AutoMode> m_modeChooser;
    private final LoggedDashboardChooser<frc.robot.commands.AutoUtils.ScoringHeights> m_heightChooser;
    private final LoggedDashboardChooser<frc.robot.commands.AutoUtils.StartingZones> m_locationChooser;
    public enum AutoMode {
        MOBILITY,
        ENGAGE,
        SINGLE_SCORE_CONE,
        SINGE_SCORE_CUBE,
        DOUBLE_SCORE_CONE,
        DOUBLE_SCORE_CUBE,
        SINGLE_CONE_ENGAGE,
        SINGLE_CUBE_ENGAGE,
        DOUBLE_CONE_ENGAGE,
        DOUBLE_CUBE_ENGAGE,
        TRIPLE_CONE
    }

    private final SwerveSubsystem swerve;
    private final frc.robot.subsystems.pistonIntake pistonIntake;
    private final outtakeTransfer transfer;
    private final linearSlideArm arrow;

    public AutoFactory(linearSlideArm arrow, SwerveSubsystem drive, outtakeTransfer wheelie, frc.robot.subsystems.pistonIntake pistonIntake) {
        this.arrow = arrow;
        this.swerve = drive;
        this.transfer = wheelie;
        this.pistonIntake = pistonIntake;
        m_modeChooser = new LoggedDashboardChooser<>("Auto Mode");
        m_heightChooser = new LoggedDashboardChooser<>("Scoring Height");
        m_locationChooser = new LoggedDashboardChooser<>("Starting Location");

        // Initialize dashboard choosers
        m_locationChooser.addDefaultOption("LEFT", frc.robot.commands.AutoUtils.StartingZones.LEFT);
        for (frc.robot.commands.AutoUtils.StartingZones start : frc.robot.commands.AutoUtils.StartingZones.values()) {
            if (start != frc.robot.commands.AutoUtils.StartingZones.LEFT) {
                m_locationChooser.addOption(start.toString(), start);
            }
        }

        m_heightChooser.addDefaultOption("LOW", frc.robot.commands.AutoUtils.ScoringHeights.LOW);
        for (frc.robot.commands.AutoUtils.ScoringHeights height : frc.robot.commands.AutoUtils.ScoringHeights.values()) {
            if (height != frc.robot.commands.AutoUtils.ScoringHeights.LOW) {
                m_heightChooser.addOption(height.toString(), height);
            }
        }

        m_modeChooser.addDefaultOption("Mobility", AutoMode.MOBILITY);

        for (AutoMode mode : AutoMode.values()) {
            if (mode != AutoMode.MOBILITY) {
                m_modeChooser.addOption(mode.toString(), mode);
            }
        }
    }

    public Command getAutoRoutine() {
        frc.robot.commands.AutoUtils.StartingZones start = m_locationChooser.get();
        // frc.robot.commands.AutoUtils.ScoringHeights height = m_heightChooser.get();
        AutoMode mode = m_modeChooser.get();
        switch (mode) {
            case MOBILITY:
                return new MobilityCommandGroup(swerve, start);
            case ENGAGE:
                return new BalanceCommandGroup(swerve, start);
            case SINGLE_SCORE_CONE:
            case SINGE_SCORE_CUBE:
            case SINGLE_CONE_ENGAGE:
            case SINGLE_CUBE_ENGAGE:
            case DOUBLE_SCORE_CONE:
            case DOUBLE_SCORE_CUBE:

        }

        return new InstantCommand();
    }
}
