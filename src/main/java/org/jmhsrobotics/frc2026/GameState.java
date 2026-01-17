package org.jmhsrobotics.frc2026;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;

public class GameState {
  public static final int TELEOP_SEC = 140;
  public static final int ENDGAME_SEC = 30;
  public static final int AUTO_SEC = 20;
  public static final int TRANS_LENGTH_SEC = 10;

  public enum Hub {
    ACTIVE,
    INACTIVE,
    UNKNOWN
  }

  public static Hub getHubStatus() {
    double time = DriverStation.getMatchTime();
    boolean isAuto = DriverStation.isAutonomous();
    Alliance alliance = getAlliance(DriverStation.getAlliance());
    boolean wonAuto = getWonAuto(DriverStation.getGameSpecificMessage(), alliance);

    // determine which quarter we are in
    int quarter = (int) ((time - ENDGAME_SEC) / 25.0);

    // SmartDashboard.putNumber("Actual Switch Time", TELEOP_SEC-TRANS_LENGTH_SEC-ENDGAME_SEC);
    SmartDashboard.putNumber("Switch Time", time - ENDGAME_SEC);
    SmartDashboard.putNumber("Quarter", quarter);
    SmartDashboard.putBoolean("TransitionReturnsActive", time >= TELEOP_SEC - TRANS_LENGTH_SEC);
    SmartDashboard.putBoolean("FirstShiftActivity", quarter % 2 == (wonAuto ? 1 : 0));
    SmartDashboard.putBoolean("WonAuto", wonAuto);
    SmartDashboard.putNumber("Time ", DriverStation.getMatchTime());
    SmartDashboard.putBoolean("In Auto ", DriverStation.isAutonomous());

    if (isAuto) { // auto
      return Hub.ACTIVE;
    }
    if (time <= ENDGAME_SEC) { // endgame
      return Hub.ACTIVE;
    }
    if (time >= TELEOP_SEC - TRANS_LENGTH_SEC) { // transition
      return Hub.ACTIVE;
    }
    if (quarter % 2
        == (wonAuto
            ? 1
            : 0)) { // first and third quarter and we won, or second and fourth quarter and we lost
      return Hub.INACTIVE;
    } else { // first and third quarter and we lost, or second and fourth quarter and we won
      return Hub.ACTIVE;
    }
  }

  private static Alliance getAlliance(Optional<Alliance> ally) {
    if (!ally.isPresent()) {
      return Alliance.Red;
    }
    return ally.get();
  }

  private static boolean getWonAuto(String gameData, Alliance ally) {
    if (gameData.length() < 1 || gameData == null) {
      return false;
    }
    if ((gameData.charAt(0) == 'R' && (ally == Alliance.Red))
        || (gameData.charAt(0) == 'B' && (ally == Alliance.Blue))) {
      return true; // we won auto
    } else {
      return false; // we didn't win auto
    }
  }
}
