package org.jmhsrobotics.frc2026.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

public class GameState {
  public static final int TELEOP_SEC = 140;
  public static final int ENDGAME_SEC = 30;
  public static final int AUTO_SEC = 20;
  public static final int TRANS_LENGTH_SEC = 10;
  public static final int Q1_END_SEC = 35;
  public static final int Q2_END_SEC = 60;
  public static final int Q3_END_SEC = 85;
  public static final int Q4_END_SEC = 110;

  public enum Hub {
    ACTIVE,
    INACTIVE,
    UNKNOWN
  }

  public static Hub getHubStatus() {
    double time = DriverStation.getMatchTime();
    boolean isAuto = DriverStation.isAutonomous();

    if (isAuto) { // auto
      return Hub.ACTIVE;
    }
    if (time <= ENDGAME_SEC) { // endgame
      return Hub.ACTIVE;
    }
    if (time >= TELEOP_SEC - TRANS_LENGTH_SEC) { // transition
      return Hub.ACTIVE;
    }

    // determine which quarter we are in
    int quarter = (int) ((time - ENDGAME_SEC) / 25.0);
    Alliance alliance = getAlliance(DriverStation.getAlliance());
    boolean wonAuto = getWonAuto(DriverStation.getGameSpecificMessage(), alliance);

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
      // FIXME: add some sort of alert/behavior change if we can not get our Alliance Color
      return Alliance.Red;
    }
    return ally.get();
  }

  public static double getRemainingTimeInPeriod() {
    double time = DriverStation.getMatchTime();

    if (time < TRANS_LENGTH_SEC) {
      return TRANS_LENGTH_SEC - time;
    } else if (time < Q1_END_SEC) {
      return Q1_END_SEC - time;
    } else if (time < Q2_END_SEC) {
      return Q2_END_SEC - time;
    } else if (time < Q3_END_SEC) {
      return Q3_END_SEC - time;
    } else if (time < Q4_END_SEC) {
      return Q4_END_SEC - time;
    } else {
      return 0;
    }
  }

  private static boolean getWonAuto(String gameData, Alliance ally) {
    if (gameData.length() < 1 || gameData == null) {
      return false;
    }
    if ((gameData.charAt(0) == 'R' && (ally == Alliance.Red))
        || (gameData.charAt(0) == 'B' && (ally == Alliance.Blue))) {
      // FIXME: add some sort of alert/behavior change if we can not able to get Game data.
      return true; // we won auto
    } else {
      return false; // we didn't win auto
    }
  }
}
