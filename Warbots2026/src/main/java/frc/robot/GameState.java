
package frc.robot;
import edu.wpi.first.wpilibj.DriverStation;

public class GameState {

    public enum AllianceColor {
        RED, BLUE, UNKNOWN
    }
    public enum hubStatus {
        ACTIVE, INACTIVE, UNKNOWN
    }

    private AllianceColor currentAlliance = AllianceColor.UNKNOWN;

    public void updateGameState() {
        String gameData = DriverStation.getGameSpecificMessage();
        
        if (gameData != null && gameData.length() > 0) {
            switch (gameData.charAt(0)) {
                case 'B':
                    currentAlliance = AllianceColor.BLUE;
                    break;
                case 'R':
                    currentAlliance = AllianceColor.RED;
                    break;
                default:
                    currentAlliance = AllianceColor.UNKNOWN;
                    break;
            }



        } else {
            currentAlliance = AllianceColor.UNKNOWN;
        }
    }
    public AllianceColor getState(){
        return currentAlliance;
    }
}