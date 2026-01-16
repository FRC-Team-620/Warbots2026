
package frc.robot;
import edu.wpi.first.wpilibj.DriverStation;

public class GameState {
    public enum hubStatus {
        ACTIVE, INACTIVE, UNKNOWN
    }

    // private AllianceColor currentAlliance = AllianceColor.UNKNOWN;

    Optional<Alliance> ally = DriverStation.getAlliance();
    //import ^^

    public void updateGameState() {
        String gameData;
        gameData = DriverStation.getGameSpecificMessage();
        if(gameData.length() > 0){
            if(gameData.charAt(0) == "R" &&  ally.get() == Alliance.Red){
                //finish logic
            }
            else if(gameData.charAt(0) == "B"){
                //finish logic
            }
            else{
                //else, if we dont know what hub is active
            }
        }
        else{
            //else if we dont receive any data
        }
    }
    //eventual return statement
}