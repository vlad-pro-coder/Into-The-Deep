package org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.TeleopsStarter.team;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.IntoTheDeep.TEAM;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Wrapers.CachedMotor;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Wrapers.RGBsensor;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Wrapers.RangeSensor;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Wrapers.ServoPlus;
@Config
public class Intake {

    public enum SampleType {
        BLUE,
        RED,
        YELLOW,
        NONE
    }

    public static ServoPlus blocker,dropdownServo;
    public static CachedMotor spinner;
    public static RGBsensor colorsensor;
    public static RangeSensor rangesensor;
    public static double dropdownAngle = 185,dropupAngle = 290,startDropDownUpCorrection = 230;
    public static double NotBlockingPos = 27,BlockingPos = 170;


    public static void Block(){
        blocker.setAngle(BlockingPos);
    }

    public static void Unblock(){
        blocker.setAngle(NotBlockingPos);
    }
    public static void DropDown(){
        double pos = dropdownAngle + (startDropDownUpCorrection - dropdownAngle) / Extendo.MaxExtension * Extendo.getPosition();
        dropdownServo.setAngle(pos);
    }

    public static void DropUp(){
        dropdownServo.setAngle(dropupAngle);
    }

    public static void RotateToStore(){
        spinner.setPower(1);
    }
    public static void RotateToStore(double power){
        spinner.setPower(Math.abs(power));
    }
    public static void RotateToEject(){
        spinner.setPower(-1);
    }
    public static void RotateToEject(double power){
        spinner.setPower(-Math.abs(power));
    }
    public static void StopSpinner(){
        spinner.setPower(0);
    }

    public static SampleType getStorageStatus(){
        if(colorsensor.getDistance(DistanceUnit.CM) >= 4.3) return SampleType.NONE;
        switch (colorsensor.getColorSeenBySensor()){
            case RED:
                return SampleType.RED;
            case BLUE:
                return SampleType.BLUE;
            case YELLOW:
                return SampleType.YELLOW;
            default:
                return SampleType.NONE;
        }
    }
    public static boolean SampleReachedTrapDoor(){
        return rangesensor.getDist() <= 9;//in cm
    }
    public static boolean isStorageEmpty(){
        return getStorageStatus() == SampleType.NONE;
    }

    public static boolean HasExactTeamPiece(){
        if(team == TEAM.RED && getStorageStatus() == SampleType.RED)
            return true;
        else return team == TEAM.BLUE && getStorageStatus() == SampleType.BLUE;
    }
    public static boolean HasMixedTeamPiece(){
        return getStorageStatus() == SampleType.YELLOW || HasExactTeamPiece();
    }
    public static boolean HasWrongTeamPiece() {
        return !HasExactTeamPiece();
    }


}
