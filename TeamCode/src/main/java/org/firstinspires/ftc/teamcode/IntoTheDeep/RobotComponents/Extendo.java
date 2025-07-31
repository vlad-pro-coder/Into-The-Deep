package org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.TeleopsStarter.gm1;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.IntoTheDeep.MathHelpers.PIDController;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Wrapers.CachedMotor;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Wrapers.LimitSwitch;

@Config
public class Extendo {

    public enum ExtendoStates{
        RETRACTING,
        FREEWILL,
    }

    public static CachedMotor motor,encoder;
    public static PIDController pidController = new PIDController(0,0,0);
    public static LimitSwitch lm;
    public static final int MaxExtension = 880;
    public static ExtendoStates state = ExtendoStates.FREEWILL;
    private static double lastRegisteredPos = 0;

    public static void setExtendoPos(double Pos){
        Pos = Math.max(0,Math.min(Pos,MaxExtension));
        pidController.setTargetPosition(Pos);
    }
    public static double getPosition(){
        return encoder.getCurrentPosition();
    }

    public static void update(){

        switch (state) {
            case FREEWILL:
                if(-gm1.right_stick_y >= 0.05 && getPosition() <= MaxExtension && motor.getCurrent(CurrentUnit.AMPS) < 3){
                motor.setPower(-Math.signum(gm1.right_stick_y) * (gm1.right_stick_y * gm1.right_stick_y));
                    lastRegisteredPos = getPosition();
            }
                    else if(-gm1.right_stick_y <= -0.05 && getPosition() >= 10 && motor.getCurrent(CurrentUnit.AMPS) < 3)
                {
                    motor.setPower(-Math.signum(gm1.right_stick_y) * (gm1.right_stick_y * gm1.right_stick_y));
                    lastRegisteredPos = getPosition();
                }
                    else {
                        setExtendoPos(lastRegisteredPos);
                    motor.setPower(pidController.calculatePower(getPosition()));
                }
                    break;

            case RETRACTING:

                motor.setPower(-1);
                if(lm.getState() || motor.getCurrent(CurrentUnit.AMPS) >= 4)
                {
                    motor.setPower(0);
                    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    lastRegisteredPos = 0;
                    state = ExtendoStates.FREEWILL;
                }
                break;

        }
    }

}
