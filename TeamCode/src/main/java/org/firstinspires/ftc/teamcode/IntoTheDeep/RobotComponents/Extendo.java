package org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.MathHelpers.GetSamplePosition.MMToEncoderTicks;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Lift.LIFTSTATES.CUSTOMMOTORPOWER;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.RobotInitializers.Dashtelemetry;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.RobotInitializers.ExpansionHubDigital;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.TeleopsStarter.gm1;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.IntoTheDeep.MathHelpers.PIDController;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Wrapers.CachedMotor;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Wrapers.LimitSwitch;

@Config
public class Extendo {

    public enum ExtendoStates {
        RETRACTING,
        FREEWILL,
        GOTOPOS,
        CUSTOMMOTORPOWER,
        BALANCEFROMPOINT
    }

    public static CachedMotor motor, encoder;
    public static PIDController pidController = new PIDController(0.0075, 0, 0.0002),
                                pidBalancing = new PIDController(-0.13,0,0);
    public static LimitSwitch lm;
    public static final int MaxExtension = 880;
    public static ExtendoStates state = ExtendoStates.RETRACTING;
    public static boolean DoingAuto = false;
    private static boolean wasHoldPosOnce = false;
    private static double PowerToMotors = 0;
    public static double TimeoutToStabilize = 0.3;
    public static SparkFunOTOS.Pose2D chosenpos = new SparkFunOTOS.Pose2D(0,0,0);
    public static boolean chosenPoint = false;
    private static ElapsedTime time = new ElapsedTime();

    public static void setExtendoPos(double Pos) {
        Pos = Math.max(0, Math.min(Pos, MaxExtension));
        pidController.setTargetPosition(Pos);
    }

    public static double getPosition() {
        return encoder.getCurrentPosition();
    }

    public static boolean IsExtendoDone(double Error) {
        return Math.abs(pidController.getTargetPosition() - getPosition()) <= Error;
    }

    public static void CustomPowerToMotors(double power) {
        PowerToMotors = power;
        state = Extendo.ExtendoStates.CUSTOMMOTORPOWER;
    }

    public static void update() {
        Dashtelemetry.addData("extendo state", state);
        Dashtelemetry.addData("extendo ticks", getPosition());
        switch (state) {
            case FREEWILL:
                if (-gm1.right_stick_y >= 0.05 && getPosition() <= MaxExtension - 30) {
                    time.reset();
                    motor.setPower(-Math.signum(gm1.right_stick_y) * (gm1.right_stick_y * gm1.right_stick_y));
                    wasHoldPosOnce = false;
                } else if (-gm1.right_stick_y <= -0.05 && getPosition() >= 30) {
                    time.reset();
                    motor.setPower(-Math.signum(gm1.right_stick_y) * (gm1.right_stick_y * gm1.right_stick_y));
                    wasHoldPosOnce = false;
                }
                else{
                    if (time.seconds() > TimeoutToStabilize)
                    {
                        if(!wasHoldPosOnce){
                            wasHoldPosOnce = true;
                            setExtendoPos(getPosition());
                        }
                        motor.setPower(pidController.calculatePower(getPosition()));
                    }
                    else {
                        motor.setPower(0);
                    }
                }

                break;

            case RETRACTING:

                motor.setPower(-1);
                boolean ShouldReset;
                try {
                    ShouldReset = lm.getState();
                } catch (Exception e) {
                    ShouldReset = motor.getCurrent(CurrentUnit.AMPS) >= 4 && Math.abs(encoder.getVelocity()) <= 0 && getPosition() < 40;
                    Dashtelemetry.addData("lift limit switch not operational", "");
                }
                if (ShouldReset) {
                    motor.setPower(0);
                    encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    setExtendoPos(0);
                    if (!DoingAuto)
                        state = ExtendoStates.FREEWILL;
                    else {
                        setExtendoPos(0);
                        state = ExtendoStates.GOTOPOS;
                    }
                }
                break;
            case GOTOPOS:
                chosenPoint = false;
                motor.setPower(pidController.calculatePower(getPosition()));
                break;
            case CUSTOMMOTORPOWER:
                motor.setPower(PowerToMotors);
                break;
            case BALANCEFROMPOINT:
                if(!chosenPoint) {
                    chosenpos = Localizer.getCurrentPosition();
                    chosenPoint = true;
                    setExtendoPos(0);
                }
                if(chosenPoint){
                    double distanceInMM = Localizer.getDistanceFromTwoPoints(chosenpos,Localizer.getCurrentPosition());
                    setExtendoPos(MMToEncoderTicks(distanceInMM));
                }
                if(getPosition() < 75)
                    Intake.DropUp();
                else
                    Intake.DropDown();
                motor.setPower(pidController.calculatePower(getPosition()));
                break;
        }
    }

}
