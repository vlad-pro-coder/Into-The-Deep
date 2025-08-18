package org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOps.Tests;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.ApproachingSubmersibleActions.AfterThirdSampleGoToSubmersible.AfterThirdSampleGoToSubmersibleActions;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.ApproachingSubmersibleActions.FailedToDetect.FailedToDetectActions;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.ApproachingSubmersibleActions.TakeCachedSample.TakeCachedSampleActions;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.besttxty;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.CameraPipelines.YellowSampleDetectionPipeline.diminuator;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.CameraPipelines.YellowSampleDetectionPipeline.startingY;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.MathHelpers.GetSamplePosition.centerToExtendo;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.MathHelpers.GetSamplePosition.getExtendoRotPair;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.MathHelpers.GetSamplePosition.getPositionRelativeToRobot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Scheduler;
import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Task;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.Samples;
import org.firstinspires.ftc.teamcode.IntoTheDeep.CameraPipelines.YellowSampleDetectionPipeline;
import org.firstinspires.ftc.teamcode.IntoTheDeep.MathHelpers.GetSamplePosition;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Chassis;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Extendo;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Intake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Lift;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Localizer;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Outtake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.RobotInitializers;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Wrapers.Logitech920;

import java.util.List;

@TeleOp
@Config
public class CameraTest extends LinearOpMode {

    Logitech920 camera;
    public enum RobotStates{
        CRUISETOSUBMERSIBLE,
        OCCUPIEDPRELOADS,
        DETECT,
        FAILEDDETECTION,
        GRABFROMSUBMERSIBLE,
        SUBMERSIBLESAMPLECYCLE,
        NEXTONE,
    }

    public static boolean gofaceSample = false;
    public static boolean laststate = false;

    public static double k = 1;
    public RobotStates robotstate = CameraTest.RobotStates.CRUISETOSUBMERSIBLE;
    public boolean WereInstructionGiven = false;
    @Override
    public void runOpMode() throws InterruptedException {
        RobotInitializers.InitializeFull(hardwareMap);
        RobotInitializers.InitializeForOperation();
        Outtake.armProfile.setInstant(351);
        Intake.DropUp();
        Extendo.DoingAuto = true;
        Scheduler tasks = new Scheduler();


        camera  = new Logitech920("logitik",hardwareMap);
        camera.startCamera();
        Chassis.usedTrajectory = Chassis.trajectoryStates.FREEWILL;

        Chassis.setTargetPosition(new SparkFunOTOS.Pose2D(0,0,0));

        while(opModeInInit()){

            RobotInitializers.clearCache();

            YellowSampleDetectionPipeline.SamplePoint sample = camera.yellow.getBestTxTy();
            SparkFunOTOS.Pose2D offsets = getPositionRelativeToRobot(sample.x, sample.y);
            SparkFunOTOS.Pose2D data = getExtendoRotPair(sample.x, sample.y);

            RobotInitializers.Dashtelemetry.addData("extendo distance pachy style",(Math.sqrt(data.x * data.x + data.y * data.y)) * k);

            RobotInitializers.Dashtelemetry.addData("x distance",offsets.x);
            RobotInitializers.Dashtelemetry.addData("y distance",offsets.y);

            RobotInitializers.Dashtelemetry.addData("extendo distance",data.x);
            RobotInitializers.Dashtelemetry.addData("h heading",data.h);

            RobotInitializers.Dashtelemetry.addData("function of accepted distance",startingY + Math.toDegrees(Math.abs(Localizer.getAngleDifference(Math.toRadians(0),data.h))) * diminuator);
            RobotInitializers.Dashtelemetry.addData("extendo data", Extendo.getPosition());


            //Chassis.update();

            //Extendo.update();
            //tasks.update();
            Localizer.Update();
        }
        waitForStart();
        while(opModeIsActive()){
            RobotInitializers.clearCache();

            switch (robotstate){
                case CRUISETOSUBMERSIBLE:
                    if(tasks.IsSchedulerDone())
                    {
                        robotstate = CameraTest.RobotStates.DETECT;
                        WereInstructionGiven = false;
                    }
                    break;
                case DETECT:
                    if(!WereInstructionGiven){
                        tasks.
                                addTask(new Task() {
                                    @Override
                                    protected void Actions() {
                                        Intake.StopSpinner();
                                        Intake.Unblock();
                                    }

                                    @Override
                                    protected boolean Conditions() {
                                        return true;
                                    }
                                });
                        WereInstructionGiven = true;
                        camera.yellow.newResultsReady = false;
                    }
                    if(tasks.IsSchedulerDone() && camera.yellow.newResultsReady)//and no blur
                    {
                        besttxty = camera.yellow.getBestTxTy();
                        robotstate = CameraTest.RobotStates.GRABFROMSUBMERSIBLE;
                        WereInstructionGiven = false;
                    }
                    break;

                case GRABFROMSUBMERSIBLE:
                    if(!WereInstructionGiven){
                        tasks.AddAnotherScheduler(TakeCachedSampleActions(besttxty,0.7));
                        WereInstructionGiven = true;
                    }
                    if(Intake.HasWrongTeamPiece() && !Intake.isStorageEmpty())
                    {
                        tasks.clear();
                        robotstate = CameraTest.RobotStates.FAILEDDETECTION;
                        WereInstructionGiven = false;
                    }
                    if(tasks.IsSchedulerDone()){
                        RobotLog.ii("color detected", "" + Intake.getStorageStatus());
                        RobotLog.ii("reached trapdoor", "" + Intake.SampleReachedTrapDoor());

                        if(Intake.HasMixedTeamPiece()) {
                            robotstate = CameraTest.RobotStates.NEXTONE;
                            RobotLog.ii("accepted","");
                        }
                        else {
                            robotstate = CameraTest.RobotStates.FAILEDDETECTION;
                            RobotLog.ii("rejected","");
                        }
                        WereInstructionGiven = false;
                    }
                    break;
                case FAILEDDETECTION:
                    if(!WereInstructionGiven){
                        tasks.AddAnotherScheduler(FailedToDetectActions());
                        WereInstructionGiven = true;
                    }
                    if(tasks.IsSchedulerDone()){
                        robotstate = CameraTest.RobotStates.DETECT;
                        WereInstructionGiven = false;
                    }
                    break;
                case NEXTONE:
                    if(!WereInstructionGiven){
                        tasks.addTask(new Task() {
                                    @Override
                                    protected void Actions() {
                                        Extendo.state = Extendo.ExtendoStates.GOTOPOS;
                                        Extendo.setExtendoPos(100);
                                        Intake.DropUp();
                                    }

                                    @Override
                                    protected boolean Conditions() {
                                        return Extendo.IsExtendoDone(30);
                                    }
                                })
                                .addTask(new Task() {
                            @Override
                            protected void Actions() {
                                Chassis.setHeading(Math.toRadians(90));
                                Intake.Unblock();
                            }

                            @Override
                            protected boolean Conditions() {
                                return Chassis.IsHeadingDone(3);
                            }
                        })
                                .addTask(new Task() {
                                    @Override
                                    protected void Actions() {
                                        Intake.RotateToEject();
                                    }

                                    @Override
                                    protected boolean Conditions() {
                                        return true;
                                    }
                                })
                                .waitSeconds(0.1)
                                .addTask(new Task() {
                                    @Override
                                    protected void Actions() {
                                        Intake.StopSpinner();
                                    }

                                    @Override
                                    protected boolean Conditions() {
                                        return true;
                                    }
                                });
                        WereInstructionGiven = true;
                    }
                    if(tasks.IsSchedulerDone()){
                        robotstate = RobotStates.FAILEDDETECTION;
                        WereInstructionGiven = false;
                    }
                    break;
            }




            RobotInitializers.Dashtelemetry.addData("color", Intake.getStorageStatus());
            RobotLog.ii("from loop color", "" + Intake.getStorageStatus());
            RobotInitializers.Dashtelemetry.addData("has mixed team piece",Intake.HasMixedTeamPiece());
            RobotInitializers.Dashtelemetry.addData("tasks length",tasks.tasks.size());
            RobotInitializers.Dashtelemetry.addData("state",robotstate);
            RobotInitializers.Dashtelemetry.addData("best tx",besttxty.x);
            RobotInitializers.Dashtelemetry.addData("best ty",besttxty.y);
            RobotInitializers.Dashtelemetry.addData("r",Intake.colorsensor.RGB.R);
            RobotInitializers.Dashtelemetry.addData("g",Intake.colorsensor.RGB.G);
            RobotInitializers.Dashtelemetry.addData("b",Intake.colorsensor.RGB.B);
            RobotInitializers.Dashtelemetry.addData("d",Intake.colorsensor.RGB.D);

            Chassis.update();
            Extendo.update();
            tasks.update();
            Localizer.Update();
        }
    }
}
