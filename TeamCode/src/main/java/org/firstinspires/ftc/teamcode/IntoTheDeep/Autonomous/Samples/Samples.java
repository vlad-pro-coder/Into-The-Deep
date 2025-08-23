package org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.ApproachingSubmersibleActions.AfterThirdSampleGoToSubmersible.AfterThirdSampleGoToSubmersibleActions;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.ApproachingSubmersibleActions.FailedToDetect.FailedToDetectActions;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.ApproachingSubmersibleActions.TakeCachedSample.TakeCachedSampleActions;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.ApproachingSubmersibleActions.TakeSubmersibleSampleToHighBasket.GoingBackAndForthToHighBasketAndSubmersibleActions;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.CHASSIS_sample1pos;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.CHASSIS_sample2pos;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.CHASSIS_sample3pos;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.CHASSIS_turntosubmersibleforcontinuostaking;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.EXTENDO_sample1;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.EXTENDO_sample2;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.EXTENDO_sample3;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.EXTENSION_overbasket;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.EXTENSION_secondsample;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.LIFT_firstsample;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.LIFT_preload;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.LIFT_secondsample;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.LIFT_thirdsample;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.OVERHEAD_firstsample;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.OVERHEAD_idle;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.OVERHEAD_preload;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.OVERHEAD_secondsample;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.OVERHEAD_thirdsample;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.besttxty;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.FOURpreloadsLogic.PutLastSamplePredefined.PutLastSamplePredefinedActions;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.FOURpreloadsLogic.SimultaniosSampleGrabingScoring.FasterPreloadGrabingScoringActions;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.FOURpreloadsLogic.SimultaniosSampleGrabingScoring.SimultaniosSampleGrabingScoringActions;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.ParkingActions.Parking.ParkingActions;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.CameraPipelines.YellowSampleDetectionPipeline.diminuator;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.CameraPipelines.YellowSampleDetectionPipeline.startingY;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.MathHelpers.GetSamplePosition.getExtendoRotPair;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.MathHelpers.GetSamplePosition.getPositionRelativeToRobot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Scheduler;
import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Task;
import org.firstinspires.ftc.teamcode.IntoTheDeep.CameraPipelines.YellowSampleDetectionPipeline;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Chassis;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Extendo;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Intake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Lift;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Localizer;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Outtake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.RobotInitializers;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Wrapers.Logitech920;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Wrapers.PinPoint;

import java.util.Objects;

@Autonomous
@Config
public class Samples extends LinearOpMode {
    public enum RobotStates{
        CRUISETOSUBMERSIBLE,
        OCCUPIEDPRELOADS,
        DETECT,
        FAILEDDETECTION,
        GRABFROMSUBMERSIBLE,
        SUBMERSIBLESAMPLECYCLE,
        PARKING,
    }
    public boolean WereInstructionGiven = false;
    public static boolean Wasparked = false;
    public RobotStates robotstate = RobotStates.OCCUPIEDPRELOADS;
    public ElapsedTime time = new ElapsedTime();
    public static
    Logitech920 camera;
    @Override
    public void runOpMode() throws InterruptedException {
        RobotInitializers.InitializeFull(hardwareMap);
        RobotInitializers.InitializeForOperationAuto();

        camera  = new Logitech920("logitik",hardwareMap);
        Scheduler tasks = new Scheduler();
        Scheduler Parkingtasks = new Scheduler();
        tasks.AddAnotherScheduler(FasterPreloadGrabingScoringActions(CHASSIS_sample1pos,EXTENDO_sample1,0.7,LIFT_preload,OVERHEAD_preload,EXTENSION_overbasket-0.1));
        tasks.AddAnotherScheduler(SimultaniosSampleGrabingScoringActions(CHASSIS_sample2pos,EXTENDO_sample2,0.7,LIFT_firstsample,OVERHEAD_firstsample,EXTENSION_secondsample-0.1));
        tasks.AddAnotherScheduler(SimultaniosSampleGrabingScoringActions(CHASSIS_sample3pos,EXTENDO_sample3,0.7,LIFT_secondsample,OVERHEAD_secondsample,EXTENSION_secondsample-0.2));
        tasks.AddAnotherScheduler(PutLastSamplePredefinedActions(LIFT_thirdsample,OVERHEAD_thirdsample,EXTENSION_overbasket));

        Chassis.usedTrajectory = Chassis.trajectoryStates.FREEWILL;
        Lift.DoingAuto = true;
        Extendo.DoingAuto = true;
        //Chassis.setTargetPosition(CHASSIS_turntosubmersibleforcontinuostaking);//to comment


        while(opModeInInit()){

            RobotInitializers.clearCache();

            Lift.update();
            Extendo.update();
            Outtake.update();
            Localizer.Update();
            telemetry.addData("x",Localizer.getCurrentPosition().x);
            telemetry.addData("y",Localizer.getCurrentPosition().y);
            telemetry.addData("h",Localizer.getCurrentPosition().h);
            telemetry.update();
            //Chassis.update();//to comment

            RobotInitializers.Dashtelemetry.addData("x encoder",Localizer.pinPoint.getEncoderX());
            RobotInitializers.Dashtelemetry.addData("y encoder",Localizer.pinPoint.getEncoderY());
        }

        waitForStart();
        time.reset();
        while(opModeIsActive())
        {
            RobotInitializers.clearCache();

            RobotInitializers.Dashtelemetry.addData("is Scheduler done", tasks.tasks.size());

            switch (robotstate){
                case OCCUPIEDPRELOADS:
                    if(Extendo.state  != Extendo.ExtendoStates.RETRACTING && Extendo.getPosition() >= 650 && !Intake.isStorageEmpty() ){
                        Extendo.state = Extendo.ExtendoStates.RETRACTING;
                        Intake.Block();
                        Intake.DropUp();
                    }
                    if(tasks.IsSchedulerDone()) {
                        tasks.AddAnotherScheduler(AfterThirdSampleGoToSubmersibleActions());
                        robotstate = RobotStates.CRUISETOSUBMERSIBLE;
                        camera.startCamera();
                    }
                    break;
                case CRUISETOSUBMERSIBLE:
                    if(tasks.IsSchedulerDone())
                    {
                        robotstate = RobotStates.DETECT;
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
                    }
                    if(tasks.IsSchedulerDone())//and no blur
                    {
                        besttxty = camera.yellow.getBestTxTy();
                        SparkFunOTOS.Pose2D offsets = getPositionRelativeToRobot(besttxty.x, besttxty.y);
                        SparkFunOTOS.Pose2D data = getExtendoRotPair(besttxty.x, besttxty.y);
                        RobotLog.ii("x distance"," " + offsets.x);
                        RobotLog.ii("y distance"," " + offsets.y);

                        RobotLog.ii("extendo distance"," " + data.x);
                        RobotLog.ii("h heading"," " + data.h);
                        robotstate = RobotStates.GRABFROMSUBMERSIBLE;
                        WereInstructionGiven = false;
                    }
                    break;

                case GRABFROMSUBMERSIBLE:
                    if(!WereInstructionGiven){
                        tasks.AddAnotherScheduler(TakeCachedSampleActions(besttxty,0.7));
                        WereInstructionGiven = true;
                    }
                    if(!Intake.HasMixedTeamPiece() && !Intake.isStorageEmpty())
                    {
                        tasks.clear();
                        robotstate = RobotStates.FAILEDDETECTION;
                        WereInstructionGiven = false;
                    }
                    if(tasks.IsSchedulerDone()){
                        RobotLog.ii("color detected", "" + Intake.getStorageStatus());
                        RobotLog.ii("reached trapdoor", "" + Intake.SampleReachedTrapDoor());

                        if(Intake.HasMixedTeamPiece() && Intake.SampleReachedTrapDoor()) {//change in color
                            if(30 - time.seconds() > 1)
                                robotstate = RobotStates.SUBMERSIBLESAMPLECYCLE;
                            RobotLog.ii("accepted","");
                        }
                        else {
                            robotstate = RobotStates.FAILEDDETECTION;
                            RobotLog.ii("rejected","");
                        }
                        WereInstructionGiven = false;
                    }
                    break;
                case FAILEDDETECTION:
                    if(!WereInstructionGiven){
                        tasks.AddAnotherScheduler(FailedToDetectActions());
                        camera.yellow.AddResultToIgnored(getExtendoRotPair(besttxty.x,besttxty.y));
                        WereInstructionGiven = true;
                    }
                    if(tasks.IsSchedulerDone()){
                        robotstate = RobotStates.DETECT;
                        WereInstructionGiven = false;
                    }
                    break;
                case SUBMERSIBLESAMPLECYCLE:
                    if(!WereInstructionGiven){
                        camera.yellow.resetIgnoredSample();
                        tasks.AddAnotherScheduler(GoingBackAndForthToHighBasketAndSubmersibleActions());
                        WereInstructionGiven = true;
                    }
                    if(tasks.IsSchedulerDone()){
                        robotstate = RobotStates.CRUISETOSUBMERSIBLE;
                        WereInstructionGiven = false;
                    }
                    break;
                case PARKING:

                    break;
            }

            RobotInitializers.Dashtelemetry.addData("robot state",robotstate);
            RobotInitializers.Dashtelemetry.addData("new data ready",camera.yellow.newResultsReady);

            RobotInitializers.Dashtelemetry.addData("vel x",Localizer.getVelocity().x);
            RobotInitializers.Dashtelemetry.addData("vel y",Localizer.getVelocity().y);
            RobotInitializers.Dashtelemetry.addData("vel h",Localizer.getVelocity().h);

            if(30 - time.seconds() <= 1 && (robotstate != RobotStates.SUBMERSIBLESAMPLECYCLE || (robotstate == RobotStates.SUBMERSIBLESAMPLECYCLE && Localizer.getDistanceFromTwoPoints(new SparkFunOTOS.Pose2D(-430, -1290,0),Localizer.getCurrentPosition()) < 80 && tasks.tasks.size() <= 3 )) && !Wasparked) {
                Parkingtasks.AddAnotherScheduler(ParkingActions());
                Wasparked = true;
            }

            Chassis.update();
            Lift.update();
            Extendo.update();
            Outtake.update();
            tasks.update();
            Parkingtasks.update();
            Localizer.Update();

            if(time.seconds() > 30)
                break;
        }

    }
}
