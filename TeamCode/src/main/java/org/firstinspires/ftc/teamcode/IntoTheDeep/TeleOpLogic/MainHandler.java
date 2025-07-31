package org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOpLogic;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOpLogic.SampleLogic.DropSampleAndRetract.DropSampleAndRetractActions;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOpLogic.SampleLogic.HighBasketScore.HighBasketScoreActions;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOpLogic.SampleLogic.LowBasketScore.LowBasketScoreActions;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOpLogic.PanicActions.PanicElevatorPos.PanicElevatorDownActions;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOpLogic.PanicActions.PanicElevatorPos.PanicElevatorUpActions;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOpLogic.PanicActions.PanicReset.PanicResetActions;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOpLogic.SwitchToSamples.SwitchToSampsActions;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOpLogic.SwitchToSpecimens.SwitchToSpecsActions;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOpLogic.SampleLogic.TransferSample.TransferSampleActions;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.TeleopsStarter.gm1;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.TeleopsStarter.gm2;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Scheduler;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Extendo;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Intake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Lift;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Outtake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.RobotInitializers;

@Config
public class MainHandler {

    Scheduler currentTasks;
    modes mode = modes.SAMPLES;
    ActionStates CurrentState = ActionStates.SAMPLETRANSFER;

    public MainHandler(){
        currentTasks = new Scheduler();
    }
    public enum ActionStates{
        SAMPLETRANSFER,
        SAMPLEBASKET,
        RETRACTINGSAMPLE,

        SPECIMENGRABANDSTAYIDLE,
        SPECIMENRETRACTFORNEXT,
        SAMPLEOVERHEADOBSERVATIONTHROW,
        SAMPLEOBSERVATIONOPENCLAW,
    }

    public enum modes{
        SAMPLES,
        SPECIMENS,
    }

    public void IntakeActions(){
        if(Extendo.state != Extendo.ExtendoStates.FREEWILL) {
            RobotInitializers.telemetry.addLine("error cannot move extendo in free to move intake");
            return;
        }

        if(gm1.left_bumper && gm1.right_bumper){
            //eject
            Intake.RotateToEject();
            Intake.Unblock();
        }
        else if(gm1.right_bumper) {
            Intake.RotateToStore();
            Intake.DropDown();
            Intake.Unblock();
        }
        else {
            Intake.DropUp();
            if(Intake.HasMixedTeamPiece())
                Intake.Block();
            else
                Intake.Unblock();
        }
    }


    public void update(){

        IntakeActions();

        if(gm2.triangleWasPressed() && currentTasks.IsSchedulerDone()){
            if(mode == modes.SAMPLES && CurrentState == ActionStates.SPECIMENGRABANDSTAYIDLE){
                currentTasks.AddAnotherScheduler(SwitchToSpecsActions);
                mode = modes.SPECIMENS;
                CurrentState = ActionStates.SAMPLETRANSFER;
            }
            else if(CurrentState == ActionStates.SAMPLETRANSFER) {
                currentTasks.AddAnotherScheduler(SwitchToSampsActions);
                mode = modes.SAMPLES;
                CurrentState = ActionStates.SPECIMENGRABANDSTAYIDLE;
            }
        }

        if(gm2.circleWasPressed())
        {
            currentTasks.clear();
            currentTasks.AddAnotherScheduler(PanicResetActions);
            if(mode == modes.SAMPLES)
                CurrentState = ActionStates.SAMPLETRANSFER;
            else
                CurrentState = ActionStates.SPECIMENGRABANDSTAYIDLE;
        }
        if(gm2.IsCircleHeld() && gm2.dpadUpWasPressed())
            currentTasks.AddAnotherScheduler(PanicElevatorUpActions);
        if(gm2.circleWasPressed() && gm2.dpadDownWasPressed())
            currentTasks.AddAnotherScheduler(PanicElevatorDownActions);


        switch (mode){
            case SAMPLES:
                switch (CurrentState){
                    case SAMPLETRANSFER:
                        if(currentTasks.IsSchedulerDone() && (Intake.HasMixedTeamPiece() || gm2.squareWasPressed())) {
                            currentTasks.AddAnotherScheduler(TransferSampleActions);
                            CurrentState = ActionStates.SAMPLEBASKET;
                        }
                        break;
                    case SAMPLEBASKET:
                        if(currentTasks.IsSchedulerDone()) {
                            if(gm2.dpadUpWasPressed())
                                currentTasks.AddAnotherScheduler(HighBasketScoreActions);
                            if(gm2.dpadDownWasPressed())
                                currentTasks.AddAnotherScheduler(LowBasketScoreActions);
                            if(gm1.squareWasPressed())
                                CurrentState = ActionStates.RETRACTINGSAMPLE;
                        }
                        break;
                    case RETRACTINGSAMPLE:
                        currentTasks.AddAnotherScheduler(DropSampleAndRetractActions);
                        CurrentState = ActionStates.SAMPLETRANSFER;
                        break;
                }
                break;
            case SPECIMENS:


                break;
        }


        currentTasks.update();
        Lift.update();
        Extendo.update();
        Outtake.update();
    }
}
