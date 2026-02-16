package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.base;


@Autonomous(name = " StateAuto", group = "A")
public class StateAuto extends OpMode {

    private Follower follower;

    Pose2D pose;
    private Timer pathTimer, actionTimer, opmodeTimer,outtaketimer;
    private DcMotorEx intakeLeft,intakeRight, flyBot, flyTop,leftFront, rightFront, leftBack, rightBack,turretPosition;
    private CRServo turretLeft, turretRight;
    private Servo Hood, Blocker, Tripod;
    private DigitalChannel botBB,topBB,midBB;
    double intakevel,intaketargtvel=0,intakestdvel=1500,outakestdvel=1300,outtakebstvel=1600;
    int shootstep=0;
    private Limelight3A Limelight;
    LLResult result;
    private boolean atopbb,abotbb,amidbb,configured=false,intakefull=false;
    public double hoodLastPos = 0.0,Tx,Ty,  flyCurrentVel,flypower=0.7;
    public double hoodPos = 0,outtaketime=0;
    public int shootState=0 ,turretTarget=0 ;
   public final int preshoot=0,shoot=1,done=2;
    int  turretPos;
    public boolean red=true,recevieinfo=false,adrive=false,limeValid=false,Limelocked=false,fardis=true;
    private boolean row3=false;
    double turnPower;

    double autoTurretOffset = 0;


    private int pathState=0 ,i=0;

    //fardistance
    private final Pose RFstartPose = new Pose(0, 0, Math.toRadians(270)); // Start Pose of our robot.
    private final Pose RFstart1Pose = new Pose(0, 6, Math.toRadians(270)); // Start Pose of our robot.
    private final Pose RFscorePose = new Pose(0, 6, Math.toRadians(0)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose RFpickup1Pose = new Pose(12, 23, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose RFpickup1PoseA = new Pose(38, 23, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose RFpickup2Pose = new Pose(36, -3, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose RFpickup2PoseA = new Pose(37, 10, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose RFpickup2PoseB = new Pose(40, 10, Math.toRadians(0));
    private final Pose RFendPose = new Pose(12, 10, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.


    // BLUE FAR Positions
    private final Pose BFstartPose =   new Pose(0, 0, Math.toRadians(270)); // Start Pose of our robot.

    private final Pose BFstart1Pose = new Pose(0, 6, Math.toRadians(270)); // Start Pose of our robot.

    private final Pose BFscorePose = new Pose(0, 6, Math.toRadians(180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose BFpickup1Pose = new Pose(-12, 23, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose BFpickup1PoseA = new Pose(-38, 23, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose BFpickup2Pose = new Pose(-36, -3, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose BFpickup2PoseA = new Pose(-37, 10, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.

    private final Pose BFpickup2PoseB = new Pose(-40, 10, Math.toRadians(180));

    private final Pose BFendPose = new Pose(-12, 10, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.


    private Path scorePreload;
    private PathChain RFStart1,    RFapproachPickup1, RFgrabPickup1,RFscorePickup1, RF1approchPickup2,RFapproachPickup2,RFgrabPickup2, RFscorePickup2,RFparkEnd;
    private PathChain BFStart1,    BFapproachPickup1, BFgrabPickup1,BFscorePickup1, BF1approchPickup2,BFapproachPickup2,BFgrabPickup2, BFscorePickup2,BFparkEnd;
    private PathChain RNscorstart0;
  //          RFgrabPickup1,RFscorePickup1, RFapproachPickup2,RFgrabPickup2, RFscorePickup2,RFparkEnd;
   // private PathChain BFapproachPickup1, BFgrabPickup1,BFscorePickup1, BFapproachPickup2,BFgrabPickup2, BFscorePickup2,BFparkEnd;
//    private PathChain approachPickup1, grabPickup1,scorePickup1, approachPickup2,grabPickup2, scorePickup2;
//    private PathChain approachPickup1, grabPickup1,scorePickup1, approachPickup2,grabPickup2, scorePickup2;



    //neardistance

    private final Pose RNstartPose = new Pose(0, 0, Math.toRadians(0)); // Start Pose of our robot.
    private final Pose RNscore0Pose = new Pose(-26, -28, Math.toRadians(0)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose RNpickup1Pose = new Pose(-6, -28, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose RNgatePose = new Pose(-1, -38, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose RNpickup2Pose = new Pose(-6, -52, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose RNscore1Pose = new Pose(-40, -38, Math.toRadians(0)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose RNpickup3Pose = new Pose(-6, -76, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose RNendPose = new Pose(12, 10, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose RNscore2Pose = new Pose(-40, -0, Math.toRadians(270));




    public void nearbuildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
//nearside
        RNscorstart0 = follower.pathBuilder()
                .addPath(new BezierLine(RNstartPose, RNscore0Pose))
                .setLinearHeadingInterpolation(RFstartPose.getHeading(), RFstart1Pose.getHeading())
                .build();


//     //   RNapproachPickup1 = follower.pathBuilder()
//                .addPath(new BezierLine(RNscore0Pose, RFpickup1Pose))
//                .setLinearHeadingInterpolation(RFstart1Pose.getHeading(), RFpickup1Pose.getHeading())
//                .build();

        RFgrabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(RFpickup1Pose, RFpickup1PoseA))
                .setLinearHeadingInterpolation(RFpickup1Pose.getHeading(), RFpickup1PoseA.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        RFscorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(RFpickup1PoseA, RFscorePose))
                .setLinearHeadingInterpolation(RFpickup1PoseA.getHeading(), RFscorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        RF1approchPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(RFstartPose, RFpickup2Pose))
                .setLinearHeadingInterpolation(RFstartPose.getHeading(), RFpickup2Pose.getHeading(),0.5)
                .build();


        RFapproachPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(RFscorePose, RFpickup2Pose))
                .setLinearHeadingInterpolation(RFscorePose.getHeading(), RFpickup2Pose.getHeading(),0.5)
                .build();

        RFgrabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(RFpickup2Pose, RFpickup2PoseA))
                // .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                //.setConstantHeadingInterpolation(Math.toRadians(RFpickup2Pose.getHeading()))
                .setConstantHeadingInterpolation( 0)
                .addPath(new BezierLine(RFpickup2PoseA, RFpickup2PoseB))
                .setConstantHeadingInterpolation( 0)
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        RFscorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(RFpickup2Pose, RFscorePose))
                .setLinearHeadingInterpolation(RFpickup2Pose.getHeading(), RFscorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */


        RFparkEnd= follower.pathBuilder()
                .addPath(new BezierLine(RFscorePose, RFendPose))
                .setLinearHeadingInterpolation(RFscorePose.getHeading(), RFendPose.getHeading())
                .build();

        //BF
        BFapproachPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(BFstartPose, BFpickup1Pose))
                .setLinearHeadingInterpolation(BFstartPose.getHeading(), BFpickup1Pose.getHeading())
                .build();

        BFgrabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(BFpickup1Pose, BFpickup1PoseA))
                .setLinearHeadingInterpolation(BFpickup1Pose.getHeading(), BFpickup1PoseA.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        BFscorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(BFpickup1PoseA, BFscorePose))
                .setLinearHeadingInterpolation(BFpickup1PoseA.getHeading(), BFscorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        BFapproachPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(BFscorePose, BFpickup2Pose))
                .setLinearHeadingInterpolation(BFscorePose.getHeading(), BFpickup2Pose.getHeading())
                .build();





        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        BFscorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(BFpickup2Pose, BFscorePose))
                .setLinearHeadingInterpolation(BFpickup2Pose.getHeading(), BFscorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        BFgrabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(BFpickup2Pose, BFpickup2PoseA))
                // .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .setConstantHeadingInterpolation(Math.toRadians(340))
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */

        BFparkEnd= follower.pathBuilder()
                .addPath(new BezierLine(BFscorePose, BFendPose))
                .setLinearHeadingInterpolation(BFscorePose.getHeading(), BFendPose.getHeading())
                .build();


    }




    boolean shooting=false,firstshoot=true;
    public base rbga=new base();

    public void FarPathUpdate() {

        if(opmodeTimer.getElapsedTime()>28000&&pathState<100) {setPathState(101);turretTarget=0;shooting=false;};
        switch (pathState) {

            case 0:


                outtaketimer.resetTimer();

//                rbga.txoffset = 2;


                if(red) follower.followPath(RFStart1, 0.5,true);
                else follower.followPath(BFStart1,0.5, true);


                if(fardis) Hood.setPosition(rbga.hoodfarpos);

                setPathState(1);



                break;
            case 1:
                Ty = -12.0;   //-12.0


                if(ashoot()) {

                if(row3)  setPathState(3);
                else setPathState(11);

                }

                break;

            case 3:

                if(red) follower.followPath(RFapproachPickup1, true);
               else follower.followPath(BFapproachPickup1, true);
               setPathState(4);
               break;

            case 4:

                if (!follower.isBusy()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    if(red) follower.followPath(RFgrabPickup1, 0.35,true);
                    else follower.followPath(BFgrabPickup1, 0.35,true);
                    flypower=rbga.flypower2;
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()|| rbga.beamscanintake(atopbb,amidbb,abotbb)) {

                    if(red) follower.followPath(RFscorePickup1, true);
                    else follower.followPath(BFscorePickup1, true);
                    intaketargtvel=0;
                    setPathState(7);

                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */

                if (!follower.isBusy()) {
                    setPathState(10);
                }
                break;

            case 10:
                if(ashoot()) setPathState(11);
                break;

           case 11:

               if(firstshoot) {
                   if (red) follower.followPath(RF1approchPickup2, false);
                   else follower.followPath(BF1approchPickup2, false);
                   firstshoot=false;
                   setPathState(14);
                   flypower=rbga.flypower2;
                   if (red) turretTarget=-12000;
                   else turretTarget=12000;

                   rbga.turrettarget = turretTarget;
                   break;
               }

                  if (opmodeTimer.getElapsedTime() > 24000)  setPathState(101);
                  else {
                      if (red) follower.followPath(RFapproachPickup2, false);
                      else follower.followPath(BFapproachPickup2, false);
                      setPathState(14);
                      flypower=rbga.flypower2;
                      if (red) turretTarget=-12000;
                      else turretTarget=12000;
                      rbga.turrettarget = turretTarget;
                  }

                break;

            case 14:

                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    if(red)follower.followPath(RFgrabPickup2,0.5, false);
                    else follower.followPath(BFgrabPickup2, 0.5,false);
                    setPathState(17);


                }
                break;

            case 17:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                    intakefull=rbga.beamscanintake(atopbb,amidbb,abotbb);
                if (intakefull||actionTimer.getElapsedTime()>1800) {
                    if(red) follower.followPath(RFscorePickup2,true);
                    else follower.followPath(BFscorePickup2,true);
                    if(intakefull) intaketargtvel=0;
                    setPathState(19);
                }

                break;

            case 19:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */

                if (!follower.isBusy()) {//cycle wall grab and shooting
                    setPathState(10);
                    outtaketimer.resetTimer();
                }
                break;
            case 101:
                        flypower=0;
                        rbga.turret_offset = 0;
                        if(red)follower.followPath(RFparkEnd, true);
                        else follower.followPath(BFparkEnd, true);
                        turretTarget=0;
                        rbga.turrettarget = turretTarget;
                        setPathState(103);


                break;




            case 103: /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    flypower=0;
                    flyprepower(0);
                    Blocker.setPosition(rbga.blockClose);
                    Intake(0);
                    actionTimer.resetTimer();
                    setPathState(105);


                }
                break;

            case 105:
                if(actionTimer.getElapsedTime()>300) {

                    if (red) {
                        blackboard.put("Heading", follower.getPose().getHeading());
                        blackboard.put("X", follower.getPose().getX() + rbga.rfxoffset);
                        //   blackboard.put("Y", follower.getPose().getY() + rbga.REDYOFFSET);

                    } else {
                        blackboard.put("Heading", follower.getPose().getHeading());
                        blackboard.put("X", follower.getPose().getX() + rbga.bfxoffset);
                        //    blackboard.put("Y", follower.getPose().getY() + rbga.BLUEYOFFSET);
                    }
                    blackboard.put("Y", follower.getPose().getY() + rbga.rfyoffset);

                    setPathState(200);
                }
                break;
        }
    }


    public void  NearPathUpdate() {

        if(opmodeTimer.getElapsedTime()>28000&&pathState<100) {setPathState(101);turretTarget=0;shooting=false;};
        switch (pathState) {

            case 0:

                if(red) follower.followPath(RNscorstart0,false);
                else follower.followPath(BFapproachPickup1,0.5, true);
                turretTarget=-12000;
                flypower=0.6;
               // Ty = -12.2;
              Hood.setPosition(rbga.hoodnearpose);

                setPathState(2);



                break;
            case 2:

                if (!follower.isBusy()) { setPathState(3);}


            case 3:
                if(ashoot()) setPathState(5);


            case 5:

                if (!follower.isBusy()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    if(red) follower.followPath(RFgrabPickup1, 0.35,true);
                    else follower.followPath(BFgrabPickup1, 0.35,true);
                    flypower=rbga.flypower2;
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()|| rbga.beamscanintake(atopbb,amidbb,abotbb)) {

                    if(red) follower.followPath(RFscorePickup1, true);
                    else follower.followPath(BFscorePickup1, true);
                    intaketargtvel=0;
                    setPathState(7);

                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */

                if (!follower.isBusy()) {
                    setPathState(10);
                }
                break;

            case 10:
                if(ashoot()) setPathState(11);
                break;

            case 11:

                if(firstshoot) {
                    if (red) follower.followPath(RF1approchPickup2, false);
                    //      else follower.followPath(BFapproachPickup2, false);
                    firstshoot=false;
                    setPathState(14);
                    flypower=rbga.flypower2;
                    if (red) turretTarget=-12000;
                    else turretTarget=12000;
                    rbga.turrettarget = turretTarget;
                    break;
                }

                if (opmodeTimer.getElapsedTime() > 24000)  setPathState(101);
                else {
                    if (red) follower.followPath(RFapproachPickup2, false);
                    else follower.followPath(BFapproachPickup2, false);
                    setPathState(14);
                    flypower=rbga.flypower2;
                    if (red) turretTarget=-12000;
                    else turretTarget=12000;
                    rbga.turrettarget = turretTarget;
                }

                break;

            case 14:

                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    if(red)follower.followPath(RFgrabPickup2, false);
                    else follower.followPath(BFgrabPickup2, false);
                    setPathState(17);


                }
                break;

            case 17:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                intakefull=rbga.beamscanintake(atopbb,amidbb,abotbb);
                if (intakefull||actionTimer.getElapsedTime()>1800) {
                    if(red) follower.followPath(RFscorePickup2,true);
                    else follower.followPath(BFscorePickup2,true);
                    setPathState(19);
                }

                break;

            case 19:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */

                if (!follower.isBusy()) {//cycle wall grab and shooting
                    setPathState(10);
                }
                break;
            case 101:
                flypower=0;
                rbga.turret_offset = 0;
                if(red)follower.followPath(RFparkEnd, true);
                else follower.followPath(BFparkEnd, true);
                turretTarget=0;
                rbga.turrettarget = turretTarget;
                setPathState(103);


                break;




            case 103: /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    flypower=0;
                    flyprepower(0);
                    Blocker.setPosition(rbga.blockClose);
                    Intake(0);
                    actionTimer.resetTimer();
                    setPathState(105);


                }
                break;

            case 105:
                if(actionTimer.getElapsedTime()>300) {

                    if (red) {
                        blackboard.put("Heading", follower.getPose().getHeading());
                        blackboard.put("X", follower.getPose().getX() + rbga.rfxoffset);
                        //   blackboard.put("Y", follower.getPose().getY() + rbga.REDYOFFSET);

                    } else {
                        blackboard.put("Heading", follower.getPose().getHeading());
                        blackboard.put("X", follower.getPose().getX() + rbga.bfxoffset);
                        //    blackboard.put("Y", follower.getPose().getY() + rbga.BLUEYOFFSET);
                    }
                    blackboard.put("Y", follower.getPose().getY() + rbga.rfyoffset);

                    setPathState(200);
                }
                break;
        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {
        follower.update();
        statusupdate();
        FarPathUpdate();
        turntable();
     //   autonomousNearPathUpdate();

        // Feedback to Driver fHub for debugging
//        telemetry.addData("path state", pathState);
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("heading", follower.getPose().getHeading());
//        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        //  rbga=new base();
        Hw_init();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        outtaketimer = new Timer();
        actionTimer = new Timer();
        rbga.init();
        follower = Constants.createFollower(hardwareMap);
        farbuildPaths();
        follower.setStartingPose(RFstartPose);
       // telemetry.addLine("Turn the camera to the shooting target");
        telemetry.addLine("Driver Cross select Blue side");
        telemetry.addLine("Driver Circle select  Red  side");
        telemetry.addLine("Gunner Triangle select 3rd Row Auto");
        telemetry.addLine("Gunner Square deselect 3rd Row Auto");


//        telemetry.addLine("Drive Right Bumper Confrim ");

       telemetry.update();
    }





    @Override
    public void init_loop() {

       if(gamepad2.triangle) {

           row3=true;
       }
        if(gamepad2.square) {

            row3=false;
        }


        if (gamepad1.crossWasPressed()) {
            red = false;
            i=0;
            configured = true;
            recevieinfo = true;
            ;
        }

        if (gamepad1.circleWasPressed()) {
            red = true;
            i=0;
            recevieinfo = true;
            configured=true;
        }
        if(configured){
            if(i==0) {

                if (red) {
                    Limelight.pipelineSwitch(2);

                    autoTurretOffset = -0.5;

                    rbga.targetGoalY=rbga.targetGoalY-rbga.rfyoffset;
                    rbga.targetGoalX=rbga.targetGoalX-rbga.rfxoffset;
                    telemetry.addLine("Red  Selected");
                    Limelight.start();
                } else {
                    autoTurretOffset = 1.25;
                    Limelight.pipelineSwitch(3);

                    rbga.targetGoalY=rbga.targetGoalY-rbga.bfyoffset;
                    rbga.targetGoalX=0-rbga.bfxoffset;
                    telemetry.addLine("Blue Selected");
                    Limelight.start();


                }

                blackboard.put("COLOR",  red);
                setPathState(0);
            }

            if(Limelight.getLatestResult().isValid()){
                configured=false;
                telemetry.addLine("Camera good");
                telemetry.update();

            }
            i++;
            if(i>30) {
                telemetry.addLine("Camera issue, pls restart!!!!!!!");
                configured=false;
                i=0;
                telemetry.update();}

        }

    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        if(i==0){
            if (red) {
                Limelight.pipelineSwitch(2);
                rbga.targetGoalY=rbga.targetGoalY-rbga.rfyoffset;
                rbga.targetGoalX=rbga.targetGoalX-rbga.rfxoffset;
                telemetry.addLine("Red  Selected");
            } else {
                Limelight.pipelineSwitch(3);
                rbga.targetGoalY=rbga.targetGoalY-rbga.bfyoffset;
                rbga.targetGoalX=0-rbga.bfxoffset;
                telemetry.addLine("Blue Selected");
            }
            if(row3)  telemetry.addLine("3rd Row Auto  Selected");
            else telemetry.addLine("N3rd Row Auto Not Selected");
            Limelight.start();
            blackboard.put("COLOR",  red);
            setPathState(0);
            telemetry.update();

        }
        opmodeTimer.resetTimer();


    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        flyBot.setPower(0);
        flyTop.setPower(0);
        intakeLeft.setPower(0);
        intakeRight.setPower(0);
        turretLeft.setPower(0);
        turretRight.setPower(0);
    }
    public void statusupdate()

    {
        turretPos=leftFront.getCurrentPosition();;
        atopbb=topBB.getState();
        abotbb=botBB.getState();
        amidbb=midBB.getState();
        intakevel=intakeLeft.getVelocity();
        Intake(intaketargtvel);
        if(shooting) {
            result = Limelight.getLatestResult();
            limeValid = result.isValid();
            if(limeValid)  {
                Tx=result.getTx();
                Ty=result.getTy();
            }
            flyCurrentVel=flyBot.getVelocity();
//

        }
        else   flyprepower(flypower);



    }

    public void  turntable(){

      //  pose = (new Pose2D(DistanceUnit.INCH, follower.getPose().getX(), follower.getPose().getY(), AngleUnit.RADIANS, follower.getHeading()));

        turnPower= rbga.turretturn(shooting,limeValid,turretTarget,turretPos,Tx, autoTurretOffset);
//        turnPower=rbga.turret(shooting,pose,turretPos,red,true,limeValid,Tx,false);
        turretLeft.setPower(turnPower);
        turretRight.setPower(turnPower);
    }





    public void flyprepower(double power) {
        flyBot.setPower(power);
        flyTop.setPower(power);
    }
    public void stopDriveMotors(){
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }


    public void flywheel() {
    double fpower=rbga.flyspeed(flyCurrentVel,Ty);;
      //  double fpower = rbga.flyspeedPP(flyCurrentVel);
        flyBot.setPower(fpower);
        flyTop.setPower(fpower);

    }




    public boolean ashoot() {

        flywheel();

        switch (shootstep) {
            case 0:

                shooting = true;
               if(intakefull) intaketargtvel=outakestdvel;
               else intaketargtvel=outtakebstvel;
               intakefull=false;
               rbga.Txgap = 30;//avoid to use last time valu
                shootstep = 1;
                outtaketimer.resetTimer();
                break;


            case 1:
                if (rbga.flyspeedgap <= 40 && rbga.Txgap < 1.5  && outtaketimer.getElapsedTime() > 1500) {

                    Blocker.setPosition(rbga.blockOpen);
                    outtaketimer.resetTimer();
                    shootstep = 2;
                }

                telemetry.addData("fly speed gap", rbga.flyspeedgap);
                telemetry.addData("Angle", rbga.Txgap);
                telemetry.update();


                break;
            case 2:


                if (rbga.beamscanouttake(atopbb,amidbb,abotbb) || outtaketimer.getElapsedTime() > 1200) {
                    shootstep = 3;
                    outtaketimer.resetTimer();
                }

                break;
            case 3:
                if(outtaketimer.getElapsedTime()>200) {
                    shooting = false;
                    limeValid = false;
                    flypower= rbga.flypower1;;
                    rbga.limelocked = false;
                    adrive = true;
                   // firstshoot = false;
                    shootstep = 0;
                    Blocker.setPosition(rbga.blockClose);
                    intaketargtvel=intakestdvel;
                    return true;
                }
                break;

        }

        return false;

    }






    public void Intake(double targetvel)

    {
        double power= rbga.intakePID.calculate(intakevel, targetvel) + rbga.intakef * targetvel;

        //  power= Range.clip(power,-0.8,0.8);

        intakeLeft.setPower(power);
        intakeRight.setPower(power);




    }



    public void farbuildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */


    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        RFStart1 = follower.pathBuilder()
                .addPath(new BezierLine(RFstartPose, RFstart1Pose))
                .setLinearHeadingInterpolation(RFstartPose.getHeading(), RFstart1Pose.getHeading())
                .build();


        RFapproachPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(RFstart1Pose, RFpickup1Pose))
                .setLinearHeadingInterpolation(RFstart1Pose.getHeading(), RFpickup1Pose.getHeading())
                .build();

        RFgrabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(RFpickup1Pose, RFpickup1PoseA))
                .setLinearHeadingInterpolation(RFpickup1Pose.getHeading(), RFpickup1PoseA.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        RFscorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(RFpickup1PoseA, RFscorePose))
                .setLinearHeadingInterpolation(RFpickup1PoseA.getHeading(), RFscorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        RF1approchPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(RFstart1Pose, RFpickup2Pose))
                .setLinearHeadingInterpolation(RFstart1Pose.getHeading(), RFpickup2Pose.getHeading(),0.5)
                .build();


        RFapproachPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(RFscorePose, RFpickup2Pose))
                .setLinearHeadingInterpolation(RFscorePose.getHeading(), RFpickup2Pose.getHeading(),0.5)
                .build();

        RFgrabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(RFpickup2Pose, RFpickup2PoseA))
                // .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                //.setConstantHeadingInterpolation(Math.toRadians(RFpickup2Pose.getHeading()))
                .setConstantHeadingInterpolation( 0)
                .addPath(new BezierLine(RFpickup2PoseA, RFpickup2PoseB))
                .setConstantHeadingInterpolation( 0)
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        RFscorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(RFpickup2PoseB, RFscorePose))
                .setLinearHeadingInterpolation(RFpickup2PoseB.getHeading(), RFscorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */


        RFparkEnd= follower.pathBuilder()
                .addPath(new BezierLine(RFscorePose, RFendPose))
                .setLinearHeadingInterpolation(RFscorePose.getHeading(), RFendPose.getHeading())
                .build();

        //BF

        BFStart1 = follower.pathBuilder()
                .addPath(new BezierLine(BFstartPose, BFstart1Pose))
                .setLinearHeadingInterpolation(BFstartPose.getHeading(), BFstart1Pose.getHeading())
                .build();

        BFapproachPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(BFstart1Pose, BFpickup1Pose))
                .setLinearHeadingInterpolation(BFstart1Pose.getHeading(), BFpickup1Pose.getHeading())
                .build();


        BFgrabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(BFpickup1Pose, BFpickup1PoseA))
                .setLinearHeadingInterpolation(BFpickup1Pose.getHeading(), BFpickup1PoseA.getHeading())
                .build();


        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        BFscorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(BFpickup1PoseA, BFscorePose))
                .setLinearHeadingInterpolation(BFpickup1PoseA.getHeading(), BFscorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        BFapproachPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(BFscorePose, BFpickup2Pose))
                .setLinearHeadingInterpolation(BFscorePose.getHeading(), BFpickup2Pose.getHeading())
                .build();

        BF1approchPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(BFstart1Pose, BFpickup2Pose))
                .setLinearHeadingInterpolation(BFstart1Pose.getHeading(), BFpickup2Pose.getHeading(),0.5)
                .build();



        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        BFscorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(BFpickup2PoseB, BFscorePose))
                .setLinearHeadingInterpolation(BFpickup2PoseB.getHeading(), BFscorePose.getHeading())
                .build();




        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        BFgrabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(BFpickup2Pose, BFpickup2PoseA))
                // .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierLine(BFpickup2PoseA, BFpickup2PoseB))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */

        BFparkEnd= follower.pathBuilder()
                .addPath(new BezierLine(BFscorePose, BFendPose))
                .setLinearHeadingInterpolation(BFscorePose.getHeading(), BFendPose.getHeading())
                .build();


    }



    public void Hw_init() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");


        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);


      //  turretPosition=hardwareMap.get(DcMotorEx.class, "leftFront");

      //  Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        flyBot = hardwareMap.get(DcMotorEx.class, "flyBot");
        flyTop = hardwareMap.get(DcMotorEx.class, "flyTop");
        intakeLeft = hardwareMap.get(DcMotorEx.class,"intakeLeft");
        intakeRight = hardwareMap.get(DcMotorEx.class,"intakeRight");

        turretLeft = hardwareMap.get(CRServo.class,"turretLeft");
        turretRight = hardwareMap.get(CRServo.class, "turretRight");
//        turretSpin = hardwareMap.get(DcMotorEx.class, "turretSpin");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());//todo
        botBB = hardwareMap.get(DigitalChannel.class, "botBB");
        topBB = hardwareMap.get(DigitalChannel.class, "topBB");
        midBB = hardwareMap.get(DigitalChannel.class, "midBB");
        Hood = hardwareMap.get(Servo.class, "Hood");
        Blocker = hardwareMap.get(Servo.class, "Blocker");
        Tripod = hardwareMap.get(Servo.class, "Tripod");
        Limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
//        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        Intake.setVelocity(0);
//        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flyBot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flyBot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flyTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flyBot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyBot.setDirection(DcMotorSimple.Direction.REVERSE);
        flyTop.setDirection(DcMotorSimple.Direction.FORWARD);
        Hood.setDirection(Servo.Direction.REVERSE);
//        turretSpin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        turretSpin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        turretSpin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Blocker.setPosition(rbga.blockClose);
        Tripod.setPosition(rbga.tripodIdle);

        intakeLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);

        turretLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        turretRight.setDirection(DcMotorSimple.Direction.REVERSE);


//        Flylut.add(-13.5,1750); //far
//
//        Flylut.add(-12.7,1700); //far
//
//        Flylut.add(-11.6 , 1550); // far
//
//
//
//
//        Flylut.add(-9.27,1400); //close
//
//
//        Flylut.add(-2.71,1240); //close
//
//        Flylut.add(6.28,1060); //close
//
//        Flylut.add(11 , 1000); // close
//
//
//
//        Hoodlut.add(-13.5,0.78);   //far
//
//        Hoodlut.add(-12.7,0.75);   //far
//
//        Hoodlut.add(-11.6 ,0.7);    //far
//
//
//
//        Hoodlut.add(-9.27,0.55);
//
//        Hoodlut.add(-2.71,0.45);
//
//        Hoodlut.add(6.28,0.2);
//
//        Hoodlut.add(11,0.18);




// far hood pos 0.48 power 0.9

//        Flylut.add(-15, 0.92);  //far 0.89
//
//        Flylut.add(-14, 0.895);  //far 0.87
//        Flylut.add(-13.8, 0.9);  //far 0.85
//
//        Flylut.add(-13, 0.878);  //far 0.85
//        Flylut.add(-12.8, 0.863);  //far   0.82
//        Flylut.add(-11.5, 0.82);  //far   0.82
//
//        Flylut.add(-10.5, 0.77); //+1 // Input camera Ty, Output flywheel power
//        Flylut.add(-9.55, 0.744);   // - 9.55 0.78 (2.0 hood)
//        Flylut.add(-9.00, 0.735);   // - 9.55 0.78 (2.0 hood)
//        Flylut.add(-8.70, 0.73);   // - 9.55 0.78 (2.0 hood)
//        Flylut.add(-6.55, 0.7);// -6.55 0.74
//        Flylut.add(-0.59, 0.63); // - 0.59 0.7
//        Flylut.add(3.65, 0.6); // 3.65 0.67
//
//        Flylut.add(11, 0.58); // 10 0.67 ; 0.0 hood
//
//        // NEAR HOOD ANGLES
//        Hoodlut.add(-10.5, 0.59);  //close
//
////        Hoodlut.add(-9.5 , 0.5);  //close
//
//        Hoodlut.add(-6.5, 0.48);  //close
//
//        Hoodlut.add(-0.65, 0.25);  //close
//        Hoodlut.add(4, 0.19);  //close
//        Hoodlut.add(11, 0);  //close


//        Flylut.createLUT();
//
//        Hoodlut.createLUT();


    }




}