package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.base;


@Autonomous(name = " StateAutoNear", group = "A")
public class StateAutoNear extends OpMode {

    private Follower follower;


    private Timer pathTimer, actionTimer, opmodeTimer,outtaketimer;
    private DcMotorEx intakeLeft,intakeRight, flyBot, flyTop,leftFront, rightFront, leftBack, rightBack,turretPosition;
    private CRServo turretLeft, turretRight;
    private Servo Hood, Blocker, Tripod;
    private DigitalChannel botBB,topBB,midBB;
    double intakevel,intaketargtvel=0,intakestdvel=1500,outakestdvel=1300,outtakebstvel=1600;
    int shootstep=0;
    private Limelight3A Limelight;
    LLResult result;
    private boolean atopbb,abotbb,amidbb,configured=false,intakefull=true;
    public double hoodLastPos = 0.0,Tx,Ty,  flyCurrentVel,flypower=0.7;
    public double hoodPos = 0,outtaketime=0;
    public int shootState=0 ,turretTarget=0 ,turretredtarget1=-2333,turretbluetarget1=2333,turretredtarget2=-10000,turretbluetarget2=10000;
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
    private final Pose RFpickup2Pose = new Pose(36, -2, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose RFpickup2PoseA = new Pose(34, 8, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose RFpickup2PoseB = new Pose(40, 8, Math.toRadians(0));
    private final Pose RFendPose = new Pose(12, 10, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.


    // BLUE FAR Positions
    private final Pose BFstartPose =   new Pose(0, 0, Math.toRadians(270)); // Start Pose of our robot.

    private final Pose BFstart1Pose = new Pose(0, 6, Math.toRadians(270)); // Start Pose of our robot.

    private final Pose BFscorePose = new Pose(0, 6, Math.toRadians(180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose BFpickup1Pose = new Pose(-12, 23, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose BFpickup1PoseA = new Pose(-38, 23, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose BFpickup2Pose = new Pose(-36, -2, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose BFpickup2PoseA = new Pose(-34, 8, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.

    private final Pose BFpickup2PoseB = new Pose(-40, 8, Math.toRadians(180));

    private final Pose BFendPose = new Pose(-12, 10, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.


    private Path scorePreload;
    private PathChain RFStart1,   RFapproachPickup1, RFgrabPickup1,RFscorePickup1, RF1approchPickup2,RFapproachPickup2,RFgrabPickup2B, RFgrabPickupB2,RFgrabPickup2B2,RFscorePickup2,RFparkEnd;
    private PathChain BFStart1,    BFapproachPickup1, BFgrabPickup1,BFscorePickup1, BF1approchPickup2,BFapproachPickup2,BFgrabPickup2B, BFgrabPickup2B2,BFgrabPickupB2,BFscorePickup2,BFparkEnd;
    private PathChain RNscorstart0,RNapproachPickup1, RNopenGate1,RNscoreGate1,RNapproachPickup2, RNgrabPickup2,RNopenGate2, RNscoreGate2,RNapproachPickup3, RNgrabPickup3,RNscorePickup3,RNparkEnd;
    private PathChain BNscorstart0,BNapproachPickup1, BNopenGate1,BNscoreGate1,BNapproachPickup2, BNgrabPickup2,BNopenGate2, BNscoreGate2,BNapproachPickup3, BNgrabPickup3,BNscorePickup3;


    //neardistance

    private final Pose RNstartPose = new Pose(0, 0, Math.toRadians(0)); // Start Pose of our robot.
    private final Pose RNscore0Pose = new Pose(-34, -27, Math.toRadians(315)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose RNpickup1Pose = new Pose(-5, -28, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose RNgatePose = new Pose(-1, -38, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose RNgatePoseC1 = new Pose(-6, -33, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose RNgatePoseC2 = new Pose(-6, -43, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Ma

    private final Pose RNpickup2PoseA = new Pose(-29, -52, Math.toRadians(0)); // Middle (Second Set) of Artifacts fro
    private final Pose RNpickup2Pose = new Pose(-5, -52, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose RNscore1Pose = new Pose(-34, -28, Math.toRadians(315)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose RNpickup3PoseA = new Pose(-29, -74, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose RNpickup3Pose = new Pose(-10, -77, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose RNscore3Pose = new Pose(-40, -8, Math.toRadians(315));


    private final Pose BNstartPose = new Pose(0, 0, Math.toRadians(180)); // Start Pose of our robot.
    private final Pose BNscore0Pose = new Pose(34, -27, Math.toRadians(225)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose BNpickup1Pose = new Pose(5, -28, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose BNgatePose = new Pose(1, -38, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose BNgatePoseC1 = new Pose(6, -33, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose BNgatePoseC2 = new Pose(6, -43, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Ma

    private final Pose BNpickup2PoseA = new Pose(29, -52, Math.toRadians(180)); // Middle (Second Set) of Artifacts fro
    private final Pose BNpickup2Pose = new Pose(5, -52, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose BNscore1Pose = new Pose(34, -28, Math.toRadians(225)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose BNpickup3PoseA = new Pose(29, -74, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose BNpickup3Pose = new Pose(10, -77, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose BNscore3Pose = new Pose(40, -8, Math.toRadians(225));



    public void nearbuildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
//nearside
        RNscorstart0 = follower.pathBuilder()
                .addPath(new BezierLine(RNstartPose, RNscore0Pose))
                .setLinearHeadingInterpolation(RNstartPose.getHeading(), RNscore0Pose.getHeading())
               // .setBrakingStrength(0.6)
                .build();
        RNapproachPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(RNscore0Pose, RNpickup1Pose))
                .setLinearHeadingInterpolation(RNscore0Pose.getHeading(), RNpickup1Pose.getHeading(),0.3)
                .setBrakingStrength(0.6)
                .build();
        RNscoreGate1 = follower.pathBuilder()
                .addPath(new BezierLine(RNgatePose, RNscore1Pose))
                .setLinearHeadingInterpolation(RNgatePose.getHeading(), RNscore1Pose.getHeading())
              //  .setBrakingStrength(0.6)
                .build();
        RNopenGate1 = follower.pathBuilder()
                .addPath(new BezierCurve(RNpickup1Pose,RNgatePoseC1,RNgatePose))
                .setConstantHeadingInterpolation( 0)
                .setTimeoutConstraint(500)
                .build();

        RNopenGate2 = follower.pathBuilder()
                .addPath(new BezierCurve(RNpickup2Pose,RNgatePoseC2,RNgatePose))
                .setConstantHeadingInterpolation( 0)
                .setTimeoutConstraint(500)
                .build();

        RNscoreGate2 = follower.pathBuilder()
                .addPath(new BezierLine(RNgatePose, RNscore1Pose))
                .setLinearHeadingInterpolation(RNgatePose.getHeading(), RNscore1Pose.getHeading())
                .build();
        RNapproachPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(RNscore1Pose, RNpickup2PoseA))
                .setLinearHeadingInterpolation(RNscore1Pose.getHeading(), RNpickup2PoseA.getHeading())
                .setBrakingStrength(0.6)

                .build();


        RNgrabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(RNpickup2PoseA, RNpickup2Pose))
                .setConstantHeadingInterpolation( 0)
               // .setBrakingStrength(0.6)
                .build();

        RNapproachPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(RNscore1Pose, RNpickup3PoseA))
                .setLinearHeadingInterpolation(RNscore1Pose.getHeading(), RNpickup3PoseA.getHeading())
                 .build();
        RNgrabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(RNpickup3PoseA, RNpickup3Pose))
                .setConstantHeadingInterpolation( 0)
            //    .setBrakingStrength(0.6)
                .build();

        RNscorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(RNpickup3Pose, RNscore3Pose))
                .setLinearHeadingInterpolation(RNpickup3Pose.getHeading(), RNscore3Pose.getHeading())
                //.setBrakingStrength(0.6)
                .build();




        BNscorstart0 = follower.pathBuilder()
                .addPath(new BezierLine(BNstartPose, BNscore0Pose))
                .setLinearHeadingInterpolation(BNstartPose.getHeading(), BNscore0Pose.getHeading())
                // .setBrakingStrength(0.6)
                .build();
        BNapproachPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(BNscore0Pose, BNpickup1Pose))
                .setLinearHeadingInterpolation(BNscore0Pose.getHeading(), BNpickup1Pose.getHeading(),0.3)
                .setBrakingStrength(0.6)
                .build();
        BNscoreGate1 = follower.pathBuilder()
                .addPath(new BezierLine(BNgatePose, BNscore1Pose))
                .setLinearHeadingInterpolation(BNgatePose.getHeading(), BNscore1Pose.getHeading())
                //  .setBrakingStrength(0.6)
                .build();
        BNopenGate1 = follower.pathBuilder()
                .addPath(new BezierCurve(BNpickup1Pose,BNgatePoseC1,BNgatePose))
                .setConstantHeadingInterpolation( Math.toRadians(180))
                .setTimeoutConstraint(500)
                .build();

        BNopenGate2 = follower.pathBuilder()
                .addPath(new BezierCurve(BNpickup2Pose,BNgatePoseC2,BNgatePose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setTimeoutConstraint(500)
                .build();

        BNscoreGate2 = follower.pathBuilder()
                .addPath(new BezierLine(BNgatePose, BNscore1Pose))
                .setLinearHeadingInterpolation(BNgatePose.getHeading(), BNscore1Pose.getHeading())
                .build();
        BNapproachPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(BNscore1Pose, BNpickup2PoseA))
                .setLinearHeadingInterpolation(BNscore1Pose.getHeading(), BNpickup2PoseA.getHeading())
                .build();


        BNgrabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(BNpickup2PoseA, BNpickup2Pose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                // .setBrakingStrength(0.6)
                .build();

        BNapproachPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(BNscore1Pose, BNpickup3PoseA))
                .setLinearHeadingInterpolation(BNscore1Pose.getHeading(), BNpickup3PoseA.getHeading())
                .build();
        BNgrabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(BNpickup3PoseA, BNpickup3Pose))
                .setConstantHeadingInterpolation( Math.toRadians(180))
                //    .setBrakingStrength(0.6)
                .build();

        BNscorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(BNpickup3Pose, BNscore3Pose))
                .setLinearHeadingInterpolation(BNpickup3Pose.getHeading(), BNscore3Pose.getHeading())
                //.setBrakingStrength(0.6)
                .build();




    }




    boolean shooting=false,firstshoot=true;
    public base rbga=new base();

    public void FarPathUpdate() {

       if(opmodeTimer.getElapsedTime()>28000&&pathState<100) {setPathState(101);};
        switch (pathState) {

            case 0:

                if(red) {

                    follower.followPath(RFStart1, 0.5,true);
                    rbga.txoffset=-2;
                    autoTurretOffset = -2.5;
                    turretTarget=turretredtarget1;

                }
                else {

                    follower.followPath(BFStart1,0.5, true);
                    rbga.txoffset=2;
                    autoTurretOffset = 4;
                    turretTarget=turretbluetarget1;


                }

                if(fardis) Hood.setPosition(rbga.hoodfarpos);

                setPathState(1);

                break;
            case 1:
                Ty = -11.8;   //-12.0

                if(ashoot()) {

                    if (red) turretTarget=turretredtarget2;
                    else turretTarget=turretbluetarget2;

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
                    if(red) follower.followPath(RFgrabPickup1, 0.45,true);
                    else follower.followPath(BFgrabPickup1, 0.45,true);
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
               if (opmodeTimer.getElapsedTime() > 24000)  {setPathState(101);break;}
               if(firstshoot) {
                   if (red) follower.followPath(RF1approchPickup2, false);
                   else follower.followPath(BF1approchPickup2, false);
                   firstshoot=false;
               }
              else {
                      if (red) follower.followPath(RFapproachPickup2, false);
                      else follower.followPath(BFapproachPickup2, false);
                  }

               setPathState(14);
               flypower=rbga.flypower2;
               if (red) turretTarget=turretredtarget2;
               else turretTarget=turretbluetarget2;
               break;

            case 14:


                if (!follower.isBusy()) {
                 //   actionTimer.resetTimer();
                    if(red)follower.followPath(RFgrabPickup2B2, 0.9,false);
                    else follower.followPath(BFgrabPickup2B2,0.9, false);
                    setPathState(18);

                }
                break;





            case 18:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                intakefull=rbga.beamscanintake(atopbb,amidbb,abotbb);
                if (!follower.isBusy()||intakefull){
                    if(red)follower.followPath(RFscorePickup2, true);
                    else follower.followPath(BFscorePickup2, true);
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
                        shooting=false;
                        turretTarget=0;
                        flypower=0;
                        rbga.turret_offset = 0;
                        if(red)follower.followPath(RFparkEnd, true);
                        else follower.followPath(BFparkEnd, true);
                        Intake(0);
                        setPathState(103);
                        Blocker.setPosition(rbga.blockClose);

                break;




            case 103: /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()&& Math.abs(turretPos)<200 ) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */

                    actionTimer.resetTimer();
                    setPathState(105);
                    turretLeft.setPower(0);
                    turretRight.setPower(0);




                }
                break;

            case 105:
                if(actionTimer.getElapsedTime()>500) {

                    if (red) {
                        blackboard.put("Heading", follower.getPose().getHeading());
                        blackboard.put("X", follower.getPose().getX() + rbga.rnxoffset);
                        //   blackboard.put("Y", follower.getPose().getY() + rbga.REDYOFFSET);

                    } else {
                        blackboard.put("Heading", follower.getPose().getHeading());
                        blackboard.put("X", follower.getPose().getX() + rbga.bnxoffset);
                        //    blackboard.put("Y", follower.getPose().getY() + rbga.BLUEYOFFSET);
                    }
                    blackboard.put("Y", follower.getPose().getY() + rbga.rnyoffset);

                    blackboard.put("T", turretPos);

                    setPathState(200);
                }
                break;
        }
    }


    public void  NearPathUpdate() {

        if(opmodeTimer.getElapsedTime()>28000&&pathState<100) {setPathState(102);turretTarget=0;shooting=false;};
        switch (pathState) {

            case 0:

                if(red) {follower.followPath(RNscorstart0,false);  turretTarget=turretredtarget2;autoTurretOffset = -3;}
                else {follower.followPath(BNscorstart0,false);  turretTarget=turretbluetarget2;autoTurretOffset = 3;}
               rbga.flypower2=0.6;

               // Ty = -12.2;
              Hood.setPosition(rbga.hoodnearpose);
              setPathState(2);

                break;
            case 2:

                if (!follower.isBusy()) { setPathState(3);}
                break;


            case 3:
                if(ashoot()) {
                    if(red) follower.followPath(RNapproachPickup1,0.8,false);
                    else   follower.followPath(BNapproachPickup1,0.8,false);

                    setPathState(6);

                }
                break;

            case 6:
                    intakefull=rbga.beamscanintake(atopbb,amidbb,abotbb);
                if (!follower.isBusy()|| intakefull) {

                    if(red) follower.followPath(RNopenGate1,true);
                    else follower.followPath(BNopenGate1, true);
                    flypower=rbga.flypower2;
                    if(intakefull) intaketargtvel=0;
                    setPathState(10);
                }
                break;
            case 10:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {

                    if(red) follower.followPath(RNscoreGate1, true);
                    else follower.followPath(BNscoreGate1, true);

                    setPathState(12);

                }
                break;
            case 12:
            /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
            if (!follower.isBusy()) {



                setPathState(14);

            }
            break;



            case 14:
                if(ashoot()) {
                    if (red) follower.followPath(RNapproachPickup2,  false);
                    else follower.followPath(BNapproachPickup2,  false);
                    setPathState(15);
                }
                break;


            case 15:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {

                    if(red) follower.followPath(RNgrabPickup2, 0.45,true);
                    else follower.followPath(BNgrabPickup2, 0.45,true);

                    setPathState(16);

                }
                break;


            case 16:

                intakefull=rbga.beamscanintake(atopbb,amidbb,abotbb);
                if (!follower.isBusy()|| intakefull) {

                    if(red) follower.followPath(RNopenGate2, true);
                    else follower.followPath(BNopenGate2, true);
                    flypower=rbga.flypower2;
                    if(intakefull) intaketargtvel=0;
                    setPathState(18);
                }
                break;


            case 18:

                if (!follower.isBusy()) {

                    if(red)follower.followPath(RNscoreGate2, false);
                    else follower.followPath(BNscoreGate2, false);
                    setPathState(20);


                }
                break;

            case 20:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {//cycle wall grab and shooting
                    setPathState(22);
                }
                break;


            case 22:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(ashoot()) {
                    if (red) follower.followPath(RNapproachPickup3,  false);
                    else follower.followPath(BNapproachPickup3,  false);
                    setPathState(23);
                    autoTurretOffset=1;
                }

                break;
            case 23:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {

                    if(red) follower.followPath(RNgrabPickup3, 0.45,true);
                    else follower.followPath(BNgrabPickup3, 0.45,true);

                    setPathState(25);

                }
                break;

            case 25:
                intakefull=rbga.beamscanintake(atopbb,amidbb,abotbb);
                if (!follower.isBusy()|| intakefull) {

                    if(red) follower.followPath(RNscorePickup3, true);
                    else follower.followPath(BNscorePickup3, true);
                    flypower=rbga.flypower2;
                    if(intakefull) intaketargtvel=0;
                    setPathState(27);
                }
                break;
            case 27:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {//cycle wall grab and shooting
                    setPathState(29);
                }
                break;


            case 29:
                if(ashoot()) {


                    setPathState(102);


                }

                break;


            case 102:
                shooting=false;
                flypower=0;
                rbga.turret_offset = 0;
                intaketargtvel=0;
                turretTarget=0;

                Blocker.setPosition(rbga.blockClose);
                setPathState(103);

                break;


            case 103:
                if (Math.abs(turretPos) < 200){
                    turretLeft.setPower(0);
                    turretRight.setPower(0);

                    actionTimer.resetTimer();
                    setPathState(105);

                }
                break;



            case 105:
                if(actionTimer.getElapsedTime()>300) {
                    if (red) {
                        blackboard.put("Heading", follower.getPose().getHeading());
                        blackboard.put("X", follower.getPose().getX() + rbga.rnxoffset);
                        //   blackboard.put("Y", follower.getPose().getY() + rbga.REDYOFFSET);

                    } else {
                        blackboard.put("Heading", follower.getPose().getHeading());
                        blackboard.put("X", follower.getPose().getX() + rbga.bnxoffset);
                        //    blackboard.put("Y", follower.getPose().getY() + rbga.BLUEYOFFSET);
                    }
                    blackboard.put("Y", follower.getPose().getY() + rbga.rnyoffset);

                    blackboard.put("T", turretPos);

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
     //  FarPathUpdate();
        NearPathUpdate();
        turntable();


        // Feedback to Driver fHub for debugging
     telemetry.addData("path state", pathState);
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("heading", follower.getPose().getHeading());
    telemetry.update();
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
        setPathState(0);
        follower = Constants.createFollower(hardwareMap);
      //  farbuildPaths();
       nearbuildPaths();



       // follower.setStartingPose(RFstartPose);
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
                    Limelight.pipelineSwitch(6);


                    rbga.targetGoalY=rbga.targetGoalY-rbga.rfyoffset;
                    rbga.targetGoalX=rbga.targetGoalX-rbga.rfxoffset;
                    telemetry.addLine("Red  Selected");
                    Limelight.start();
                } else {
                    autoTurretOffset = 4;
                    Limelight.pipelineSwitch(7);

                    rbga.targetGoalY=rbga.targetGoalY-rbga.bfyoffset;
                    rbga.targetGoalX=0-rbga.bfxoffset;
                    telemetry.addLine("Blue Selected");
                    Limelight.start();


                }

                blackboard.put("COLOR",  red);

            }

            if(Limelight.getLatestResult().isValid()){
                configured=false;
                telemetry.addLine("Camera good");
                telemetry.update();

            }
            i++;
            if(i>40) {
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
                Limelight.pipelineSwitch(6);
                rbga.targetGoalY=rbga.targetGoalY-rbga.rfyoffset;
                rbga.targetGoalX=rbga.targetGoalX-rbga.rfxoffset;
                telemetry.addLine("Red  Selected");

            } else {
                Limelight.pipelineSwitch(7);
                rbga.targetGoalY=rbga.targetGoalY-rbga.bfyoffset;
                rbga.targetGoalX=0-rbga.bfxoffset;
                telemetry.addLine("Blue Selected");
            }
            if(row3)  telemetry.addLine("3rd Row Auto  Selected");
            else telemetry.addLine("N3rd Row Auto Not Selected");
            Limelight.start();
            blackboard.put("COLOR",  red);

            telemetry.update();

        }
        if (red) follower.setStartingPose(RNstartPose);
        else follower.setStartingPose(BNstartPose);

        opmodeTimer.resetTimer();
        setPathState(0);

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
            else Ty = -11.8;
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
               intakefull=false;
               rbga.Txgap = 30;//avoid to use last time valu
                shootstep = 1;
                outtaketimer.resetTimer();
                break;


            case 1:
                if (rbga.flyspeedgap <= 40 && rbga.Txgap < 1.5  && outtaketimer.getElapsedTime() > 600) {
                    if(intakefull) intaketargtvel=outakestdvel;
                    else intaketargtvel=outtakebstvel;
                    Blocker.setPosition(rbga.blockOpen);
                    outtaketimer.resetTimer();
                    shootstep = 2;
                }

//                telemetry.addData("fly speed gap", rbga.flyspeedgap);
//                telemetry.addData("Angle", rbga.Txgap);
//                telemetry.addData("Angle", Ty);
//                telemetry.addData("Turret pos",turretPos);
//                telemetry.update();


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
                .setLinearHeadingInterpolation(RFscorePose.getHeading(), RFpickup2Pose.getHeading())
                .build();

        RFgrabPickup2B2 = follower.pathBuilder()
                .addPath(new BezierLine(RFpickup2Pose, RFpickup2PoseA))
                .setConstantHeadingInterpolation( 0)
                .setTimeoutConstraint(300)
                .addPath(new BezierLine(RFpickup2PoseA, RFpickup2PoseB))
                .setConstantHeadingInterpolation( 0)
                .addPath(new BezierLine(RFpickup2PoseB, RFpickup2PoseA))
                .setConstantHeadingInterpolation( 0)
                .setTimeoutConstraint(300)
                .addPath(new BezierLine(RFpickup2PoseA, RFpickup2Pose))
                .setConstantHeadingInterpolation( 0)

                .build();



        RFgrabPickup2B = follower.pathBuilder()
                .addPath(new BezierLine(RFpickup2Pose, RFpickup2PoseA))
                .setConstantHeadingInterpolation( 0)
                .addPath(new BezierLine(RFpickup2PoseA, RFpickup2PoseB))
                .setConstantHeadingInterpolation( 0)
                .build();


        RFgrabPickupB2 = follower.pathBuilder()
                .addPath(new BezierLine(RFpickup2PoseB, RFpickup2PoseA))
                .setConstantHeadingInterpolation( 0)
                .addPath(new BezierLine(RFpickup2PoseA, RFpickup2Pose))
                .setConstantHeadingInterpolation( 0)
                .build();




        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        RFscorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(RFpickup2PoseA, RFscorePose))
                .setConstantHeadingInterpolation( 0)
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
                .addPath(new BezierCurve(BFpickup2PoseA, BFscorePose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();




        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        BFgrabPickup2B = follower.pathBuilder()
                .addPath(new BezierLine(BFpickup2Pose, BFpickup2PoseA))
                // .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierLine(BFpickup2PoseA, BFpickup2PoseB))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        BFgrabPickup2B2 = follower.pathBuilder()
                .addPath(new BezierLine(BFpickup2Pose, BFpickup2PoseA))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setTimeoutConstraint(300)
                .addPath(new BezierLine(BFpickup2PoseA, BFpickup2PoseB))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierLine(BFpickup2PoseB, BFpickup2PoseA))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setTimeoutConstraint(300)
                .addPath(new BezierLine(BFpickup2PoseA, BFpickup2Pose))
                .setConstantHeadingInterpolation(Math.toRadians(180))

                .build();
        BFgrabPickupB2 = follower.pathBuilder()
                .addPath(new BezierLine(BFpickup2PoseB, BFpickup2PoseA))
               // .setTValueConstraint(300)
                // .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierLine(BFpickup2PoseA, BFpickup2Pose))
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


        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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