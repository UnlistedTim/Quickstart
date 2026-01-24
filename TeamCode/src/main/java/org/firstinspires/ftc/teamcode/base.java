
package org.firstinspires.ftc.teamcode;



import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.InterpLUT;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.Arrays;

public class base {


    Limelight3A limelight;
  //  LLResult result = limelight.getLatestResult();

  //  double Tx=0,Ty=0;
   public boolean limelocked=false;
    InterpLUT Flylut = new InterpLUT();
    InterpLUT Hoodlut = new InterpLUT();
    //public static double turretkP = 0.025, turretkI = 0.05, turretkD = 0.002;//
    public static double turretkP = 0.028, turretkI = 0.05, turretkD = 0.002;//
    public static double flyp = 0.002, flyi = 0, flyd = 0, flyf = 0.0005;
    public double flyspeedgap=500,Txgap=50,turnMax=0.3;


    public double targetVel=0;
    public double blockClose = 0.3, blockOpen = 0.65;
    public double tripodIdle = 0.95, tripodPark = 0.27;


    //double turretPower=0;
    PIDController turretPID = new PIDController(turretkP, turretkI, turretkD);
    PIDController flyPID = new PIDController(flyp, flyi, flyd);
    double Tx_offset=0;
    int turretCwlim=-200;
    int turretCcwlim=200;
    public MedianFilter intakeCurrentFilter = new MedianFilter(10);

    public double intakeVel = 2500,outtakVel=2599;



    public class MedianFilter {
        private final double[] window;
        private int index = 0;
        private boolean filled = false;

        public MedianFilter(int size) {
            window = new double[size];
        }

        public double update(double value) {
            window[index] = value;
            index = (index + 1) % window.length;

            if (index == 0) {
                filled = true;
            }

            double[] temp = filled
                    ? window.clone()
                    : Arrays.copyOf(window, index);

            Arrays.sort(temp);

            return temp[temp.length / 2];
        }
    }


    public void init()

    {

        flyPID.setPID(flyp, flyi, flyd);
        turretPID.setPID(turretkP, turretkI, turretkD);
        Flylut.add(-13.5,1720); //far
        Flylut.add(-12.7,1680); //far
        Flylut.add(-12,1570); //far
        Flylut.add(-11.6 , 1530); // far
        Flylut.add(-9.27,1440); //close
        Flylut.add(-2.71,1240); //close
        Flylut.add(6.28,1060); //close
        Flylut.add(11 , 1000); // close

        Hoodlut.add(-13.5,0.78);   //far
        Hoodlut.add(-12.7,0.75);   //far
        Hoodlut.add(-11.6 ,0.7);    //far
        Hoodlut.add(-9.27,0.55);
        Hoodlut.add(-2.71,0.45);
        Hoodlut.add(6.28,0.2);
        Hoodlut.add(11,0.18);


        Flylut.createLUT();

        Hoodlut.createLUT();


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





    }




    public double  turretturn(boolean outtake , boolean valid,int target, int turretPos, double tx, double offset){
        double turretPower;
        if (outtake)  {
            if(valid) {
                turretPower = turretPID.calculate(tx, offset);
                limelocked=true;
                Txgap=Math.abs(tx-offset);
                if (turretPos > turretCcwlim- 10 &&  turretPower  > 0) {
                    turretPower= -0.3;
//                    target = 0;
                    limelocked = false;
                }
                if (turretPos < (turretCwlim + 10) && turretPower< 0) {
                    turretPower= 0.3;
//                    target = 0;
                    limelocked = false;
                }


                return  turretPower ;
            }

             if (limelocked) {
                turretPower=0;
                return turretPower;
            }


        }




        turretPower = turretPID.calculate( turretPos*0.35,  target*0.35);



        return turretPower;



    }

    public double flyhood(double Ty) {

        double hoodLutGet;
        if (Ty < 11 && Ty > -13.5) {


            hoodLutGet = Hoodlut.get(Ty);
        }

        else hoodLutGet=0;

        return hoodLutGet;


    }

    public double getVel(double ty){
        return Flylut.get(ty);
    }

    public double  flyspeed(double currentVel,double ty) {


        if (ty < 11 && ty > -13.5) {

            targetVel = Flylut.get(ty);// Tx offset

//            Tx_offset=0;

        }
        else targetVel = 0.75;



        targetVel = Math.round(targetVel / 0.001) * 0.001;

        flyspeedgap=Math.abs(currentVel-targetVel);


     //   double power = flyPID.calculate(flyCurrentVel, targetVel) + flyf * targetVel;

        return(flyPID.calculate(currentVel, targetVel) + flyf * targetVel);


    }



}

//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.controller.PIDController;
//import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
//import com.arcrobotics.ftclib.util.InterpLUT;
//
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//
//import java.util.List;
//
//public class bass {
//
//
//
//    public void limeAutoAimer(boolean lim , int target, int turretPos, Limelight3A limelight){
//
////        int turretPos=turretSpin.getCurrentPosition();
//        int id;
//
//        if (lim)  {
//
//            LLResult result = limelight.getLatestResult();
//            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
//            for (LLResultTypes.FiducialResult fiducial : fiducials) {
//                id = fiducial.getFiducialId(); // The ID number of the fiducial
//            }
//
//            limeValid = result.isValid();
//
//            if (limeValid && ((id == target_id)) ) {
//
//                //telemetry.addLine("detected");
//                //   flag[autoaim]=true;
//                // flag[limedetect]=true;
//                Tx = result.getTx();
//                Ty = result.getTy();
//                if (red && Ty < 10 && Ty> -10.5) Tx_offset = -1.5;
//
//                if (!red && Ty < 10 && Ty> -10.5) Tx_offset = 1.5;
//
//                else if (Ty < -10.5 && red) Tx_offset = -3;
//
//                else if (Ty < -10.5 && !red) Tx_offset = 2.5;
//                flag[detected]=true;
//
//                //turettarget=turretPos;
//                pid_power = turretPID.calculate(Tx, Tx_offset);
//                if(turretPos>turretcounterclock-10&& pid_power>0)  {pid_power=-0.2;turettarget=0;flag[detected]=false;}
//                if(turretPos<(turrretclock+10)&& pid_power<0)  {pid_power=0.2;turettarget=0;flag[detected]=false;}
//                turretSpin.setPower(pid_power);
//
//                return;
//
//
//            }
//            else if (flag[detected]) {
//                turretSpin.setPower(0);
//                Tx=10;
//                return;
//            }
//
//
//
//        }
//        // non limalinment, contrl by encoder/angle
//
//        //      if(Math.abs(turretPos-target)<12) {pid_power=0; if (target!=0) turettarget=0;}// if reach the limit, turn to the middle
//
//        double turnpower = turretPID.calculate( turretPos*0.35,  target*0.35);
//
//        pid_power = Range.clip(turnpower, -MAX_TURN_POWER, MAX_TURN_POWER);
//
//        if(turretPos>turretcounterclock-10&& pid_power>0)  {pid_power=-0.2;turettarget=0;}
//        if(turretPos<(turrretclock+10)&& pid_power<0)  {pid_power=0.2;turettarget=0;}
//
//        turretSpin.setPower(pid_power);
//
//
//    }
//
//}

