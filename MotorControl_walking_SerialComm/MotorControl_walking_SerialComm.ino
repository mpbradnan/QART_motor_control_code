

/* Serial device defines for dxl bus */
#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define DXL_BUS_SERIAL2 2  //Dynamixel on Serial2(USART2)  <-LN101,BT210
#define DXL_BUS_SERIAL3 3  //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP

#define SerialBaud 57600

/* Dynamixel ID defines */
#define ID_1 1                    //reference variable for motor ID. 
#define ID_2 2
#define ID_3 3
#define ID_4 4
#define ID_5 5
#define ID_6 6
#define ID_7 7
#define ID_8 8
#define ID_9 9
#define ID_10 10
#define ID_11 11
#define ID_12 12

//define upper limit in degrees by motor
#define ULimit1 180
#define ULimit2 90
#define ULimit3 100
#define ULimit4 180
#define ULimit5 90
#define ULimit6 100
#define ULimit7 180
#define ULimit8 90
#define ULimit9 100
#define ULimit10 180
#define ULimit11 90
#define ULimit12 100

//definte lower limit in degrees by motor
#define LLimit1 -180
#define LLimit2 -10
#define LLimit3 -100
#define LLimit4 -180
#define LLimit5 -10
#define LLimit6 -100
#define LLimit7 -180
#define LLimit8 -10
#define LLimit9 -100
#define LLimit10 -180
#define LLimit11 -10
#define LLimit12 -100

/* Control table defines */        //variable names for internal motor variable addresses
#define GOAL_POSITION 116
#define GOAL_VELOCITY 112
#define TORQUE_ENABLE 64
#define MOVING 122
#define CP 132              //current position byte

Dynamixel Dxl(DXL_BUS_SERIAL3); 

int Command; //Control variable for storing command recieved via serial port
int Command_motion; //storage variable for current movement command
int Command_reps;   //storage variable for number of repetitions

float pa1=0;  //create additional float for each added motor - variable holds preconverted goal position value (stored in degrees). Allows us to track motor positions without having to request them
float pa2=0;
float pa3=0;
float pa4=0;
float pa5=0;
float pa6=0;
float pa7=0;
float pa8=0;
float pa9=0;
float pa10=0;
float pa11=0;
float pa12=0;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//position coordinate conversion functions
/* int d (float input){ //maps 360 point (degrees) to 4095 point radial map
   int output; 
   if(input>90){input=90;}          
   if(input<-90){input=-90;}
   output = (input+180)/360*4095; 
   return (output); 
}
*/

int d (float input, int ID){ //maps 360 point (degrees) to 4095 point radial map  //possibly giving error because of float/int ambiguity, but not since it's fine with limits below 90, not sure
   float Ulimit = 180;
   float Llimit = -180;
   int output;
   
   switch(ID)
   {
     case 1: Ulimit = ULimit1;
             Llimit = LLimit1;
             break;
     case 2: Ulimit = ULimit2;
             Llimit = LLimit2;
             break;
     case 3: Ulimit = ULimit3;
             Llimit = LLimit3;
             break;
     case 4: Ulimit = ULimit4;
             Llimit = LLimit4;
             break;
     case 5: Ulimit = ULimit5;
             Llimit = LLimit5;
             break;
     case 6: Ulimit = ULimit6;
             Llimit = LLimit6;
             break;    
     case 7: Ulimit = ULimit7;
             Llimit = LLimit7;
             break;
     case 8: Ulimit = ULimit8;
             Llimit = LLimit8;
             break;
     case 9: Ulimit = ULimit9;
             Llimit = LLimit9;
             break;
     case 10: Ulimit = ULimit10;
             Llimit = LLimit10;
             break;
     case 11: Ulimit = ULimit11;
             Llimit = LLimit11;
             break;
     case 12: Ulimit = ULimit12;
             Llimit = LLimit12;
             break;         
   }
   
  
   if(input>Ulimit){input=Ulimit;}          
   if(input<Llimit){input=Llimit;} 
   output = (input+180)/360*4095;
   return output;
}
 
int p (float input)  //maps 4095 point radial map back to degrees
{
   int output; 
   output = (input/4095*360)-180; 
   return (output); 
} 

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Single Joint Control function 
void SJC(int ID,int GP,int T, int OPP)//ID num, Goal postion, Time(ms),(current position)
{  
     int speed; 
     int GP2 = d(GP,ID); 
     int PP = d(OPP,ID);      
      
     speed = ((GP2-PP)*1000/T)/15.79; //0.229rpm = 1.374deg/s => 1.374/0.087 => 15.79  //0.088 is conversion constant between degrees and the internal 4095 point position map
     if(speed<0){speed=-speed;}  
     if(speed==0){speed=1;} 
     if(speed>278){speed=278;} 
     
     Dxl.writeDword(ID,GOAL_VELOCITY,  speed);//speed);
     Dxl.writeDword(ID,GOAL_POSITION,GP2);
 
 //update pa_#    
   switch (ID)                        
 {
   case 1: pa1 = GP; break;
   case 2: pa2 = GP; break;
   case 3: pa3 = GP; break;
   case 4: pa4 = GP; break;
   case 5: pa5 = GP; break;
   case 6: pa6 = GP; break;
   case 7: pa7 = GP; break;
   case 8: pa8 = GP; break;
   case 9: pa9 = GP; break;
   case 10: pa10 = GP; break;
   case 11: pa11 = GP; break;
   case 12: pa12 = GP; break;
 }
     
      

         
     //delay(T*1.5);
   

 } 
 /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 //double joint control
 void DJC(int ID1, int ID2, int GP1, int GP2, int T1, int T2, int OPP1, int OPP2)
 {
  //   int T=T1;          //greater of 2 times used for delay
  //  if(T<T2) {T=T2;}
     
     int speed1; 
     int speed2; 
     int GPC1 = d(GP1,ID1); //converted value
     int GPC2 = d(GP2,ID2); 
     int PP1 = d(OPP1,ID1);
     int PP2 = d(OPP2,ID2);     
      
     speed1 = ((GPC1-PP1)*1000/T1)/15.79; //0.229rpm = 1.374deg/s => 1.374/0.087 => 15.79  //0.088 is conversion constant between degrees and the internal 4095 point position map
     speed2 = ((GPC2-PP2)*1000/T2)/15.79;
     if(speed1<0){speed1=-speed1;}  
     if(speed1==0){speed1=1;} 
     if(speed1>278){speed1=278;} 
     
     if(speed2<0){speed2=-speed2;}  
     if(speed2==0){speed2=1;} 
     if(speed2>278){speed2=278;} 
     
     Dxl.writeDword(ID1,GOAL_VELOCITY,  speed1);//speed);
     Dxl.writeDword(ID1,GOAL_POSITION,GPC1);
     Dxl.writeDword(ID2,GOAL_VELOCITY,  speed2);//speed);
     Dxl.writeDword(ID2,GOAL_POSITION,GPC2);
 
 //update pa_ variables
 switch (ID1)                        //ID1
 {
   case 1: pa1 = GP1; break;
   case 2: pa2 = GP1; break;
   case 3: pa3 = GP1; break;
   case 4: pa4 = GP1; break;
   case 5: pa5 = GP1; break;
   case 6: pa6 = GP1; break;
   case 7: pa7 = GP1; break;
   case 8: pa8 = GP1; break;
   case 9: pa9 = GP1; break;
   case 10: pa10 = GP1; break;
   case 11: pa11 = GP1; break;
   case 12: pa12 = GP1; break;
 }
 
   switch (ID2)                      //ID2
 {
   case 1: pa1 = GP2; break;
   case 2: pa2 = GP2; break;
   case 3: pa3 = GP2; break;
   case 4: pa4 = GP2; break;
   case 5: pa5 = GP2; break;
   case 6: pa6 = GP2; break;
   case 7: pa7 = GP2; break;
   case 8: pa8 = GP2; break;
   case 9: pa9 = GP2; break;
   case 10: pa10 = GP2; break;
   case 11: pa11 = GP2; break;
   case 12: pa12 = GP2; break;
 }

   
     
   //  delay(T*1.5);
     

 }
 
 /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 //triple joint control function
 void TJC(int ID1, int ID2, int ID3, int GP1, int GP2, int GP3, int T1, int T2, int T3, int OPP1, int OPP2, int OPP3)    //Needs testing
 {
   //  int T=T1;          //greater of 3 times used for delay
   //  if(T<T2) {T=T2;}
   //  if(T<T3) {T=T3;}
     
     int speed1; 
     int speed2; 
     int speed3;
     int GPC1 = d(GP1,ID1); //converted value
     int GPC2 = d(GP2,ID2); 
     int GPC3 = d(GP3,ID3); 
     int PP1 = d(OPP1,ID1);
     int PP2 = d(OPP2,ID2);  
     int PP3 = d(OPP3,ID3);   
      
     speed1 = ((GPC1-PP1)*1000/T1)/15.79; //0.229rpm = 1.374deg/s => 1.374/0.087 => 15.79  //0.088 is conversion constant between degrees and the internal 4095 point position map
     speed2 = ((GPC2-PP2)*1000/T2)/15.79;
     speed3 = ((GPC3-PP3)*1000/T3)/15.79;
     
     if(speed1<0){speed1=-speed1;}  
     if(speed1==0){speed1=1;} 
     if(speed1>278){speed1=278;} 
     
     if(speed2<0){speed2=-speed2;}  
     if(speed2==0){speed2=1;} 
     if(speed2>278){speed2=278;} 
     
     if(speed3<0){speed3=-speed3;}  
     if(speed3==0){speed3=1;} 
     if(speed3>278){speed3=278;} 
     
//send commands to motors
     Dxl.writeDword(ID1,GOAL_VELOCITY,  speed1);//speed);
     Dxl.writeDword(ID1,GOAL_POSITION,GPC1);
     Dxl.writeDword(ID2,GOAL_VELOCITY,  speed2);//speed);
     Dxl.writeDword(ID2,GOAL_POSITION,GPC2);
     Dxl.writeDword(ID3,GOAL_VELOCITY,  speed3);//speed);
     Dxl.writeDword(ID3,GOAL_POSITION,GPC3);
 
 
 //update pa_# variables
 
 switch (ID1)
 {
   case 1: pa1 = GP1; break;
   case 2: pa2 = GP1; break;
   case 3: pa3 = GP1; break;
   case 4: pa4 = GP1; break;
   case 5: pa5 = GP1; break;
   case 6: pa6 = GP1; break;
   case 7: pa7 = GP1; break;
   case 8: pa8 = GP1; break;
   case 9: pa9 = GP1; break;
   case 10: pa10 = GP1; break;
   case 11: pa11 = GP1; break;
   case 12: pa12 = GP1; break;
 }
  switch (ID2)
 {
   case 1: pa1 = GP2; break;
   case 2: pa2 = GP2; break;
   case 3: pa3 = GP2; break;
   case 4: pa4 = GP2; break;
   case 5: pa5 = GP2; break;
   case 6: pa6 = GP2; break;
   case 7: pa7 = GP2; break;
   case 8: pa8 = GP2; break;
   case 9: pa9 = GP2; break;
   case 10: pa10 = GP2; break;
   case 11: pa11 = GP2; break;
   case 12: pa12 = GP2; break;
 }
   switch (ID3)
 {
   case 1: pa1 = GP3; break;
   case 2: pa2 = GP3; break;
   case 3: pa3 = GP3; break;
   case 4: pa4 = GP3; break;
   case 5: pa5 = GP3; break;
   case 6: pa6 = GP3; break;
   case 7: pa7 = GP3; break;
   case 8: pa8 = GP3; break;
   case 9: pa9 = GP3; break;
   case 10: pa10 = GP3; break;
   case 11: pa11 = GP3; break;
   case 12: pa12 = GP3; break;
 }
 
  
   // delay(T*1.5);
    
    
 }
  //simple interface functions that pass IDs and PA variables to motor control function, and negates angles as necessary. Helps keep number of variables under control.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //front right leg
 void FRLeg(int GP1, int GP2, int GP3, int T1, int T2, int T3) //(Shoulder joint 1 position, shoulder joint 2 position, elbow joint position, time1, time2, time3)
 {
    TJC(ID_1, ID_2, ID_3, GP1, GP2, GP3, T1, T2, T3, pa1, pa2, pa3);  //passes supplied variables as well as the Ids and pa_ variables for motors in front right leg
 }
 
 //front left leg
  void FLLeg(int GP1, int GP2, int GP3, int T1, int T2, int T3) //(Shoulder joint 1 position, shoulder joint 2 position, elbow joint position, time1, time2, time3)
 {
    TJC(ID_4, ID_5, ID_6, -GP1, -GP2, -GP3, T1, T2, T3, pa4, pa5, pa6);  //passes supplied variables as well as the Ids and pa_ variables for motors in front right leg
 }
 
 //rear right leg
  void RRLeg(int GP1, int GP2, int GP3, int T1, int T2, int T3) //(Shoulder joint 1 position, shoulder joint 2 position, elbow joint position, time1, time2, time3)
 {
    TJC(ID_7, ID_8, ID_9, GP1, -GP2, GP3, T1, T2, T3, pa7, pa8, pa9);  //passes supplied variables as well as the Ids and pa_ variables for motors in front right leg
 }
 
 //rear left leg
  void RLLeg(int GP1, int GP2, int GP3, int T1, int T2, int T3) //(Shoulder joint 1 position, shoulder joint 2 position, elbow joint position, time1, time2, time3)
 {
    TJC(ID_10, ID_11, ID_12, -GP1, GP2, -GP3, T1, T2, T3, pa10, pa11, pa12);  //passes supplied variables as well as the Ids and pa_ variables for motors in front right leg
 }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 //check motors for moving status  
  boolean FRLegMoving()  //check all motors in FR leg for moving (returns 1 if any motors are moving)
 {
   boolean moving;
   
   if (Dxl.readByte(ID_1,MOVING) || Dxl.readByte(ID_2,MOVING)){
   moving = 1;}
   else moving = 0;
   
   if (Dxl.readByte(ID_3,MOVING) || moving){
   return 1;}
   else return 0;
   
 
 }


 
  boolean FLLegMoving(){ //check all motors in FL leg for moving

   boolean moving;
   
   if (Dxl.readByte(ID_4,MOVING) || Dxl.readByte(ID_5,MOVING)){
   moving = 1;}
   else moving = 0;
   
   if (Dxl.readByte(ID_6,MOVING) || moving){
   return 1;}
   else return 0;
  }
 
  boolean RRLegMoving()  //check all motors in RR leg for moving
 {

   boolean moving;
   
   if (Dxl.readByte(ID_7,MOVING) || Dxl.readByte(ID_8,MOVING)){
   moving = 1;}
   else moving = 0;
   
   if (Dxl.readByte(ID_9,MOVING) || moving){
   return 1;}
   else return 0;
 }
 
  boolean RLLegMoving()  //check all motors in RL leg for moving
 {
 
   boolean moving;
   
   if (Dxl.readByte(ID_10,MOVING) || Dxl.readByte(ID_11,MOVING)){
   moving = 1;}
   else moving = 0;
   
   if (Dxl.readByte(ID_12,MOVING) || moving){
   return 1;}
   else return 0;
 }
 //////////////////movements////////////////////////////////////////////////////////////////////////////////////////////////
 
 void walkInPlace(int num)
 {
  //tuning variables tuned for 10.75" link length, +-5" steps from neutral, 3" step height(14"), standing height =19"
  float t1 = 500; float t1_delay = t1+(t1*0.4);
  float a1_neutral = -27.9055 ; float a2_neutral = 0; float a3_neutral = 55.8109; //angles for neutral standing position
  float a1_lift = -49.3707 ; float a2_lift = 0; float a3_lift = 98.7413; //angles for lifted neutral position
  
   if(!FRLegMoving()){
   FRLeg(a1_lift, a2_lift, a3_lift, t1, t1, t1);}
   if(!RLLegMoving()){
   RLLeg(a1_lift, a2_lift, a3_lift, t1, t1, t1);} 
   delay(t1_delay);
   
   if(!FRLegMoving()){
   FRLeg(a1_neutral, a2_neutral, a3_neutral, t1, t1, t1);} 
   if(!RLLegMoving()){
   RLLeg(a1_neutral, a2_neutral, a3_neutral, t1, t1, t1);} 
   delay(t1_delay);
   
   if(!FLLegMoving()){
   FLLeg(a1_lift, a2_lift, a3_lift, t1, t1, t1);} 
   if(!RRLegMoving()){
   RRLeg(a1_lift, a2_lift, a3_lift, t1, t1, t1);} 
   delay(t1_delay);
   
   if(!FLLegMoving()){
   FLLeg(a1_neutral, a2_neutral, a3_neutral, t1, t1, t1);} 
   if(!RRLegMoving()){
   RRLeg(a1_neutral, a2_neutral, a3_neutral, t1, t1, t1);} 
   delay(t1_delay);

   
   for(int i = 1; i < num; i++)
   {
     if(!FRLegMoving()){
     FRLeg(a1_lift, a2_lift, a3_lift, t1, t1, t1);}
     if(!RLLegMoving()){
     RLLeg(a1_lift, a2_lift, a3_lift, t1, t1, t1);} 
     delay(t1_delay);
   
     if(!FRLegMoving()){
     FRLeg(a1_neutral, a2_neutral, a3_neutral, t1, t1, t1);}  
     if(!RLLegMoving()){
     RLLeg(a1_neutral, a2_neutral, a3_neutral, t1, t1, t1);} 
     delay(t1_delay);
   
     if(!FLLegMoving()){
     FLLeg(a1_lift, a2_lift, a3_lift, t1, t1, t1);} 
     if(!RRLegMoving()){
     RRLeg(a1_lift, a2_lift, a3_lift, t1, t1, t1);} 
     delay(t1_delay);
   
     if(!FLLegMoving()){
     FLLeg(a1_neutral, a2_neutral, a3_neutral, t1, t1, t1);} 
     if(!RRLegMoving()){
     RRLeg(a1_neutral, a2_neutral, a3_neutral, t1, t1, t1);} 
     delay(t1_delay);  
   }
   
 }
 
 void FStride2(int num) //syncronized gait, input argument is number of step repetitions
 {
   //tuning variables tuned for 10.75" link length
   float t1 = 500; float t2 = 2*t1; float t0 = 0.5*t1; float t1_delay = t1+(t1*0.4); //t2 should be twice t1
   float a1_neutral = -43.6515; float a2_neutral = 0; float a3_neutral =87.3031; //angles for neutral standing position
   float a1_liftHalf = -37.7571; float a2_liftHalf = 0; float a3_liftHalf = 95.4154; //angles for first half of step motion from neutral
   float a1_lift = -48.4869; float a2_lift = 0; float a3_lift = 96.9739; //angles for first half of step motion from rearmost position
   float a1_drop = -22.7172; float a2_drop = 0; float a3_drop = 81.0705; //angles for second half of step motion from both possible prior positions
   float a1_liftEnd = -57.6583; float a2_liftEnd = 0; float a3_liftEnd = 95.4154; //angles for first half of step motion returning to neutral
   float a1_rear = -58.3533 ; float a2_rear = 0; float a3_rear = 81.0705; //angles for grounded leg at rearmost position
   float a1_midf = -33.7459; float a2_midf = 0; float a3_midf = 85.7512; //angles for grounded legs at forward half step
   float a1_midr = -52.0054 ; float a2_midr = 0; float a3_midr = 85.7512; //angles for grounded legs at negative half step (end effector behind neutral position)
   float a1_midfEnd = a1_midf; float a2_midfEnd = a2_midf; float a3_midfEnd = a3_midf; //angles for grounded legs at forward half step
   float a1_midrStart = a1_midr; float a2_midrStart = a2_midr; float a3_midrStart = a3_midr; //angles for grounded legs at negative half step (end effector behind neutral position)
   
   //forward step
   if(!FRLegMoving()){
   FRLeg(a1_liftHalf, a2_liftHalf, a3_liftHalf, t1, t1, t1);} //frleg lift(y) and move forward half of x distance   
   if(!RLLegMoving()){
   RLLeg(a1_liftHalf, a2_liftHalf, a3_liftHalf, t1, t1, t1);} //rlleg lift(y) and move forward half of x distance 
   if(!FLLegMoving()){
   FLLeg(a1_midrStart, a2_midrStart, a3_midrStart, t1, t1, t1);} //flleg and rrleg move halfway to midr at the same time as above commands
   if(!RRLegMoving()){
   RRLeg(a1_midrStart, a2_midrStart, a3_midrStart, t1, t1, t1);} //flleg and rrleg move halfway to midr at the same time as above commands
   delay(t1_delay);
   
   if(!FRLegMoving()){
   FRLeg(a1_drop, a2_drop, a3_drop, t1, t1, t1);} //frleg lower(y) and move forward remaining x distance
   if(!RLLegMoving()){
   RLLeg(a1_drop, a2_drop, a3_drop, t1, t1, t1);} //rlleg lower(y) and move forward remaining x distance
   if(!FLLegMoving()){
   FLLeg(a1_rear, a2_rear, a3_rear, t1, t1, t1);} //flleg and rrleg move to rearmost position at the same time as above commands
   if(!RRLegMoving()){
   RRLeg(a1_rear, a2_rear, a3_rear, t1, t1, t1);} //flleg and rrleg move to rearmost position at the same time as above commands
   delay(t1_delay);
   
   for(int i = 1; i < num; i++) //will only execute if more than 1 movement is commanded
   {
     if(!FLLegMoving()){
     FLLeg(a1_lift, a2_lift, a3_lift, t2, t2, t1);} //flleg lift(y) and move forward to raised neutral position
     if(!RRLegMoving()){
     RRLeg(a1_lift, a2_lift, a3_lift, t2, t2, t1);} //rrleg lift(y) and move forward to raised neutral position
     if(!FRLegMoving()){
     FRLeg(a1_midf, a2_midf, a3_midf, t1, t1, t1);} //frleg and rrleg move to midf
     if(!RLLegMoving()){
     RLLeg(a1_midf, a2_midf, a3_midf, t1, t1, t1);} //frleg and rrleg move to midf
     delay(t1_delay);
 
     if(!FRLegMoving()){
     FRLeg(a1_neutral, a2_neutral, a3_neutral, t1, t1, t1);} //frleg and rrleg move to neutral
     if(!RLLegMoving()){
     RLLeg(a1_neutral, a2_neutral, a3_neutral, t1, t1, t1);} //frleg and rrleg move to neutral
     delay(t1_delay);
     
     if(!FLLegMoving()){
     FLLeg(a1_drop, a2_drop, a3_drop, t2, t2, t2);} //flleg lower(y) and move forward remaining x distance   
     if(!RRLegMoving()){
     RRLeg(a1_drop, a2_drop, a3_drop, t2, t2, t2);} //rrleg lower(y) and move forward remaining x distance   
     if(!FRLegMoving()){
     FRLeg(a1_midr, a2_midr, a3_midr, t1, t1, t1);} //frleg and rlleg move to midr
     if(!RLLegMoving()){
     RLLeg(a1_midr, a2_midr, a3_midr, t1, t1, t1);} //frleg and rlleg move to midr  
     delay(t1_delay);
     
     if(!FRLegMoving()){
     FRLeg(a1_rear, a2_rear, a3_rear, t1, t1, t1);} //frleg and rlleg move to rearmost position
     if(!RLLegMoving()){
     RLLeg(a1_rear, a2_rear, a3_rear, t1, t1, t1);} //frleg and rlleg move to rearmost position    
     delay(t1_delay);
     
     if(!FRLegMoving()){
     FRLeg(a1_lift, a2_lift, a3_lift, t2, t2, t1);} //frleg lift(y) and move forward to raised neutral position
     if(!RLLegMoving()){
     RLLeg(a1_lift, a2_lift, a3_lift, t2, t2, t1);} //rlleg lift(y) and move forward to raised neutral position 
     if(!FLLegMoving()){
     FLLeg(a1_midf, a2_midf, a3_midf, t1, t1, t1);} //flleg and rrleg move to midf
     if(!RRLegMoving()){
     RRLeg(a1_midf, a2_midf, a3_midf, t1, t1, t1);} //flleg and rrleg move to midf
     delay(t1_delay);

     if(!FLLegMoving()){
     FLLeg(a1_neutral, a2_neutral, a3_neutral, t1, t1, t1);} //flleg and rrleg move to neutral
     if(!RRLegMoving()){
     RRLeg(a1_neutral, a2_neutral, a3_neutral, t1, t1, t1);} //flleg and rrleg move to neutral
     delay(t1_delay);
   
     if(!FRLegMoving()){
     FRLeg(a1_drop, a2_drop, a3_drop, t2, t2, t2);} //frleg drop(y) and move forward remaining x distance   
     if(!RLLegMoving()){
     RLLeg(a1_drop, a2_drop, a3_drop, t2, t2, t2);} //rlleg drop(y) and move forward remaining x distance   
     if(!FLLegMoving()){
     FLLeg(a1_midr, a2_midr, a3_midr, t1, t1, t1);} //flleg and rrleg move to midr
     if(!RRLegMoving()){
     RRLeg(a1_midr, a2_midr, a3_midr, t1, t1, t1);} //flleg and rrleg move to midr
     delay(t1_delay); 
     
     if(!FLLegMoving()){
     FLLeg(a1_rear, a2_rear, a3_rear, t1, t1, t1);} //flleg and rrleg move to rearmost position
     if(!RRLegMoving()){
     RRLeg(a1_rear, a2_rear, a3_rear, t1, t1, t1);} //flleg and rrleg move to rearmost position
     delay(t1_delay); 
   }
   
     //return to neutral
     if(!FLLegMoving()){
     FLLeg(a1_liftEnd, a2_liftEnd, a3_liftEnd, t1, t1, t1);} // flleg lift(y) and move forward to half step
     if(!RRLegMoving()){
     RRLeg(a1_liftEnd, a2_liftEnd, a3_liftEnd, t1, t1, t1);} // rrleg lift(y) and move forward to half step
     if(!FRLegMoving()){
     FRLeg(a1_midfEnd, a2_midfEnd, a3_midfEnd, t1, t1, t1);} //frleg move 
     if(!RLLegMoving()){
     RLLeg(a1_midfEnd, a2_midfEnd, a3_midfEnd, t1, t1, t1);} //rlleg move  
     delay(t1_delay); 
     
     if(!FLLegMoving()){
     FLLeg(a1_neutral, a2_neutral, a3_neutral, t1, t1, t1);} // flleg lift(y) and move to neutral
     if(!RRLegMoving()){
     RRLeg(a1_neutral, a2_neutral, a3_neutral, t1, t1, t1);} // rrleg lift(y) and move to neutral 
     if(!FRLegMoving()){
     FRLeg(a1_neutral, a2_neutral, a3_neutral, t1, t1, t1);} //frleg move back to neutral  
     if(!RLLegMoving()){
     RLLeg(a1_neutral, a2_neutral, a3_neutral, t1, t1, t1);} //rlleg move back to neutral   
     delay(t1_delay); 
   
   
   //pseudocode
   /*
   
   frleg lift(y) and move forward half of x distance
   rlleg lift(y) and move forward half of x distance
   frleg lower(y) and move forward remaining x distance
   rlleg lower(y) and move forward remaining x distance
   flleg and rrleg move back full x distance at the same time as above commands
   



   for num repetitions:
   {
   flleg lift(y) and move forward half of x distance (should be more than x distance of initial step as position is no longer neutral)
   rrleg lift(y) and move forward half of x distance
   flleg lower(y) and move forward remaining x distance
   rrleg lower(y) and move forward remaining x distance
   frleg and rlleg move backto nuetral
   
   frleg lift(y) and move forward half of x distance  (should be more than x distance of initial step as position is no longer neutral)
   rlleg lift(y) and move forward half of x distance
   frleg lower(y) and move forward remaining x distance
   rlleg lower(y) and move forward remaining x distance
   flleg and rrleg move back full x distance at the same time as above commands
   }
   
   flleg lift(y) and move forward half of x distance (returning to neutral)
   rrleg lift(y) and move forward half of x distance
   flleg lower(y) and move forward remaining x distance
   rrleg lower(y) and move forward remaining x distance
   frleg and rlleg move back full x distance at the same time as above commands
   */
   
 }
 //----------------------------------------------------------------------------------------
 void FStep(int num)
 {
   //standing height: 20in; lift height: 15in; step size=5in;
 float Noffset = 5; //offset to correct for kinematic model error (to center neutral position)  
 float t1 = 400; float t2 = 2*t1; float t0 = 0.5*t1; float t1_delay = t1+(t1*0.5); float t2_delay = t2+(t2*0.3); //t2 should be twice t1
 float a1_neutral = -21.5289-Noffset; float a2_neutral = 5; float a3_neutral =43.0578; //angles for neutral standing position
 float a1_lift = -26.5489-Noffset; float a2_lift = 5; float a3_lift = 86.4962; 
 float a1_drop = -2.4553-Noffset; float a2_drop = 5; float a3_drop = 32.9830;
 float a1_shift_front = -26.5031-Noffset; float a2_shift_front = 5; float a3_shift_front = 41.5850;
 float a1_shift_rear = -29.0375-Noffset; float a2_shift_rear = 5; float a3_shift_rear = 19.4948;
 float a1_lift_end = -54.4468-Noffset; float a2_lift_end = 5; float a3_lift_end = 89.9690;
 float SHoffset = 5; //sideways offset for shoulders when lifting leg
 float SHoffsetR = 5; //front to back offset for shoulders when lifting front legs
 float SHoffsetR2 = 5; //front to back offset for shoulders when lifting rear legs
 float SHoffsetL = 5; //offset for shoulder of lifted leg

  
  
  //startup 
  for(int i=1; i<=num; i++)
  {
    //right leg step
   if(!FRLegMoving()){
   FRLeg(a1_lift-SHoffsetL, a2_lift-SHoffsetL, a3_lift, t1, t1, t1);} // 
   if(!FLLegMoving()){
   FLLeg(a1_neutral+SHoffsetR, a2_neutral-SHoffset, a3_neutral, t1, t1, t1);} // 
   if(!RRLegMoving()){
   RRLeg(a1_neutral+SHoffsetR, a2_neutral+SHoffset, a3_neutral, t1, t1, t1);} // 
   if(!RLLegMoving()){
   RLLeg(a1_neutral+SHoffsetR, a2_neutral-SHoffset, a3_neutral, t1, t1, t1);} // 
   delay(t1_delay);
   
   if(!FRLegMoving()){
   FRLeg(a1_drop, a2_drop, a3_drop, t1, t1, t1);} //
   if(!FLLegMoving()){
   FLLeg(a1_neutral, a2_neutral, a3_neutral, t1, t1, t1);} // 
   if(!RRLegMoving()){
   RRLeg(a1_neutral, a2_neutral, a3_neutral, t1, t1, t1);} // 
   if(!RLLegMoving()){
   RLLeg(a1_neutral, a2_neutral, a3_neutral, t1, t1, t1);} //  
   delay(t1_delay);
   
   //left leg step
   if(!FLLegMoving()){
   FLLeg(a1_lift-SHoffsetL, a2_lift-SHoffsetL, a3_lift, t1, t1, t1);} //
   if(!FRLegMoving()){
   FRLeg(a1_drop+SHoffsetR, a2_drop-SHoffset, a3_drop, t1, t1, t1);} // 
   if(!RRLegMoving()){
   RRLeg(a1_neutral+SHoffsetR, a2_neutral-SHoffset, a3_neutral, t1, t1, t1);} // 
   if(!RLLegMoving()){
   RLLeg(a1_neutral+SHoffsetR, a2_neutral+SHoffset, a3_neutral, t1, t1, t1);} // 
   delay(t1_delay);
      
   if(!FLLegMoving()){
   FLLeg(a1_drop, a2_drop, a3_drop, t1, t1, t1);} //
   if(!FRLegMoving()){
   FRLeg(a1_drop, a2_drop, a3_drop, t1, t1, t1);} // 
   if(!RRLegMoving()){
   RRLeg(a1_neutral, a2_neutral, a3_neutral, t1, t1, t1);} // 
   if(!RLLegMoving()){
   RLLeg(a1_neutral, a2_neutral, a3_neutral, t1, t1, t1);} //  
   delay(t1_delay);
   
   //shift forward
   if(!FRLegMoving()){
   FRLeg(a1_shift_front, a2_shift_front, a3_shift_front, t2, t2, t2);} // 
   if(!FLLegMoving()){
   FLLeg(a1_shift_front, a2_shift_front, a3_shift_front, t2, t2, t2);} //
   if(!RLLegMoving()){
   RLLeg(a1_shift_rear, a2_shift_rear, a3_shift_rear, t2, t2, t2);} // 
   if(!RRLegMoving()){
   RRLeg(a1_shift_rear, a2_shift_rear, a3_shift_rear, t2, t2, t2);} //
   delay(t2_delay);
   
   
   //right step (rear)
   if(!RRLegMoving()){
   RRLeg(a1_lift_end+SHoffsetL, a2_lift_end-SHoffsetL, a3_lift_end, t1, t1, t1);} // 
   if(!FLLegMoving()){
   FLLeg(a1_shift_front-SHoffsetR2, a2_shift_front-SHoffset, a3_shift_front, t1, t1, t1);} // 
   if(!FRLegMoving()){
   FRLeg(a1_shift_front-SHoffsetR2, a2_shift_front+SHoffset, a3_shift_front, t1, t1, t1);} // 
   if(!RLLegMoving()){
   RLLeg(a1_shift_rear-SHoffsetR2, a2_shift_rear-SHoffset, a3_shift_rear, t1, t1, t1);} // 
   delay(t1_delay);
   
   
   if(!RRLegMoving()){
   RRLeg(a1_shift_front, a2_shift_front, a3_shift_front, t1, t1, t1);} // 
   if(!FLLegMoving()){
   FLLeg(a1_shift_front, a2_shift_front, a3_shift_front, t1, t1, t1);} // 
   if(!FRLegMoving()){
   FRLeg(a1_shift_front, a2_shift_front, a3_shift_front, t1, t1, t1);} // 
   if(!RLLegMoving()){
   RLLeg(a1_shift_rear, a2_shift_rear, a3_shift_rear, t1, t1, t1);} // 
   delay(t1_delay);
   
   //left step (rear)
   if(!RLLegMoving()){
   RLLeg(a1_lift_end+SHoffsetL, a2_lift_end-SHoffsetL, a3_lift_end, t1, t1, t1);} //
   if(!FLLegMoving()){
   FLLeg(a1_shift_front-SHoffsetR2, a2_shift_front+SHoffset, a3_shift_front, t1, t1, t1);} // 
   if(!FRLegMoving()){
   FRLeg(a1_shift_front-SHoffsetR2, a2_shift_front-SHoffset, a3_shift_front, t1, t1, t1);} // 
   if(!RRLegMoving()){
   RRLeg(a1_shift_front-SHoffsetR2, a2_shift_front-SHoffset, a3_shift_front, t1, t1, t1);} // 
   delay(t1_delay);
   
   if(!RLLegMoving()){
   RLLeg(a1_shift_front, a2_shift_front, a3_shift_front, t1, t1, t1);} //
   if(!FLLegMoving()){
   FLLeg(a1_shift_front, a2_shift_front, a3_shift_front, t1, t1, t1);} // 
   if(!RRLegMoving()){
   RRLeg(a1_shift_front, a2_shift_front, a3_shift_front, t1, t1, t1);} // 
   if(!FRLegMoving()){
   FRLeg(a1_shift_front, a2_shift_front, a3_shift_front, t1, t1, t1);} // 
   delay(t1_delay);
   
   //return to neutral
   if(!RLLegMoving()){
   RLLeg(a1_neutral, a2_neutral, a3_neutral, t1, t1, t1);} //
   if(!FLLegMoving()){
   FLLeg(a1_neutral, a2_neutral, a3_neutral, t1, t1, t1);} // 
   if(!RRLegMoving()){
   RRLeg(a1_neutral, a2_neutral, a3_neutral, t1, t1, t1);} // 
   if(!FRLegMoving()){
   FRLeg(a1_neutral, a2_neutral, a3_neutral, t1, t1, t1);} // 
   delay(t1_delay);
  }
   
 }
 //-----------------------------------------------------------------------------------------------------------
  void FStride(int num)
 {
   //standing height: 20in; lift height: 15in; step size=5in;
 float Noffset = 4; //offset to correct for kinematic model error (to center neutral position)  
 float t1 = 250; float t2 = 4*t1; float t0 = 0.5*t1; float t1_delay = t1+(t1*0.4); float t2_delay = t2+(t2*0.3); //t2 should be twice t1
 float a1_neutral = -21.5289-Noffset; float a2_neutral = 5; float a3_neutral =43.0578; //angles for neutral standing position
 float a1_lift = -26.5489-Noffset; float a2_lift = 5; float a3_lift = 86.4962; 
 float a1_drop = -2.4553-Noffset; float a2_drop = 5; float a3_drop = 32.9830;
 //float a1_shift_front = -26.5031-Noffset; float a2_shift_front = 5; float a3_shift_front = 41.5850; //shift for legs that are in forward position
 float a1_shift_front = a1_neutral; float a2_shift_front = a2_neutral; float a3_shift_front = a3_neutral; //shift for legs that are in forward position
 float a1_shift_rear = -29.0375-Noffset; float a2_shift_rear = 5; float a3_shift_rear = 19.4948;  //shift for legs still in rear (or neutral) position after other two legs step
 float a1_lift_end = -54.4468-Noffset; float a2_lift_end = 5; float a3_lift_end = 89.9690;  //lift for leg shifted back in prefious step


  
  
  //startup 
  for(int i=1; i<=num; i++)
  {
    
   //lift fr and rl legs
  // if(!FRLegMoving()){
   FRLeg(a1_lift, a2_lift, a3_lift, t1, t1, t1);//} // 
  // if(!RLLegMoving()){
   RLLeg(a1_lift, a2_lift, a3_lift, t1, t1, t1);//} //
   if(!FLLegMoving()){
   FLLeg(a1_neutral, a2_neutral, a3_neutral, t1, t1, t1);} // 
   if(!RRLegMoving()){
   RRLeg(a1_neutral, a2_neutral, a3_neutral, t1, t1, t1);} // 
   delay(t1_delay);
   
   //drop fr and rl legs
  // if(!FRLegMoving()){
   FRLeg(a1_drop, a2_drop, a3_drop, t1, t1, t1);//} // 
 //  if(!RLLegMoving()){
   RLLeg(a1_drop, a2_drop, a3_drop, t1, t1, t1);//} //
   if(!FLLegMoving()){
   FLLeg(a1_neutral, a2_neutral, a3_neutral, t1, t1, t1);} // 
   if(!RRLegMoving()){
   RRLeg(a1_neutral, a2_neutral, a3_neutral, t1, t1, t1);} // 
   delay(t1_delay);
   
   //shift forward
  // if(!FRLegMoving()){
   FRLeg(a1_shift_front, a2_shift_front, a3_shift_front, t2, t2, t2);//} // 
 //  if(!RLLegMoving()){
   RLLeg(a1_shift_front, a2_shift_front, a3_shift_front, t2, t2, t2);//} //
  // if(!FLLegMoving()){
   FLLeg(a1_shift_rear, a2_shift_rear, a3_shift_rear, t2, t2, t2);//} // 
  // if(!RRLegMoving()){
   RRLeg(a1_shift_rear, a2_shift_rear, a3_shift_rear, t2, t2, t2);//} // 
   delay(t2_delay);
   
   //lift fl and rr legs
   if(!FRLegMoving()){
   FRLeg(a1_shift_front, a2_shift_front, a3_shift_front, t1, t1, t1);} // 
   if(!RLLegMoving()){
   RLLeg(a1_shift_front, a2_shift_front, a3_shift_front, t1, t1, t1);} //
  // if(!FLLegMoving()){
   FLLeg(a1_lift_end, a2_lift_end, a3_lift_end, t1, t1, t1);//} // 
  // if(!RRLegMoving()){
   RRLeg(a1_lift_end, a2_lift_end, a3_lift_end, t1, t1, t1);//} // 
   delay(t1_delay);
   
   //drop fl and rr legs
   if(!FRLegMoving()){
   FRLeg(a1_shift_front, a2_shift_front, a3_shift_front, t1, t1, t1);} // 
   if(!RLLegMoving()){
   RLLeg(a1_shift_front, a2_shift_front, a3_shift_front, t1, t1, t1);} //
  // if(!FLLegMoving()){
   FLLeg(a1_shift_front, a2_shift_front, a3_shift_front, t1, t1, t1);//} // 
  // if(!RRLegMoving()){
   RRLeg(a1_shift_front, a2_shift_front, a3_shift_front, t1, t1, t1);//} // 
   delay(t1_delay);
   
         
   //return to neutral
   //if(!FRLegMoving()){
   FRLeg(a1_neutral, a2_neutral, a3_neutral, t1, t1, t1);//} // 
  // if(!RLLegMoving()){
   RLLeg(a1_neutral, a2_neutral, a3_neutral, t1, t1, t1);//} //
 //  if(!FLLegMoving()){
   FLLeg(a1_neutral, a2_neutral, a3_neutral, t1, t1, t1);//} // 
  // if(!RRLegMoving()){
   RRLeg(a1_neutral, a2_neutral, a3_neutral, t1, t1, t1);//} // 
   delay(t1_delay);
  }
   
 }
 
 //------------------------------------------------------------------------------------------------------------
 void FStepPose() // Function for placing pose code
 {
   //standing height: 20in; lift height: 15in; step size=5in;
 float Noffset = 5; //offset to correct for kinematic model error (to center neutral position)  
 float t1 = 400; float t2 = 2*t1; float t0 = 0.5*t1; float t1_delay = t1+(t1*0.5); float t2_delay = t2+(t2*0.3); //t2 should be twice t1
 float a1_neutral = -21.5289-Noffset; float a2_neutral = 5; float a3_neutral =43.0578; //angles for neutral standing position
 float a1_lift = -26.5489-Noffset; float a2_lift = 5; float a3_lift = 86.4962; 
 float a1_drop = -2.4553-Noffset; float a2_drop = 5; float a3_drop = 32.9830;
 float a1_shift_front = -26.5031-Noffset; float a2_shift_front = 5; float a3_shift_front = 41.5850;
 float a1_shift_rear = -29.0375-Noffset; float a2_shift_rear = 5; float a3_shift_rear = 19.4948;
 float a1_lift_end = -54.4468-Noffset; float a2_lift_end = 5; float a3_lift_end = 89.9690;
 float SHoffset = 5; //sideways offset for shoulders when lifting leg
 float SHoffsetR = 5; //front to back offset for shoulders when lifting front legs
 float SHoffsetR2 = 5; //front to back offset for shoulders when lifting rear legs
 float SHoffsetL = 5; //offset for shoulder of lifted leg

  //######################################3
  
    //right leg step
   if(!FRLegMoving()){
   FRLeg(a1_lift-SHoffsetL, a2_lift-SHoffsetL, a3_lift, t1, t1, t1);} // 
   if(!FLLegMoving()){
   FLLeg(a1_neutral+SHoffsetR, a2_neutral-SHoffset, a3_neutral, t1, t1, t1);} // 
   if(!RRLegMoving()){
   RRLeg(a1_neutral+SHoffsetR, a2_neutral+SHoffset, a3_neutral, t1, t1, t1);} // 
   if(!RLLegMoving()){
   RLLeg(a1_neutral+SHoffsetR, a2_neutral-SHoffset, a3_neutral, t1, t1, t1);} // 

  
  //#######################################3
  
 }
 //-----------------------------------------------------------------------------------------------------------
 void crouch(int reps)
 {
  
   for(int i=1; i<=reps; i++)
   {
     if(!FRLegMoving()){
     FRLeg(-35.5160 , 0, 71.0320, 2000, 2000, 2000);} //(shoulder1 pos, shoulder2 pos, knee pos, time1, time2, time3)

   
    if(!FLLegMoving()){
     FLLeg(-35.5160 , 0, 71.0320, 2000, 2000, 2000);} //(shoulder1 pos, shoulder2 pos, knee pos, time1, time2, time3)

   
    if(!RRLegMoving()){
     RRLeg(-35.5160 , 0, 71.0320, 2000, 2000, 2000);} //(shoulder1 pos, shoulder2 pos, knee pos, time1, time2, time3)

   
    if(!RLLegMoving()){
     RLLeg(-35.5160 , 0, 71.0320, 2000, 2000, 2000);} //(shoulder1 pos, shoulder2 pos, knee pos, time1, time2, time3)
     delay(2200);
   
   
      
    if(!FRLegMoving()){
     FRLeg(-49.3707, 0, 98.7413, 2000, 2000, 2000);}

   
    if(!FLLegMoving()){
     FLLeg(-49.3707, 0, 98.7413, 2000, 2000, 2000);}

   
    if(!RRLegMoving()){
     RRLeg(-49.3707, 0, 98.7413, 2000, 2000, 2000);}

   
    if(!RLLegMoving()){
     RLLeg(-49.3707, 0, 98.7413, 2000, 2000, 2000);}
     delay(2200);
   }
 }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 
 //Setup function
void setup() {
  
  // Dynamixel 2.0 Baudrate -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
  Dxl.begin(3); //set Baud
  
  //Start serial communication
  Serial2.begin(SerialBaud);
  SerialUSB.begin();
  
  //enable motor torque
  Dxl.writeByte(ID_1,TORQUE_ENABLE,1);
  Dxl.writeByte(ID_2,TORQUE_ENABLE,1);
  Dxl.writeByte(ID_3,TORQUE_ENABLE,1);
  Dxl.writeByte(ID_4,TORQUE_ENABLE,1);
  Dxl.writeByte(ID_5,TORQUE_ENABLE,1);
  Dxl.writeByte(ID_6,TORQUE_ENABLE,1);
  Dxl.writeByte(ID_7,TORQUE_ENABLE,1);
  Dxl.writeByte(ID_8,TORQUE_ENABLE,1);
  Dxl.writeByte(ID_9,TORQUE_ENABLE,1);
  Dxl.writeByte(ID_10,TORQUE_ENABLE,1);
  Dxl.writeByte(ID_11,TORQUE_ENABLE,1);
  Dxl.writeByte(ID_12,TORQUE_ENABLE,1);
  
  //Dxl.jointMode(ID_NUM); //jointMode() is to use position mode
  
  //initialize pa_ variables (current position)
  pa1=p(Dxl.readWord(ID_1,CP));  //pa1 stores unconverted position (degrees) so a conversion function is used to read it from motor.
  pa2=p(Dxl.readWord(ID_2,CP));
  pa3=p(Dxl.readWord(ID_3,CP));
  pa4=p(Dxl.readWord(ID_4,CP));
  pa5=p(Dxl.readWord(ID_5,CP));
  pa6=p(Dxl.readWord(ID_6,CP));
  pa7=p(Dxl.readWord(ID_7,CP));
  pa8=p(Dxl.readWord(ID_8,CP));
  pa9=p(Dxl.readWord(ID_9,CP));
  pa10=p(Dxl.readWord(ID_10,CP));
  pa11=p(Dxl.readWord(ID_11,CP));
  pa12=p(Dxl.readWord(ID_12,CP));
                                           
  //reset motor positions                                                               
float a1_setup = -21.5289; float a2_setup = 5; float a3_setup = 43.0578;
  float Noffset = 5;
  FRLeg(a1_setup-Noffset, a2_setup, a3_setup, 2000, 2000, 2000); //(shoulder1 pos, shoulder2 pos, knee pos, time1, time2, time3)
  FLLeg(a1_setup-Noffset, a2_setup, a3_setup, 2000, 2000, 2000); //(shoulder1 pos, shoulder2 pos, knee pos, time1, time2, time3)
  RRLeg(a1_setup-Noffset, a2_setup, a3_setup, 2000, 2000, 2000); //(shoulder1 pos, shoulder2 pos, knee pos, time1, time2, time3)
  RLLeg(a1_setup-Noffset, a2_setup, a3_setup, 2000, 2000, 2000); //(shoulder1 pos, shoulder2 pos, knee pos, time1, time2, time3)
  
  delay(3000); //delay no longer built in to control function
  
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//artifact?
byte isMoving = 0;    //unused?

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//main looping function
void loop() {  
  //Turn dynamixel ID 1 to position 0  
  //Dxl.writeDword(ID_NUM, GOAL_POSITION, 0); //Compatible with all dynamixel series
  // Wait for 1 second (1000 milliseconds)
  //delay(1000);         
  //Turn dynamixel ID 1 to position 300
  //Dxl.writeDword(ID_NUM, GOAL_POSITION, 300);
  // Wait for 1 second (1000 milliseconds)
  //delay(1000);

//if(!Dxl.readByte(ID_NUM,MOVING)){                                        //single joint control
 // SJC(ID_NUM,0,3000,pa1);}

//if(!Dxl.readByte(ID_1,MOVING) && !Dxl.readByte(ID_2,MOVING)){             //two joint control
 // DJC(ID_1,ID_2,90,-90,3000,3000,pa1,pa2);}
 
 //if(!Dxl.readByte(ID_1,MOVING) && !Dxl.readByte(ID_2,MOVING)){            //three joint control
 // TJC(ID_1,ID_2,ID_3, 45, 90,-90, 3000, 3000, 3000,  pa1, pa2, pa3);}
 
 //if(!FRLegMoving()){                                                      //FRLeg interface function //FLLeg //RRLeg //RLLeg
  // FRLeg(0, 0, 0, 3000, 3000, 3000);} //(shoulder1 pos, shoulder2 pos, knee pos, time1, time2, time3)
 
 //begin code
 
 /*
  //tell master computer abort is complete --------- need to use serial interrupt
if (Command_motion == 00)
{
  Serial2.print(11); //send sentinel value to master computer
  delay(500);
}
*/

if ( Serial2.available() )
{
  Command = Serial2.read();  //read serial port for command. Format is int xxyy, where xx is movement type and yy is repetitions. Example: 0102 performs walk in place twice
  Command_motion = Command/100; //drop last two digits to get mool;p=][vement code
  Command_reps = Command - Command_motion*100; //subtract movement code from Command to get repetitions
  switch(Command_motion) //call corresponding movement function
  {
     case 01: walkInPlace(Command_reps); break;
     case 02: FStride(Command_reps); break;
     default: break;
  }
}


/*
SerialUSB.println("testing!!!");
SerialUSB.print("Velocity I gain: ");
SerialUSB.println(Dxl.readWord(ID_1,76));
SerialUSB.print("Velocity P gain: ");
SerialUSB.println(Dxl.readWord(ID_1,78));
SerialUSB.print("Position D gain: ");
SerialUSB.println(Dxl.readWord(ID_1,80));
SerialUSB.print("Position I gain: ");
SerialUSB.println(Dxl.readWord(ID_1,82));
SerialUSB.print("Position P gain: ");
SerialUSB.println(Dxl.readWord(ID_1,84));
SerialUSB.print("Feedforward 1st gain: ");
SerialUSB.println(Dxl.readWord(ID_1,90));
SerialUSB.print("Feedforward 2nd gain: ");
SerialUSB.println(Dxl.readWord(ID_1,88));
*/

delay(5000);


//crouch(5);
//FStep(1);
FStride(5);
//walkInPlace(3);

//FStepPose();

delay(1000);

}

   



