#include <ESP32Servo.h>
#include <SPI.h>

#include <TFT_eSPI.h>       // Hardware-specific library

TFT_eSPI tft = TFT_eSPI();  // Invoke custom library

//MIN and MAX PWM pulse sizes, they can be found in servo documentation
#define MAX 2200
#define MIN 800

//Positions of servos mounted in opposite direction
#define INV1 1
#define INV2 3
#define INV3 5

//constants for computation of positions of connection points
#define pi  3.14159
#define deg2rad 180/pi
#define deg30 pi/6

//Array of servo objects
Servo servo[6];

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;

ESP32PWM pwm;

typedef struct
{
  int a;
  int b;
  int c;
  int d;
  int e; 
  int f;
} points_6;


static int zero[6]={1475,1470,1490,1480,1460,1490};

static float arr[6];      //In this array is stored requested position for platform - x,y,z,rot(x),rot(y),rot(z)
                                                                    
static float theta_a[6]={0.0,0.0,0.0, 0.0,0.0,0.0};                       //Actual degree of rotation of all servo arms, they start at 0 - horizontal, used to reduce
                                                                          //complexity of calculating new degree of rotation
static int servo_pos[6];                                                  //Array of current servo positions in us

const float beta[] = {pi/2,-pi/2,-pi/6, 5*pi/6,-5*pi/6,pi/6};             //rotation of servo arms in respect to axis x
const float servo_min=radians(-80);
const float servo_max=radians(80);
const float servo_mult=400/(pi/4);

const float L1 = 20,L2 = 113, z_home = 100;

//maximum servo positions, 0 is horizontal position
//servo_mult - multiplier used for conversion radians->servo pulse in us
//L1-effective length of servo arm, L2 - length of base and platform connecting arm
//z_home - height of platform above base, 0 is height of servo arms

//RD distance from center of platform to attachment points (arm attachment point)
//RD distance from center of base to center of servo rotation points (servo axis)
//theta_p-angle between two servo axis points, theta_r - between platform attachment points
//theta_angle-helper variable
//p[][]=x y values for servo rotation points
//re[]{}=x y z values of platform attachment points positions
//equations used for p and re will affect postion of X axis, they can be changed to achieve
//specific X axis position

static float p[2][6]={{-10,10,67.35,57.35,-57.35,-67.35},{72,72,-27.34,-44.66,-44.66,-27.34}};

static float re[3][6] = {{-24.25, 24.25, 49.62, 25.37, -25.37, -49.62},
                          {43.3 ,43.3 ,-0.648 ,-42.62, -42.65, -0.646},
                          {0 ,0 ,0 ,0 ,0 ,0}};

//arrays used for servo rotation calculation
//H[]-center position of platform can be moved with respect to base, this is
//translation vector representing this move

static float M[3][3], rxp[3][6], T[3], H[3] = {0,0,z_home};

void setup(){

  tft.init();
  tft.setRotation(2);

  tft.fillScreen(TFT_BLACK);
  tft.setCursor(40, 70, 4);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.println("LCD Initialised");

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  ESP32PWM::allocateTimer(4);
  ESP32PWM::allocateTimer(5);
   
//begin of serial communication
   Serial.begin(115200);

  servo1.setPeriodHertz(50);      // Standard 50hz servo
  servo2.setPeriodHertz(50);     
  servo3.setPeriodHertz(50);      
  servo4.setPeriodHertz(50);      
  servo5.setPeriodHertz(50);      
  servo6.setPeriodHertz(50);     

  //attachment of servos to PWM digital pins of arduino
   servo[0].attach(21, MIN, MAX);
   servo[1].attach(22, MIN, MAX);
   servo[2].attach(19, MIN, MAX);
   servo[3].attach(5, MIN, MAX);
   servo[4].attach(8, MIN, MAX);
   servo[5].attach(7, MIN, MAX);

//putting into base position
   setPos(arr);
   set_pos_zero();
}

void set_pos_zero(){ 

static int zero[6]={1475,1470,1490,1480,1460,1490};

  servo1.writeMicroseconds(zero[1]);
  servo2.writeMicroseconds(zero[2]);
  servo3.writeMicroseconds(zero[3]);
  servo4.writeMicroseconds(zero[4]);
  servo5.writeMicroseconds(zero[5]);
  servo6.writeMicroseconds(zero[6]); 
}

float getAlpha(int *i){
   static int n;
   static float th=0;
   static float q[3], dl[3], dl2;
   double min=servo_min;
   double max=servo_max;
   n=0;
   th=theta_a[*i];
   while(n<20){
    //calculation of position of base attachment point (point on servo arm where is leg connected)
      q[0] = L1*cos(th)*cos(beta[*i]) + p[0][*i];
      q[1] = L1*cos(th)*sin(beta[*i]) + p[1][*i];
      q[2] = L1*sin(th);
      
    //calculation of distance between according platform attachment point and base attachment point
      dl[0] = rxp[0][*i] - q[0];
      dl[1] = rxp[1][*i] - q[1];
      dl[2] = rxp[2][*i] - q[2];
      dl2 = sqrt(dl[0]*dl[0] + dl[1]*dl[1] + dl[2]*dl[2]);
      
    //if this distance is the same as leg length, value of theta_a is corrent, we return it
      if(abs(L2-dl2)<0.01){
         return th;
      }
    //if not, we split the searched space in half, then try next value
      if(dl2<L2){
         max=th;
      }else{
         min=th;
      }
      n+=1;
      if(max==servo_min || min==servo_max){
         return th;
      }
      th = min+(max-min)/2;
   }
   return th;
}

//function calculating rotation matrix
void getmatrix(float pe[])
{
   float psi=pe[5];
   float theta=pe[4];
   float phi=pe[3];
   
   M[0][0] = cos(psi)*cos(theta);
   M[1][0] = -sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi);
   M[2][0] = sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta);

   M[0][1] = sin(psi)*cos(theta);
   M[1][1] = cos(psi)*cos(phi)+sin(psi)*sin(theta)*sin(phi);
   M[2][1] = cos(theta)*sin(phi);

   M[0][2] = -sin(theta);
   M[1][2] = -cos(psi)*sin(phi)+sin(psi)*sin(theta)*cos(phi);
   M[2][2] = cos(theta)*cos(phi);
}

//calculates wanted position of platform attachment poins using calculated rotation matrix
//and translation vector
void getrxp(float pe[])
{
   for(int i=0;i<6;i++){
      rxp[0][i] = T[0]+M[0][0]*(re[0][i])+M[0][1]*(re[1][i])+M[0][2]*(re[2][i]);
      rxp[1][i] = T[1]+M[1][0]*(re[0][i])+M[1][1]*(re[1][i])+M[1][2]*(re[2][i]);
      rxp[2][i] = T[2]+M[2][0]*(re[0][i])+M[2][1]*(re[1][i])+M[2][2]*(re[2][i]);
   }
}

//function calculating translation vector - desired move vector + home translation vector
void getT(float pe[])
{
   T[0] = pe[0]+H[0];
   T[1] = pe[1]+H[1];
   T[2] = pe[2]+H[2];
}
unsigned char setPos(float pe[]){
    unsigned char errorcount;
    errorcount=0;
    for(int i = 0; i < 6; i++)
    {
        getT(pe);
        getmatrix(pe);
        getrxp(pe);
        theta_a[i]=getAlpha(&i);
        if(i==INV1||i==INV2||i==INV3){
            servo_pos[i] = constrain(zero[i] - (theta_a[i])*servo_mult, MIN,MAX);
        }
        else{
            servo_pos[i] = constrain(zero[i] + (theta_a[i])*servo_mult, MIN,MAX);
        }
    }

    for(int i = 0; i < 6; i++)
    {
        if(theta_a[i]==servo_min||theta_a[i]==servo_max||servo_pos[i]==MIN||servo_pos[i]==MAX){
            errorcount++;
        }
        servo[i].writeMicroseconds(servo_pos[i]);
    }
    return errorcount;
}

void loop() {

  uint16_t adc;
  float temp, temp1;
  float pe[] = {0,0,0,3.544, 6.544, 15};    //x,y,z,y,p,r
  float voltage;

  adc = analogRead(36);
  voltage = (float)(adc + 200) * (float)(3.3 / 4096.0);
  temp = (voltage - (float)2.637) * (float)(1.0 / -0.0136);
  temp1 = 13.582 - sqrt((13.582 * 13.582) + (4.0 * 0.00433) * (2230.8 - (voltage * 1000.0)));
  temp1 = temp1 / (2.0 * -0.00433);
  temp1 += 30.0;

  printf("temp1 = %.2f, temp = %.2f  voltage = %.4f  adc=%u\n", temp1, temp, voltage, (uint32_t)adc);  

  setPos(pe);
  
  delay(500);
  
  setPos(arr);
  
  delay(500);
  
}