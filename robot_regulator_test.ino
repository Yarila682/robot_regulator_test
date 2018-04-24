

#include "fixed.h"



#define SERIAL_SPEED 115200

#define portOfPin(P)\
  (((P)>=0&&(P)<8)?&PORTD:(((P)>7&&(P)<14)?&PORTB:&PORTC))
#define ddrOfPin(P)\
  (((P)>=0&&(P)<8)?&DDRD:(((P)>7&&(P)<14)?&DDRB:&DDRC))
#define pinOfPin(P)\
  (((P)>=0&&(P)<8)?&PIND:(((P)>7&&(P)<14)?&PINB:&PINC))
#define pinIndex(P)((uint8_t)(P>13?P-14:P&7))
#define pinMask(P)((uint8_t)(1<<pinIndex(P)))

#define pinAsInput(P) *(ddrOfPin(P))&=~pinMask(P)
#define pinAsInputPullUp(P) *(ddrOfPin(P))&=~pinMask(P);digitalHigh(P)
#define pinAsOutput(P) *(ddrOfPin(P))|=pinMask(P)
#define digitalLow(P) *(portOfPin(P))&=~pinMask(P)
#define digitalHigh(P) *(portOfPin(P))|=pinMask(P)
#define isHigh(P)((*(pinOfPin(P))& pinMask(P))>0)
#define isLow(P)((*(pinOfPin(P))& pinMask(P))==0)
#define digitalState(P)((uint8_t)isHigh(P))


#define CONNECTION_LOST_TIME_INTERVAL 2000

const byte MOTOR_STATE_DISABLED = 0;
const byte MOTOR_STATE_ENABLED = 1;
const byte MOTOR_STATE_STEPS_LIMIT = 2;

const byte DIRECTION_FORWARD = 0;
const byte DIRECTION_BACKWARD = 0xFF;



bool first;



struct Motor {
  public:
    byte direction;

    volatile byte currentSpeed;

    byte maxSpeed;

    volatile unsigned int stepsLimit;

    volatile unsigned int stepsCnt;
    volatile unsigned int stepsDistance;
    volatile unsigned int stepsPath;

    byte directionPin;
    byte speedPin;

    void initMotor(byte dPin, byte sPin) {
      direction = DIRECTION_FORWARD;
      currentSpeed = 0;
      maxSpeed = 32;
      stepsLimit = 0;

      stepsCnt = 0;

      directionPin = dPin;
      speedPin = sPin;

      pinMode(directionPin, OUTPUT);
      pinMode(speedPin, OUTPUT);
    }

    void stop() {
      currentSpeed = 0;
      
//      analogWrite(speedPin, 0);
//      analogWrite(directionPin, 0);
      digitalWrite(speedPin, HIGH);
      digitalWrite(directionPin, HIGH);
   }


    void enableAndSetStepsLimit(unsigned int iStepsLimit) {
      stepsCnt = 0;
      stepsLimit = iStepsLimit;
    }






    void setSpeedAndDirection(byte speed, byte dir) {
      direction = dir;
      maxSpeed = speed;
      currentSpeed = maxSpeed;

      if(speed == 0){
         digitalWrite(speedPin, HIGH);
         digitalWrite(directionPin, HIGH);
      }
      else{
         if (dir == 0) {
           analogWrite(speedPin, currentSpeed * 4);
           analogWrite(directionPin, 0);
         }
         else {
           analogWrite(speedPin, 0);
           analogWrite(directionPin, currentSpeed * 4);
         }
      }
    }








    void onStep() {
      if (stepsLimit > 0) {
        stepsCnt++;
      }

      stepsPath++;

      if (direction == DIRECTION_FORWARD) {
        stepsDistance++;
      }
      else {
        stepsDistance--;
      }


      if (stepsCnt > 65535) {
        stepsCnt = 0;
      }
      if (stepsPath > 65535) {
        stepsPath = 0;
      }



      if (stepsDistance > 65535) {
        stepsDistance = 0;
      }
    }

};

Motor leftMotor;
Motor rightMotor;

float speed_proportion = 0;

byte globalLeftMotorSpeed;
byte globalRightMotorSpeed;

byte globalLeftDir;
byte globalRightDir;


float m_lastInput = 0;
float m_lastResult = 0;



void onLeftMotorStep() {
  leftMotor.onStep();

  if ((leftMotor.stepsLimit > 0 || rightMotor.stepsLimit > 0) && (leftMotor.stepsCnt >= leftMotor.stepsLimit || rightMotor.stepsCnt >= rightMotor.stepsLimit)) {
    leftMotor.stepsLimit = 0;
    rightMotor.stepsLimit = 0;
    leftMotor.stop();
    rightMotor.stop();
  }

  

}

void onRightMotorStep() {
  rightMotor.onStep();

  if ((leftMotor.stepsLimit > 0 || rightMotor.stepsLimit > 0) && (leftMotor.stepsCnt >= leftMotor.stepsLimit || rightMotor.stepsCnt >= rightMotor.stepsLimit)) {
    leftMotor.stepsLimit = 0;
    rightMotor.stepsLimit = 0;
    leftMotor.stop();
    rightMotor.stop();
  }

}




void setup() {

    

  Serial.begin(SERIAL_SPEED);

  //Let's set the PWR timer
  bitSet(TCCR1B, WGM12);

 
  

 


  leftMotor.initMotor(10, 9);
  rightMotor.initMotor(6, 5);

  attachInterrupt(1, onLeftMotorStep,  CHANGE);
  attachInterrupt(0, onRightMotorStep, CHANGE);


 

   m_lastInput = 0;
   m_lastResult = 0;

   first = true;
}


float Ks = 0.9;
float Kd = 0.8;
float Kr = 1.3;


//PID regulator 
float PID(float input){

   // float  sum = m_lastResult + m_integral * m_period * input;
  //  float diff = m_differential / m_period * (input - m_lastInput);
  //  float result = m_proportional * input + sum + diff;

  float  sum = m_lastResult + Ks * input;
  float diff = Kd * (input - m_lastInput);
  float result = Kr * input + sum + diff;

   // result = qMax(m_minSaturation, result);
   // result = qMin(m_maxSaturation, result);

    m_lastResult = sum;
    m_lastInput = input;

    return result;
      
}


void update_power_using_PID(byte leftSpeed, byte rightSpeed,float input, bool need_to_correct_right_speed ){

 float corrector_koef = 0; 

 byte leftSpeedCorrected = leftSpeed;
 byte rightSpeedCorrected = rightSpeed;

      corrector_koef = PID(input);

             Serial.print("  corrector_koef: ");
             Serial.println(corrector_koef);

      if (need_to_correct_right_speed){

        
          rightSpeedCorrected = byte((float)rightSpeed + corrector_koef);
          leftSpeedCorrected = byte((float)leftSpeed - corrector_koef);

          
          

         // leftMotor.setSpeedAndDirection(0,    globalLeftDir);
         // rightMotor.setSpeedAndDirection(rightSpeedCorrected,  globalRightDir);
        
      }else{


        corrector_koef = abs(corrector_koef);

        rightSpeedCorrected = byte((float)rightSpeed - corrector_koef);
        leftSpeedCorrected = byte((float)leftSpeed + corrector_koef);


       //   leftSpeedCorrected = byte(corrector_koef * leftSpeed);

        //  leftMotor.setSpeedAndDirection(leftSpeedCorrected,    globalLeftDir);
         // rightMotor.setSpeedAndDirection(0,  globalRightDir);
        
      }


            Serial.print(" leftSpeedCorrected: ");
            Serial.println(leftSpeedCorrected);

            Serial.print(" rightSpeedCorrected: ");
            Serial.println(rightSpeedCorrected);

             Serial.println(" ");
      

       leftMotor.setSpeedAndDirection(leftSpeedCorrected,    globalLeftDir);
       rightMotor.setSpeedAndDirection(rightSpeedCorrected,  globalRightDir);

      
  
}

void loop() {

      if (first){
      
              byte leftSpeed =  30;
              byte rightSpeed = 30;

              byte leftDir = DIRECTION_FORWARD;
              byte rightDir = DIRECTION_FORWARD;

              if ( leftSpeed >= 64) {
                leftDir = DIRECTION_BACKWARD;
                leftSpeed -= 64;
              }
              if ( rightSpeed >= 64) {
                rightDir = DIRECTION_BACKWARD;
                rightSpeed -= 64;
              }

               globalLeftMotorSpeed = leftSpeed;
               globalRightMotorSpeed = rightSpeed;

               globalLeftDir = leftDir;
               globalRightDir = rightDir;

               speed_proportion = leftSpeed / rightSpeed;


          

                    leftMotor.setSpeedAndDirection(leftSpeed, leftDir);
                    rightMotor.setSpeedAndDirection(rightSpeed, rightDir);
                    first = false;
              
            }

              
      
      
      /*
   * 
   * Morors regulator algorithm
   * 
   * 
   */

   // Fixed steps_proportion;

    float steps_proportion;
    float e = 0; //невязка

  /*
   *  TODO
   * 
   *  Check whether the power was turn on.
   * 
   */

   
  if ( ((leftMotor.stepsPath - rightMotor.stepsPath ) > 2 ) && (leftMotor.stepsPath > 0) && (rightMotor.stepsPath > 0) ){


          
       // steps_proportion =  steps_proportion.div(Fixed::fromInt(leftMotor.stepsPath),Fixed::fromInt(rightMotor.stepsPath));


            steps_proportion = (float)leftMotor.stepsPath / (float)rightMotor.stepsPath;


            Serial.print(" leftMotor.stepsPath: ");
             Serial.println(leftMotor.stepsPath);

             Serial.print(" rightMotor.stepsPath: ");
             Serial.println(rightMotor.stepsPath);

             Serial.print(" steps_proportion: ");
           //  Serial.print(steps_proportion.fixed2float());

            Serial.println(steps_proportion);

             Serial.print(" speed_proportion: ");
             Serial.println(speed_proportion);

        if (steps_proportion != speed_proportion){


              e = steps_proportion - speed_proportion; //вычисляем невязку

             Serial.print(" e: ");
             Serial.println(e);

               

              if (e > 0){ //левый мотор перемещается быстрее, чем нужно

                 update_power_using_PID(globalLeftMotorSpeed,globalRightMotorSpeed,e,true);
                
              }else{//правый мотор перемещается быстрее, чем нужно


                  update_power_using_PID(globalLeftMotorSpeed,globalRightMotorSpeed,e,false);
                
              }


             
        } 
    
  }
  
}
