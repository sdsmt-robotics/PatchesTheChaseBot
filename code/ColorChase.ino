//----------------------------------------------------------------------------
// Author: Stephanie Athow
// Date: 13 February 2016
// Description:
//		This program runs on an Arduino Uno with the Pixy (CMUCam5) as the 
//		vision input and the Sparkfun RedBot chassis. 
// 		The code is modified from the code provided by Charmed Labs at:
//			http://cmucam.org/boards/9/topics/2898
//-----------------------------------------------------------------------------
#include <SPI.h>
#include <Pixy.h>

Pixy pixy;

#define LEFT_AXIS       0
#define RIGHT_AXIS      1
#define Y_TRACK         130 		// distance to the object before stopping

#define X_CENTER        (320/2) 	
#define MOTOR_MAX       500
#define MOTOR_MIN       -500

// Motor drive pins
int RightMotorPin = 6;
int LeftMotorPin  = 5;

//-----------------------------------------------------------------------------
// Start MotorLoop Class
//-----------------------------------------------------------------------------
class MotorLoop
{
  public:
  MotorLoop(uint32_t pgain, uint32_t dgain);

  int32_t update(int32_t error);

  private:

  int32_t m_prevError;
  int32_t m_pgain;
  int32_t m_dgain;
};

// Motor Loop Construction
MotorLoop::MotorLoop(uint32_t pgain, uint32_t dgain)
{
  m_pgain = pgain;
  m_dgain = dgain;
  m_prevError = 0x80000000; // to indicate that it's never been set
}

// control loop update!
// calculates new output based on measured error and current state
int32_t MotorLoop::update(int32_t error)
{
  int32_t vel;
  
  if (m_prevError!=0x80000000) 
  {
    vel = (error*m_pgain + (error - m_prevError)*m_dgain)/1000; // calc proportional-derivative
    
    // saturation
    if (vel>MOTOR_MAX)
      vel = MOTOR_MAX;
    
    else if (vel<MOTOR_MIN)
    vel = MOTOR_MIN;
  }
  m_prevError = error;

  return vel;
}

static MotorLoop g_transLoop(500, 800);
static MotorLoop g_rotLoop(700, 900);

//----------------------------------
// End MotorLoop Class
//----------------------------------


// Map axes!  go from translational/rotational to left/right wheel space
void axisMap(int32_t in[], int32_t out[])
{
  out[0] = (in[0] - in[1])/2;
  out[1] = (in[0] + in[1])/2;
}

// Send PWM commands to appropriate wheel
void setMotorVoltage( int AXIS, int32_t out )
{
  // left wheel
  if( AXIS == 1 )
  {
    analogWrite( LeftMotorPin, out ); 
  }

  // right wheel
  if( AXIS == 0 )
  {
    analogWrite( RightMotorPin, out );
  }
  
}

// calculate left and right wheel commands based on x, y value of blob
// just call this over and over again and robot will chase blob
void combine(uint32_t x, uint32_t y)
{
  int32_t xError;
  int32_t yError;
  int32_t axesIn[2];
  int32_t axesOut[2];

  // Calculate error
  xError = X_CENTER-x;
  yError = Y_TRACK-y;

  // Determine translation and rotation needed for correction
  axesIn[0] = g_transLoop.update(yError);
  axesIn[1] = g_rotLoop.update(xError);

  // Convert to axes frame
  axisMap(axesIn, axesOut);

  // Send motor commands
  setMotorVoltage(LEFT_AXIS, axesOut[0]);
  setMotorVoltage(RIGHT_AXIS, axesOut[1]);
}


void setup() 
{
  Serial.begin( 9600 );					// communication baud rate
  // Serial.print( "Starting ... \n" ); // used for debugging

  // Initialize pixy
  pixy.init();

  pinMode( RightMotorPin, OUTPUT );
  pinMode( LeftMotorPin, OUTPUT );
  
}

uint32_t lastBlockTime = 0;		// time the last pixy blocks were recieved.

void loop() 
{
  uint16_t blocks;					// block information for a color

  // Get blocks
  blocks = pixy.getBlocks();

  // if we see a color, go chase it!
  if( blocks )
  {
    // adjust left/right and move forward
    combine( (uint32_t) pixy.blocks[0].x, (uint32_t) pixy.blocks[0].y );
    lastBlockTime = millis();				            // timestamp block recieved
  }
  
  // no new blocks recieved in last .1 second, stop
  else if( millis() - lastBlockTime > 100 )
  {
    setMotorVoltage( LEFT_AXIS, 0 );
    setMotorVoltage( RIGHT_AXIS, 0 );
  }

}





