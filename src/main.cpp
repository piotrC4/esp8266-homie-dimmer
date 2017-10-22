/*
 * ESP8266 MQTT Homie based LED dimmer with light sensor (photo-resistor)
 * based on: https://github.com/marvinroger/homie-esp8266
 *
 * Features:
 * - dimming   - devices/_device_ID_/dimmer/value/set (0..100)
 * - switching - devices/_device_ID_/dimmer/switch/set (ON-dim 100%, OFF- dim 0%)
 * - timer     - devices/_device_ID_/dimmer/timer/set (time [s] of 100% dim)
 * - Manual ON/OFF by momentary turning off/on power
 *
 * USED GPIO (ESP1):
 * - GPIO-3 - RX Line - boot delay detector
 * - GPIO-2 - Dimmer output (MOSFET)
 * - GPIO-0 - Homie reset input (pullup + switch to ground)
 * USED GPIO (WEMOS-ESP12)
 * - GPIO-14 - bootdelay detector
 * - GPIO-16 - dimmer output (MOSFET)
 * - GPIO-0  - Homie reset input (pullup + switch to ground)
 *
 * RX Line circuit (momentary reboot detector)
 *              |\ |    _____200k     _____2k
 *   +3,3V _____| >|---|____|---*----|____|-----*--------->GPIO3 (ESP1)
 *              |/ |            |               |          GPIO14 (WEMOS)
 *                            ----47uF        |--| 200k
 *                            ----            |__|
 *                            _|_             _|_
 *
 */
#include <Homie.h>
#include <EEPROM.h>

// Delay in ms for each percentage fade up/down (2ms = 2s full-range dim/1024)
//#define FADE_DELAY 1
// Delay in ms for each percentage fade up/down (15ms = 1,5s full-range dim/100)
#define FADE_DELAY 15

#define NODE_FIRMWARE "LED-dimmer"
#define NODE_VERSION "1.0.72"
#define DEFAULT_DIM_MODE 2

unsigned long downCounterStart;   // wskaznik startu timera
unsigned long downCounterLimit=0; // limit timera

/* ESP01
const int PIN_DIMMER = 2; // GPIO-2
const int PIN_DETECTOR = 3; // GPIO-3 - RX line - boot delay detectror
*/
/* WEMOS */
const int PIN_DIMMER = 16; // GPIO-16
const int PIN_DETECTOR = 14; // GPIO-14

bool lockMode = false; // Lock mode - update to OpenHab

int initDetecorVal;         // Value of fast reboot detector 0-normal, 1-fast
static int currentAbsolute=0;  // Current dim level absolute 0..1023
static int currentPercentage=0; // Current dim percentage 0..100
unsigned int analogWriteFreqVal = 200; // SOFT PWM frequency
unsigned long int startMomentAnalog = 0; // znacznik czasu odczytu analoga
int analogState = 0 ; // Ostatni zapamietany odczyt analoga

HomieSetting<long> dimMode("dimMode", "Dimmer start mode"); // 1 - with detector, 2 - always on on start
struct EEpromDataStruct {
  int currentPercentage;
};

EEpromDataStruct EEpromData;
HomieNode dimmerNode("dimmer", "dimmer");
HomieNode lightSensorNode("light", "voltage");

/***
 * Fade LED up/down (absolute based)
 */
void fadeToAbsolute( int toAbsolute)
{
  int delta = ( toAbsolute - currentAbsolute ) < 0 ? -1 : 1;
  while ( currentAbsolute != toAbsolute )
  {
    currentAbsolute += delta;
    analogWrite(PIN_DIMMER, currentAbsolute);
    delay( FADE_DELAY );
  }
  if (toAbsolute == 1000)
  {
    digitalWrite(PIN_DIMMER, true);
    currentPercentage = 100;
  } else {
    currentPercentage = (currentAbsolute / 10);
  }
  EEpromData.currentPercentage = currentPercentage;
  EEPROM.put(0, EEpromData);
  EEPROM.commit();
}

/***
 * Fade LED up/down (percentage based)
 */
void fadeToPercentage( int toPercentage )
{
  currentPercentage = toPercentage;
  EEpromData.currentPercentage = toPercentage;
  EEPROM.put(0, EEpromData);
  EEPROM.commit();
  //int toAbsolute = ( toPercentage * 1000 ) / 100;
  //int toAbsolute = toPercentage;
  int toAbsolute = (toPercentage*toPercentage)/10;
  int delta = ( toAbsolute - currentAbsolute ) < 0 ? -1 : 1;
  while ( currentAbsolute != toAbsolute )
  {
    currentAbsolute += delta;
    analogWrite(PIN_DIMMER, currentAbsolute);
    delay( FADE_DELAY );
  }
  if (toPercentage==100)
  {
    digitalWrite(PIN_DIMMER, true);
  }
}

/*
 * Update status of the node items to MQTT boker
 */
void updateNodeStatus()
{

  if (currentAbsolute>0)
  {
    dimmerNode.setProperty(  "switch").send( "ON");
  } else {
    dimmerNode.setProperty(  "switch").send( "OFF");
  }
  String msg;
  msg = currentPercentage;
  dimmerNode.setProperty(  "percentage").send( msg );
  msg = currentAbsolute;
  dimmerNode.setProperty(  "absolute").send( msg );
  msg = downCounterLimit;
  dimmerNode.setProperty(  "timer").send( msg );

}

/*
 * Setup (inside Homie)
 */
void handlerSetup()
{
  EEPROM.begin(sizeof(EEpromData));
  EEPROM.get(0,EEpromData);

  // korekcja wartosci dimmera
  if (EEpromData.currentPercentage>100 && EEpromData.currentPercentage<0)
  {
    EEpromData.currentPercentage = 0;
  }

  //Serial.printf("currentPercentage: %d\n",EEpromData.currentPercentage);


  switch ((int)dimMode.get())
  {
    case 1: //  1 - with detector
      Serial.println("DETECTOR mode");
      // We had fast reboot - dimmer state should change
      if (initDetecorVal==1)
      {
        Serial.println(" - fast reboot");
        // Inversion of dimmer state - compare to EEPROM data
        if (EEpromData.currentPercentage>0)
        {
          currentAbsolute=0;
          currentPercentage=0;
          EEpromData.currentPercentage=0;
        } else {
          currentAbsolute=1000;
          currentPercentage=100;
          EEpromData.currentPercentage=100;
        }
        // Without delay ESP may reboot 2 times and change of state don't have a place
        delay(3001);
      } else {
        // Slow Reboot -  0
        Serial.println(" - slow reboot");
        currentAbsolute=0;
        currentPercentage=0;
        EEpromData.currentPercentage=0;
      }
      break;
    case 2: // 2 - always ON on start
      Serial.println("ON on start mode");
      currentAbsolute=1000;
      currentPercentage=100;
      EEpromData.currentPercentage=100;
      break;
    case 3: // 3 - always OFF on start
    default:
      Serial.println("OFF on start mode");
      currentAbsolute=0;
      currentPercentage=0;
      EEpromData.currentPercentage=0;
  }
  EEPROM.put(0, EEpromData);
  EEPROM.commit();

  pinMode(PIN_DIMMER, OUTPUT);
  analogWrite(PIN_DIMMER, currentAbsolute);

  String msg;
  msg = initDetecorVal;
  dimmerNode.setProperty( "starterStatus").send( msg);
  msg = dimMode.get();
  dimmerNode.setProperty( "dimMode").send( msg);
  updateNodeStatus();
}

/*
 * Loop (inside Homie)
 */
void handlerLoop()
{
  // Timer processing
  if (downCounterLimit>0)
  {
    if ((millis() - downCounterStart ) > (downCounterLimit*1000))
    {
      downCounterLimit = 0;
      fadeToPercentage( 0 );
      updateNodeStatus();
    }
  }
  if (millis()-startMomentAnalog>2000)
  {
    startMomentAnalog=millis();
    int newAnalogState = analogRead(A0);
    if (abs(newAnalogState - analogState)>20)
    {
      analogState = newAnalogState;
      float voltage = analogState * (3.3 / 1023.0);
      Serial.print(analogState);
      Serial.print(" - ");
      Serial.println(voltage);
      String msg;
      msg = voltage;
      lightSensorNode.setProperty( "value").send( msg);
    }
  }
}

/*
 * MQTT event processing - dimmer value request percentage
 */
bool handlerDimmerPerc(const HomieRange& range, const String& message)
{
  int requestedPercentage;
  if (message.toInt() > 0)
  {
    downCounterLimit = 0;
    requestedPercentage = message.toInt();
    requestedPercentage = requestedPercentage > 100 ? 100 : requestedPercentage;
    requestedPercentage = requestedPercentage < 1   ? 1   : requestedPercentage;
    if (requestedPercentage != currentPercentage)
    {
      fadeToPercentage( requestedPercentage );
    }
    updateNodeStatus();
    return true;
  } else if (message=="0") {
    downCounterLimit = 0;
    if (currentPercentage!=0)
    {
      fadeToPercentage( 0 );
    }
    updateNodeStatus();
  }
  return false;
}
/*
 * MQTT event processing - dimmer value request absolute
 */
bool handlerDimmerAbs(const HomieRange& range, const String& message)
{
  int requestedAbsolute;
  if (message.toInt() > 0)
  {
    downCounterLimit = 0;
    requestedAbsolute = message.toInt();
    requestedAbsolute = requestedAbsolute > 1000 ? 1000 : requestedAbsolute;
    requestedAbsolute = requestedAbsolute < 1 ? 1 : requestedAbsolute;
    if (currentAbsolute!=requestedAbsolute)
    {
      fadeToAbsolute( requestedAbsolute );
    }
    updateNodeStatus();
    return true;
  } else if (message=="0") {
    downCounterLimit = 0;
    if (currentAbsolute!=0)
    {
      fadeToAbsolute( 0 );
    }
    updateNodeStatus();
  }
  return false;
}
/*
 * MQTT event processing - analog write freqency of Soft PWM
 */
bool dimmerHandlerFreq(const HomieRange& range, const String& message)
{
  int requestedFreq;
  if (message.toInt() >= 200)
  {
    requestedFreq = message.toInt();
    analogWriteFreq(requestedFreq);
    String msg;
    msg = requestedFreq;
    dimmerNode.setProperty(  "frequency").send( msg );
  }
}

/*
 * MQTT event processing - ON/OFF switch request
 */
bool handlerSwitch(const HomieRange& range, const String& message)
{
   if (message=="ON")
   {
     downCounterLimit = 0;
     fadeToPercentage( 100 );
     updateNodeStatus();
     return true;
   }
   if (message=="OFF")
   {
     downCounterLimit = 0;
     fadeToPercentage( 0 );
     updateNodeStatus();
     return true;
   }
   return false;
}
/*
 * Homie event processing
 */
void onHomieEvent(const HomieEvent& event)
{
  switch(event.type)
  {
    case HomieEventType::CONFIGURATION_MODE:
      // Do whatever you want when configuration mode is started
      fadeToPercentage(100);
      break;
    case HomieEventType::OTA_STARTED:
      // Do whatever you want when OTA is started
      fadeToPercentage(100);
    break;
  }
}
/*
 * MQTT event processing - timer Request
 */
bool handlerTimer(const HomieRange& range, const String& message)
{
   if (message.toInt() > 0)
   {
     if (currentPercentage == 0)
     {
       fadeToPercentage( 100 );
     }
     downCounterLimit = message.toInt();
     downCounterStart = millis();
     updateNodeStatus();
     return true;
   }
   return false;
}

/*
 * Main setup - Homie initialization
 */
void setup()
{


  pinMode(PIN_DETECTOR, INPUT_PULLUP);
  initDetecorVal=digitalRead(PIN_DETECTOR);


//  analogWrite(PIN_DIMMER, currentAbsolute);
  analogWriteFreq(analogWriteFreqVal);
  analogWriteRange(1000);

//  Serial.begin(115200);
//  delay(10);

  downCounterLimit = 0;


  /* Initiate homie object */
  Homie_setFirmware(NODE_FIRMWARE, NODE_VERSION);
  Homie_setBrand("MyIOT");
  Homie.disableLedFeedback();
  Homie.setSetupFunction(handlerSetup);
  Homie.setLoopFunction(handlerLoop);
  Homie.disableLogging();
  Homie.onEvent(onHomieEvent); // before Homie.setup()
  dimmerNode.advertise("absolute").settable(handlerDimmerAbs);
  dimmerNode.advertise("percentage").settable( handlerDimmerPerc);
  dimmerNode.advertise("switch").settable( handlerSwitch);
  dimmerNode.advertise("timer").settable( handlerTimer);
  lightSensorNode.advertise("value");
  dimMode.setDefaultValue(DEFAULT_DIM_MODE).setValidator([] (long candidate) {
    return (candidate >= 1) && (candidate <= 3);
  });
  Homie.setup();
}

/*
 * Main loop - only homie support
 */
void loop()
{
  Homie.loop();
}
