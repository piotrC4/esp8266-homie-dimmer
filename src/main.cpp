/*
 * ESP8266 MQTT Homie based LED dimmer with light sensor (photo-resistor)
 */
#include <Homie.h>
#include <EEPROM.h>

// Delay in ms for each percentage fade up/down (2ms = 2s full-range dim/1024)
#define FADE_DELAY 1

#define DEFAULT_DIM_MODE 2

unsigned long downCounterStart;   // wskaznik startu timera
unsigned long downCounterLimit=0; // limit timera


/* ESP01
const int PIN_DIMMER = 2; // GPIO-2
const int PIN_DETECTOR = 3; // GPIO-3 - RX line - boot delay detectror
*/
/* WEMOS */
const int PIN_DIMMER = 12;    // GPIO-12
const int PIN_DETECTOR = 14;  // GPIO-14

int initDetecorVal;                       // Value of fast reboot detector 0-normal, 1-fast
static int currentAbsolute=0;             // Current dim level absolute 0..1000
static int targetAbsolute=0;              // Target dimmer level absoulute 0..1000
static int currentPercentage=0;           // Current dim percentage 0..100
unsigned int analogWriteFreqVal = 200;    // SOFT PWM frequency
unsigned long int startMomentAnalog = 0;  // Analog read time marker
unsigned long int startMomentDimming = 0; // Dimmer time marker
int lastAnalogState = 0 ;                 // Last analog read
int dimmmDelta = 0;                       // Dimmer change current delta

HomieSetting<long> dimMode("dimMode", "Dimmer start mode"); // 1 - with detector, 2 - always on on start
struct EEpromDataStruct {
  int currentPercentage;
};

EEpromDataStruct EEpromData;
HomieNode dimmerNode("dimmer", "dimmer");
HomieNode lightSensorNode("light", "voltage");

/*
 * Fade LED up/down (absolute based)
 */
void fadeToAbsolute( int toAbsolute)
{
  int delta = ( toAbsolute - currentAbsolute ) < 0 ? -1 : 1;
  targetAbsolute = toAbsolute;
  dimmmDelta = delta;
  startMomentDimming = millis();
  return;
}

/*
 * Fade LED up/down (percentage based)
 */
void fadeToPercentage( int toPercentage )
{
  fadeToAbsolute(toPercentage * 10);
  return;
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
  switch ((int)dimMode.get())
  {
    case 1: //  1 - with detector
      Homie.getLogger() << "DETECTOR mode" << endl;
      // We had fast reboot - dimmer state should change
      if (initDetecorVal==1)
      {
        Homie.getLogger() << " - fast reboot" << endl;
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
        Homie.getLogger() << " - slow reboot" << endl;
        currentAbsolute=0;
        currentPercentage=0;
        EEpromData.currentPercentage=0;
      }
      break;
    case 2: // 2 - always ON on start
      Homie.getLogger() << "ON on start mode" << endl;
      currentAbsolute=1000;
      currentPercentage=100;
      EEpromData.currentPercentage=100;
      break;
    case 3: // 3 - always OFF on start
    default:
      Homie.getLogger() << "OFF on start mode" << endl;
      currentAbsolute=0;
      currentPercentage=0;
      EEpromData.currentPercentage=0;
  }
  EEPROM.put(0, EEpromData);
  EEPROM.commit();

  pinMode(PIN_DIMMER, OUTPUT);
  analogWrite(PIN_DIMMER, currentAbsolute);

  targetAbsolute = currentAbsolute;
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
    }
  } else
  if (millis()-startMomentAnalog>2000)
  {
    startMomentAnalog=millis();
    int newAnalogState = analogRead(A0);
    if (abs(newAnalogState - lastAnalogState)>20)
    {
      lastAnalogState = newAnalogState;
      float voltage = lastAnalogState * (3.3 / 1023.0);
      Homie.getLogger() << "analog: " <<lastAnalogState << " - " << voltage << endl;
      String msg;
      msg = voltage;
      lightSensorNode.setProperty( "value").send( msg);
    }
  } else
  if (targetAbsolute != currentAbsolute && millis()-startMomentDimming>FADE_DELAY)
  {
    // Dimming have place
    currentAbsolute += dimmmDelta;
    analogWrite(PIN_DIMMER, ((currentAbsolute*currentAbsolute)/1000));
    startMomentDimming=millis();
    if (targetAbsolute == currentAbsolute)
    {
      // End of dimming
      if (targetAbsolute == 1000)
      {
        digitalWrite(PIN_DIMMER, true);
        currentPercentage = 100;
      } else {
        currentPercentage = (currentAbsolute / 10);
      }
      if (dimMode.get()==1)
      {
        EEpromData.currentPercentage = currentPercentage;
        EEPROM.put(0, EEpromData);
        EEPROM.commit();
      }
      updateNodeStatus();
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
    return true;
  } else if (message=="0") {
    downCounterLimit = 0;
    if (currentPercentage!=0)
    {
      fadeToPercentage( 0 );
    }
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
    return true;
  } else if (message=="0") {
    downCounterLimit = 0;
    if (currentAbsolute!=0)
    {
      fadeToAbsolute( 0 );
    }
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
  return true;
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
     return true;
   }
   if (message=="OFF")
   {
     downCounterLimit = 0;
     fadeToPercentage( 0 );
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
    case HomieEventType::NORMAL_MODE:
    case HomieEventType::OTA_PROGRESS:
    case HomieEventType::OTA_SUCCESSFUL:
    case HomieEventType::OTA_FAILED:
    case HomieEventType::ABOUT_TO_RESET:
    case HomieEventType::WIFI_CONNECTED:
    case HomieEventType::WIFI_DISCONNECTED:
    case HomieEventType::MQTT_READY:
    case HomieEventType::MQTT_DISCONNECTED:
    case HomieEventType::MQTT_PACKET_ACKNOWLEDGED:
    case HomieEventType::READY_TO_SLEEP:
    case HomieEventType::SENDING_STATISTICS:
    case HomieEventType::STANDALONE_MODE:
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
  Serial.begin(CUST_SERIAL_SPEED);

  EEPROM.begin(sizeof(EEpromData));
  EEPROM.get(0,EEpromData);

  // eeprom data correction
  if (EEpromData.currentPercentage>100 && EEpromData.currentPercentage<0)
  {
    EEpromData.currentPercentage = 0;
  }

  pinMode(PIN_DETECTOR, INPUT_PULLUP);
  initDetecorVal=digitalRead(PIN_DETECTOR);

  analogWriteFreq(analogWriteFreqVal);
  analogWriteRange(1000);

  downCounterLimit = 0;

  /* Initiate homie object */
  Homie_setFirmware(FIRMWARE_NAME, FIRMWARE_VER);
  Homie_setBrand("MyIOT");
  Homie.disableLedFeedback();
  Homie.setSetupFunction(handlerSetup);
  Homie.setLoopFunction(handlerLoop);
  //Homie.disableLogging();
  Homie.onEvent(onHomieEvent);
  dimmerNode.advertise("absolute").settable(handlerDimmerAbs);
  dimmerNode.advertise("percentage").settable( handlerDimmerPerc);
  dimmerNode.advertise("switch").settable( handlerSwitch);
  dimmerNode.advertise("timer").settable( handlerTimer);
  dimmerNode.advertise("freq").settable(dimmerHandlerFreq);
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
