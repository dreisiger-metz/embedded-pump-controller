// =============================================================================
// Filename         : Well_Pump_Controller.ino
// Version          : 
//
// Original Author  : Peter Dreisiger
// Date Created     : 02-Jun-2020
//
// Revision History : 02-Jun-2020, v0.1: Initial version
//                    04-Aug-2020, v0.2: Added support for LCD
//                    to-come,     v0.9: Add parser support for interactive
//                                       querying and parameter setting
//
// Purpose          : 
//                     
// Licensing        : Copyright (C) 2020, Peter Dreisiger
//
// This program is free software: you can redistribute it and/or modify it under
// the terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or (at your option) any later
// version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more 
// details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
//
// ----------------------------------------------------------------------------
// Notes:
//   1. The flow sensor needs to include a resistor divider, so the 0 / 5V
//      output can be dropped to between 0 and 1.1V!
// =============================================================================
#include <EEPROM.h>
#include <LiquidCrystal.h>
#include <Parser.h>
#include <StateMachine.h>


// define DEBUG to enable logging of state changes, etc., to the Serial port;
// note that this approximately triples the size of the resulting code
#define ENABLE_DEBUGGING

#ifdef ENABLE_DEBUGGING
#define LogToSerial(level, message) __LogToSerial(level, message)
#else
#define LogToSerial(level, message)
#endif

enum LogLevel { FATAL, ERROR, WARNING, INFO, VERBOSE, DEBUG };

LogLevel LoggingLevel = VERBOSE;

void __LogToSerial(LogLevel level, String message) {
  if (level <= LoggingLevel)
    Serial.println(message);
}






//
#define DISPLAY_LCD




// =============================================================================
// Class            : VacuumPump
// Derived from     : StateMachine
//
// Purpose          : This class represents the State Machine of the main pump, 
//                    comprising a vacuum pumping subsystem, its lower and upper
//                    water-level sensors, and a cistern-full sensor.
//
//                    The priming and pumping timeouts are there to guard 
//                    against motor damage due to a dry well, while the standby
//                    timeout is there to periodically check to see if the water
//                    has returned to the well.
// =============================================================================
class VacuumPump : public StateMachine {
  public:
    enum { STANDBY, PRIMING, PUMPING, PREDRAINING, DRAINING, POSTDRAINING };

    struct SaveState {
      float StandbyTimeout;     // how long to wait between priming attempts
      float PrimingTimeout;     // upper limit of how long it should take from the start of pumping for the lower sensor to go high
      float PumpingTimeout;     // upper limit of how long the pump should be run for in any given cycle (to guard against the well running dry mid-cycle)
      float PredrainingDelay;   // delay between when we leave the PUMPING state and when the pump should actually be turned off
      float PostdrainingDelay;  // delay between when the sensor says we've finished draining, and when we should actually starting pumping again

      float LitresPerCycle;

      unsigned cycles;
      unsigned long runtime;
    };

    float timer = 0.0;
    Flag lowerSensor, upperSensor, cisternSensor;

    SaveState s;

    static const SaveState Defaults;
};

const VacuumPump::SaveState VacuumPump::Defaults = { 600.0, 120.0, 300.0, 0.9, 9.9, 3.5, 0, 0 };




// =============================================================================
// Class            : DistributionPump
// Derived from     : StateMachine
//
// Purpose          : This class represents the State Machine of the pump re-
//                    sponsible for distributing water from the Well Pump's 
//                    cistern to all of the (likely elevated) dependent water
//                    tanks connected to well.
//
//                    The probing timeout should be long enough for the flow 
//                    sensor to respond if any downstream tanks need filling,
//                    the pumping timeout is there to guard against excessively
//                    long pumping runs, while the standby timeout is there to
//                    periodically re-probe the downstream tanks' water levels.
// =============================================================================
class DistributionPump : public StateMachine {
  public:
    enum { STANDBY, PROBING, PUMPING };

    struct SaveState {
      float StandbyTimeout;  // how long to wait being probings
      float ProbingTimeout;  // how long to run the pump for before we can expect to sense water flow
      float PumpingTimeout;  // how long to run the pump for even if we're still detecting flow

      unsigned long runtime;
    };

    float timer = 0.0;

    Flag flowSensor;

    SaveState s;

    static const SaveState Defaults;
};

//const DistributionPump::SaveState DistributionPump::Defaults = { 300.0, 5.0, 600.0, 0 };
const DistributionPump::SaveState DistributionPump::Defaults = { 30.0, 5.0, 600.0, 0 };




// =============================================================================
// Class            : WellController
// Derived from     : StateMachine
//
// Purpose          : This class implements the overarching controller's State
//                    Machine, but it also responsible for reading the sub-
//                    systems' sensors and updating their respective states
// =============================================================================
class WellController : public StateMachine {
  public:
    enum { RUNNING, LOW_VOLTAGE, OVER_CURRENT, HALTED };
    enum { LCD_DISPLAYING_WELL_PUMP, LCD_DISPLAYING_DISTRIB_PUMP };

    struct SaveState {
      float DriveVoltageThreshold;
      float DriveCurrentThreshold;
    };

    // Note that we can't (reliably) use pin 13 / the LED pin as a pull-up
    // input, due to its likely load; it can be used as an output pin, though
    static const byte  VacuumPumpDriverPin       = 11;
    static const byte  DistributionPumpDriverPin = 10;

    static const byte  LowerSensorPin            = 12;
    static const byte  UpperSensorPin            = 4;
    static const byte  CisternSensorDriverPin    = 13;
    static const byte  CisternSensorPin          = A0;
    static const byte  FlowSensorPin             = A1;
    static const byte  DriveVoltageSensePin      = A3;
    static const byte  DriveCurrentSensePin      = A2;

    #ifdef DISPLAY_LCD
    static const byte  LCDRSPin                  = 2;
    static const byte  LCDEPin                   = 3;
    static const byte  LCDData4                  = 6;
    static const byte  LCDData5                  = 7;
    static const byte  LCDData6                  = 8;
    static const byte  LCDData7                  = 9;
    static const byte  UIButtonPin               = 5;
    #else
    static const byte  PrimingLEDPin             = 2;
    static const byte  PumpingLEDPin             = 3;
    static const byte  DrainingLEDPin            = 6;
    #endif

    // On an Arduino Pro Mini, since fewer I/O pins are brought out to the standard
    // DIP pins, we need to use some of the non-standard-placement pins (in this
    // case A4 and A5) for this functionality, and in order to use the INPUT_PULLUP
    // pin mode, we also require an adjacent pin which can be set to OUTPUT::LOW
    static const byte  ResetStatsJumperSensePin  = A5;
    static const byte  ResetStatsJumperGndPin    = A4;

    // Scaling factors calculated using a 1.1V INTERNAL analogue reference...
    static const float DriveVoltageScalingFactor = 16.5 / 1023;  // assuming a divide-by-15 voltage divider, 1023 --> 1.1 * 15, or 16.5V
    static const float DriveCurrentScalingFactor = 11.0 / 1023;  // assuming a 0.1R shunt resistor, 1023 --> 1.1V / 0.1, or 11A
    static const float DiodeVoltageDrop          = 0.66;

    static const SaveState Defaults;

    WellController(VacuumPump &vp, DistributionPump& dp, float tickInterval);
    ~WellController() { delete lcd; }

    void readSensors();
    void updateVacuumPumpStateMachine();
    void updateDistributionPumpStateMachine();
    void updateLCD();
    void tick();
  
    float tickInterval, driveVoltage, driveCurrent;
    Flag uiButton;
    VacuumPump *vp;
    DistributionPump *dp;
    LiquidCrystal *lcd;
    unsigned lcdCurrentlyDisplaying;

    SaveState s;
};

const WellController::SaveState WellController::Defaults = { 8.0, 6.0 };


// =============================================================================
// Method           : WellController::WellController()
//
// Purpose          : To initialise the controller's variables and I/O pins
//
// Inputs           : [vacpump] reference to the VacuumPump State Machine
//                    [distpump] reference to the DistributionPump State Machine
//                    [tickInterval] the number of seconds between ticks
// =============================================================================
WellController::WellController(VacuumPump &vacpump, DistributionPump& distpump, float tickInterval) {
  this->vp = &vacpump;
  this->dp = &distpump;
  this->tickInterval = tickInterval;

  // Set the various sensor and driver pin modes and initial values
  digitalWrite(VacuumPumpDriverPin, LOW);
  digitalWrite(DistributionPumpDriverPin, LOW);

  pinMode(LowerSensorPin, INPUT_PULLUP);
  pinMode(UpperSensorPin, INPUT_PULLUP);
  pinMode(CisternSensorDriverPin, OUTPUT);
  digitalWrite(CisternSensorDriverPin, LOW);
  pinMode(VacuumPumpDriverPin, OUTPUT);
  digitalWrite(VacuumPumpDriverPin, LOW);
  
  pinMode(DistributionPumpDriverPin, OUTPUT);
  digitalWrite(DistributionPumpDriverPin, LOW);

  pinMode(UIButtonPin, INPUT_PULLUP);
  pinMode(ResetStatsJumperSensePin, INPUT_PULLUP);
  pinMode(ResetStatsJumperGndPin, OUTPUT);
  digitalWrite(ResetStatsJumperGndPin, LOW);


  // If the ResetStats jumper pin is low,
  if (!digitalRead(ResetStatsJumperSensePin)) {
    // set the SaveStates to their default values;
    s.DriveVoltageThreshold = WellController::Defaults.DriveVoltageThreshold;
    s.DriveCurrentThreshold = WellController::Defaults.DriveCurrentThreshold;

    vp->s.StandbyTimeout    = VacuumPump::Defaults.StandbyTimeout;
    vp->s.PrimingTimeout    = VacuumPump::Defaults.PrimingTimeout;
    vp->s.PumpingTimeout    = VacuumPump::Defaults.PumpingTimeout;
    vp->s.PredrainingDelay  = VacuumPump::Defaults.PredrainingDelay;
    vp->s.PostdrainingDelay = VacuumPump::Defaults.PostdrainingDelay;
    vp->s.LitresPerCycle    = VacuumPump::Defaults.LitresPerCycle;
    vp->s.cycles = 0;
    vp->s.runtime = 0;

    dp->s.StandbyTimeout    = DistributionPump::Defaults.StandbyTimeout;
    dp->s.ProbingTimeout    = DistributionPump::Defaults.ProbingTimeout;
    dp->s.PumpingTimeout    = DistributionPump::Defaults.PumpingTimeout;
    dp->s.runtime = 0;

  } else {
    // otherwise, load them from EEPROM
    EEPROM.get(0, s);
    EEPROM.get(sizeof(s), vp->s);
    EEPROM.get(sizeof(s) + sizeof(vp->s), dp->s);
  }


  // Initialise the LCD object
  lcd = new LiquidCrystal(LCDRSPin, LCDEPin, LCDData4, LCDData5, LCDData6, LCDData7);
  lcd->begin(16, 2);
  lcdCurrentlyDisplaying = LCD_DISPLAYING_WELL_PUMP;


  // Set the analogue reference to 1.1V (on ATmega 168 / 328P-based boards).
  // This is done so the voltage across the drive current sensing shunt can be 
  // measured with more precision, without the need for amplication (and because
  // the drive voltage can be easily dropped using a voltage divider)
  analogReference(INTERNAL);

  // And make a couple of 'dummy' calls to analogRead --- from
  // https://arduino.cc/reference/en/language/functions/analog-io/analogreference,
  // "After changing the analog reference, the first few readings from analogRead()
  // may not be accurate."
  analogRead(DriveVoltageSensePin);
  analogRead(DriveVoltageSensePin);
  analogRead(DriveVoltageSensePin);
}


// =============================================================================
// Method           : WellController::readSensors()
//
// Purpose          : To read the sensor pins and update their corresponding 
//                    variables
//
// Inputs           : (none)
// Returns          : void
// =============================================================================
void WellController::readSensors() {
  // First drive the cistern sensor's driver pin high, so 
  digitalWrite(CisternSensorDriverPin, HIGH);

  // Next, read the sensors --- note that, due to their inputs being
  // [INPUT_PULLUP], they'll actually register as TRUE when they're _not_
  // triggered, and FALSE when they are
  vp->lowerSensor   = !digitalRead(LowerSensorPin);
  vp->upperSensor   = !digitalRead(UpperSensorPin);

  // Note that the wheel / hall-effect inline flow sensors pulse in the 10s of
  // Hz range (maybe ~10 for really slow, up to 60 / 70 Hz for fast), so the
  // detector would either need to be connected to an edge-triggered interrupt
  // pin or, say, in parallel with a ~100nF-ish capacitor whose (analogue) 
  // output voltage is then read; for the latter, if the voltage is _neither_
  // 0 nor 5V (since the wheel could stop on either a output-low or -high 
  // point), one can assume there's a pulse train there
  //dp->flowSensor    = !digitalRead(FlowSensorPin);
  dp->flowSensor    = (analogRead(FlowSensorPin) > 102) && (analogRead(FlowSensorPin) < 921);

  // Then read the analogue voltage and current inputs (with a teensy bit of averaging)
  driveVoltage      = (analogRead(DriveVoltageSensePin) + analogRead(DriveVoltageSensePin)) * DriveVoltageScalingFactor / 2.0 + DiodeVoltageDrop;
  driveCurrent      = (analogRead(DriveCurrentSensePin) + analogRead(DriveCurrentSensePin)) * DriveCurrentScalingFactor / 2.0;
  vp->cisternSensor = (analogRead(CisternSensorPin) < 300);

  digitalWrite(CisternSensorDriverPin, LOW);

  // And finally read the UI / LCD Button
  uiButton = !digitalRead(UIButtonPin);
}


// =============================================================================
// Method           : WellController::updateVacuumPumpStateMachine()
//
// Purpose          : To update the Well Pump's State Machine, and any relevant
//                    output / driver pins
//
// Inputs           : (none)
// Returns          : void
//
// ----------------------------------------------------------------------------
// Notes:
//     - The updating is done here (rather than in VacuumPumpStateMachine) because
//       of this need to update the Pump Controller's output pins, and to avoid
//       the need for redundant code and/or hooks in the Well Pump's State Machine
// =============================================================================
void WellController::updateVacuumPumpStateMachine() {
  switch (vp->state) {
    // -------------------------------------------------------------------------
    // Invariant: vp->cisternSensor && vp->timer < VacuumPump::StandbyTimeout
    // Exits to [PRIMING] when this is no longer the case
    case VacuumPump::STANDBY:
    if (vp->state.changed()) {
      digitalWrite(VacuumPumpDriverPin, LOW);
      vp->timer = 0;
    }
    if (!vp->cisternSensor) {
      if (vp->cisternSensor.HtoL()) {
        LogToSerial(VERBOSE, F("%VACUUMPUMP-V-STATE, cistern level dropped below full"));
        LogToSerial(INFO,      "%VACUUMPUMP-I-STATE, going from STANDBY to PRIMING after " + String(vp->timer) + " seconds");
        vp->state = VacuumPump::PRIMING;
      } else if (vp->timer >= vp->s.StandbyTimeout) {
        LogToSerial(VERBOSE, F("%VACUUMPUMP-V-STATE, standby timeout exceeded, trying again"));
        LogToSerial(INFO,      "%VACUUMPUMP-I-STATE, going from STANDBY to PRIMING after " + String(vp->timer) + " seconds");
        vp->state = VacuumPump::PRIMING;
      }
    } // else do nothing as we will always remain in [STANDBY] as long as the cistern is full
    break;
    
    // -------------------------------------------------------------------------
    // Invariant: !vp->lowerSensor && vp->timer < VacuumPump::PrimingTimeout
    // Exits to [PUMPING] if vp->lowerSensor goes high, or to [STANDBY] if the 
    // priming times out
    case VacuumPump::PRIMING:
    if (vp->state.changed()) {
      digitalWrite(VacuumPumpDriverPin, HIGH);
      vp->timer = 0;
    }
    if (vp->lowerSensor) {
      LogToSerial(INFO, "%VACUUMPUMP-I-STATE, going from PRIMING to PUMPING after " + String(vp->timer) + " seconds");
      vp->state = VacuumPump::PUMPING;
    } else if (vp->timer >= vp->s.PrimingTimeout) {
      LogToSerial(VERBOSE, F("%VACUUMPUMP-V-STATE, priming timeout exceeded, perhaps the well is dry?"));
      LogToSerial(INFO,      "%VACUUMPUMP-I-STATE, going from PRIMING to STANDBY after " + String(vp->timer) + " seconds");
      vp->state = VacuumPump::STANDBY;
    }
    break;

    // -------------------------------------------------------------------------
    // Invariant: vp->lowerSensor && !vp->upperSensor && !vp->cisternSensor
    // Exits to [DRAINING] if either vp->upperSensor or vp->cisternSensor go high
    case VacuumPump::PUMPING:
    if (vp->state.changed()) {
      vp->timer = 0;
    }
    if (vp->upperSensor) {
      // should introduce a 1-second delay here (somehow)
      LogToSerial(INFO, "%VACUUMPUMP-I-STATE, going from PUMPING to PREDRAINING after " + String(vp->timer) + " seconds");
      vp->state = VacuumPump::PREDRAINING;
    } else if (vp->cisternSensor) {
      LogToSerial(VERBOSE, F("%VACUUMPUMP-V-STATE, external filling of cistern detected"));
      LogToSerial(INFO,      "%VACUUMPUMP-I-STATE, going from PUMPING to DRAINING after " + String(vp->timer) + " seconds");
      vp->state = VacuumPump::DRAINING;
    } else if (vp->timer >= vp->s.PumpingTimeout) {
      LogToSerial(VERBOSE, F("%VACUUMPUMP-V-STATE, pumping timeout exceeded, perhaps the well is dry?"));
      LogToSerial(INFO,      "%VACUUMPUMP-I-STATE, going from PUMPING to DRAINING after " + String(vp->timer) + " seconds");
      vp->state = VacuumPump::DRAINING;
    }
    break;

    // -------------------------------------------------------------------------
    case VacuumPump::PREDRAINING:
    if (vp->state.changed())
      vp->timer = 0;
    if (vp->timer >= vp->s.PredrainingDelay) {
      LogToSerial(INFO, "%VACUUMPUMP-I-STATE, going from PREDRAINING to DRAINING after " + String(vp->timer) + " seconds");
      vp->state = VacuumPump::DRAINING;
    }

    // -------------------------------------------------------------------------
    // Invariant: vp->lowerSensor
    // Exits once vp->lowerSensor goes low, to [STANDBY] if vp->cisternSensor
    // is high, or to [PUMPING] otherwise
    case VacuumPump::DRAINING:
    if (vp->state.changed()) {
      digitalWrite(VacuumPumpDriverPin, LOW);
      vp->timer = 0;
    }
    if (!vp->lowerSensor) {
      if (vp->cisternSensor) {
        LogToSerial(VERBOSE, F("%VACUUMPUMP-V-STATE, cistern is now full"));
        LogToSerial(INFO,      "%VACUUMPUMP-I-STATE, going from DRAINING to STANDBY after " + String(vp->timer) + " seconds");
        vp->state = VacuumPump::STANDBY;
      } else {
        LogToSerial(INFO, "%VACUUMPUMP-I-STATE, going from DRAINING to POSTDRAINING after " + String(vp->timer) + " seconds");
        vp->state = VacuumPump::POSTDRAINING;
      }
    }
    break;
    
    // -------------------------------------------------------------------------
    // Invariant: 
    case VacuumPump::POSTDRAINING:
    if (vp->state.changed()) {
      vp->timer = 0;
    }
    if (vp->timer >= vp->s.PostdrainingDelay) {
      if (vp->cisternSensor) {
        LogToSerial(VERBOSE, F("%VACUUMPUMP-V-STATE, cistern is now full"));
        LogToSerial(INFO,      "%VACUUMPUMP-I-STATE, going from POSTDRAINING to STANDBY after " + String(vp->timer) + " seconds");
        vp->state = VacuumPump::STANDBY;
      } else {
        LogToSerial(INFO, "%VACUUMPUMP-I-STATE, going from POSTDRAINING to PRIMING after " + String(vp->timer) + " seconds");
        vp->state = VacuumPump::PRIMING;
      }
    }
    break;
    
    // -------------------------------------------------------------------------
    // Invariant: should never get here
    default:
    break;
  }
}
  

// =============================================================================
// Method           : WellController::updateDistributionPumpStateMachine()
//
// Purpose          : To update the Distribution Pump's State Machine, and any 
//                    relevant output / driver pins
//
// Inputs           : (none)
// Returns          : void
//
// ----------------------------------------------------------------------------
// Notes:
//     - The updating is done here (rather than in DistributionPumpStateMachine)
//       because of this need to update the Pump Controller's output pins, and
//       to avoid the need for redundant code and/or hooks in the Distribution
//       Pump's State Machine
// =============================================================================
void WellController::updateDistributionPumpStateMachine() {
  switch (dp->state) {
    // -------------------------------------------------------------------------
    // Invariant: 
    case DistributionPump::STANDBY:
    if (dp->state.changed()) {
      digitalWrite(DistributionPumpDriverPin, LOW);
      dp->timer = 0;
    }
    if (dp->timer >= dp->s.StandbyTimeout) {
        LogToSerial(INFO, "%DISTPUMP-I-STATE, going from STANDBY to PROBING after " + String(dp->timer) + " seconds");
        dp->state = DistributionPump::PROBING;
      }
    break;
    
    // -------------------------------------------------------------------------
    // Invariant: 
    case DistributionPump::PROBING:
    if (dp->state.changed()) {
      digitalWrite(DistributionPumpDriverPin, HIGH);
      dp->timer = 0;
    }
    if (dp->flowSensor) {
      LogToSerial(INFO, "%DISTPUMP-I-STATE, going from PROBING to PUMPING after " + String(dp->timer) + " seconds");
      dp->state = DistributionPump::PUMPING;
    } else if (dp->timer >= dp->s.ProbingTimeout) {
      LogToSerial(VERBOSE, "%DISTPUMP-V-STATE, no flow detected");
      LogToSerial(INFO,    "%DISTPUMP-I-STATE, going from PROBING to STANDBY after " + String(dp->timer) + " seconds");
      dp->state = DistributionPump::STANDBY;
    }
    break;

    // -------------------------------------------------------------------------
    // Invariant: 
    case DistributionPump::PUMPING:
    if (dp->state.changed())
      dp->timer = 0;
    if (!dp->flowSensor) {
      LogToSerial(INFO, "%DISTPUMP-I-STATE, going from PUMPING to STANDBY after " + String(dp->timer) + " seconds");
      dp->state = DistributionPump::STANDBY;
    } else if (dp->timer >= dp->s.PumpingTimeout) {
      LogToSerial(VERBOSE, F("%DISTPUMP-V-STATE, pump runtime has reached threshold"));
      LogToSerial(INFO,      "%DISTPUMP-I-STATE, going from PUMPING to STANDBY after " + String(dp->timer) + " seconds");
      dp->state = DistributionPump::STANDBY;
    }
    break;

    // -------------------------------------------------------------------------
    default:
    break;
  }
}


// =============================================================================
// Method           : WellController::updateLCD()
//
// Purpose          : To update the LCD to show relevant state information
//                    and stats
//
// Inputs           : (none)
// Returns          : void
//
// ----------------------------------------------------------------------------
// Notes: possible example displays include
// First row:
//   1234567890123456        1234567890123456
//   PRIMING       OK        STANDBY   NOFLOW
//   PUMPING       OK        PROBING
//   DRAINING    FULL        PUMPING
//   STANDBY     FULL
//   STANDBY  WELLDRY
//   STANDBY    Vi LO
//   STANDBY    Io HI
//
// Second row:
//   12.0V  1.5A  Luc
// =============================================================================
void WellController::updateLCD() {
  // Currently just going for the simplest case (no interactivity, and only
  // the vacuum pump running)
  String row1, row2;

  switch (lcdCurrentlyDisplaying) {
    case LCD_DISPLAYING_WELL_PUMP:
    // ---------------------------
    switch (vp->state) {
      case VacuumPump::STANDBY:
      if (vp->cisternSensor)
        row1 = F("W STANDBY   FULL");
      else if (driveVoltage < s.DriveVoltageThreshold)
        row1 = F("W STANDBY   V_lo");
      else
        row1 = F("W STANDBY   DRY?");
      break;

      case VacuumPump::PRIMING:
      row1 = F("W PRIMING     OK");
      break;

      case VacuumPump::PUMPING:
      row1 = F("W PUMPING     OK");
      break;

      case VacuumPump::PREDRAINING:
      row1 = F("W PREDRAIN ");
      break;

      case VacuumPump::DRAINING:
      if (vp->cisternSensor)
        row1 = F("W DRAINING  FULL");
      else
        row1 = F("W DRAINING    OK");
      break;

      case VacuumPump::POSTDRAINING:
      if (vp->cisternSensor)
        row1 = F("W POSTDRAIN FULL");
      else
        row1 = F("W POSTDRAIN   OK");
      break;

      default:
      row1 = F("W ERROR");
      break;
    }
    lcdCurrentlyDisplaying = LCD_DISPLAYING_DISTRIB_PUMP;
    break;

    case LCD_DISPLAYING_DISTRIB_PUMP:
    // ------------------------------
    switch (dp->state) {
      case DistributionPump::STANDBY:
      if (driveVoltage < s.DriveVoltageThreshold)
        row1 = F("D STANDBY   V_lo");
      else
        row1 = F("D STANDBY     OK");
      break;

      case DistributionPump::PROBING:
      row1 = F("D PROBING     OK");
      break;

      case DistributionPump::PUMPING:
      row1 = F("D PUMPING     OK");
      break;
    }
    lcdCurrentlyDisplaying = LCD_DISPLAYING_WELL_PUMP;
    break;
  }

  // 1234567890123456
  // 10V   5.0A   110
  row2 = String(driveVoltage, 1) + String("V  ") + String(driveVoltage < 10?" ":"") + String(driveCurrent, 1) + String("A  ") +
         String(vp->lowerSensor?"L":"l") + String(vp->upperSensor?"U":"u") + String(vp->cisternSensor?"C":"c");

  lcd->clear();
  lcd->print(row1);
  lcd->setCursor(0, 1);
  lcd->print(row2);
}


// =============================================================================
// Method           : WellController::tick()
//
// Purpose          : To perform the Pump Controller's regular / periodic tasks
//
// Inputs           : (none)
// Returns          : void
//
// ----------------------------------------------------------------------------
// Notes:
//     - Should be called from the Arduino's loop() method every [tickInterval]
//       seconds (or modified to keep track of the intervals between calls)
// =============================================================================
void WellController::tick() {
  if (state == RUNNING) {
    // TO DO: save current time in millis for a more calibrated delay at the
    // end of this method

    // Read the sensors
    readSensors();
  
    // Check the drive voltage and current to make sure they're within spec
    /*
    LogToSerial(DEBUG, "%CONTROLLER-D-SENSOR, drive voltage of "  + String(driveVoltage) + " V");
    if (driveVoltage < s.DriveVoltageThreshold) {
      if (vp->state != VacuumPump::STANDBY) {
        LogToSerial(INFO, "%VACUUMPUMP-I-STATE, entering STANDBY after " + String(vp->timer) + " seconds due to low drive voltage (" + String(driveVoltage) + " V, which is below threshold of " + String(s.DriveVoltageThreshold) + "V)");
        vp->state = VacuumPump::STANDBY;
      }
      if (dp->state != DistributionPump::STANDBY) {
        LogToSerial(INFO, "%DISTPUMP-I-STATE, entering STANDBY after " + String(dp->timer) + " seconds due to low drive voltage (" + String(driveVoltage) + " V, which is below threshold of " + String(s.DriveVoltageThreshold) + "V)");
        dp->state = DistributionPump::STANDBY;
      }
    }

    LogToSerial(DEBUG, "%CONTROLLER-D-SENSOR, drive current of "  + String(driveCurrent) + " A");
    if (driveCurrent >= s.DriveCurrentThreshold) {
      if (vp->state != VacuumPump::STANDBY) {
        LogToSerial(INFO, "%VACUUMPUMP-I-STATE, entering STANDBY after " + String(vp->timer) + " seconds due to over current condition (" + String(driveCurrent) + ", which is above the threshold of " + String(s.DriveCurrentThreshold) + "A");
        vp->state = VacuumPump::STANDBY;
      }
      if (dp->state != DistributionPump::STANDBY) {
        LogToSerial(INFO, "%DISTPUMP-I-STATE, entering STANDBY after " + String(dp->timer) + " seconds due to over current condition (" + String(driveCurrent) + ", which is above the threshold of " + String(s.DriveCurrentThreshold) + "A");
        dp->state = DistributionPump::STANDBY;
      }      
    }
    */
  
    // Update the state machines
    updateVacuumPumpStateMachine(); 
    vp->timer += tickInterval;

    updateDistributionPumpStateMachine(); 
    dp->timer += tickInterval;

    updateLCD();
  }

  // Serial.print(".");
}






// =============================================================================
// Arduino-specific code starts here
// =============================================================================
const float TickInterval = 1.0; // in seconds
VacuumPump vp;
DistributionPump dp;
WellController controller(vp, dp, TickInterval);




// =============================================================================
// Function         : setup()
//
// Note             : This is the standard Arduino-defined setup function
// =============================================================================
void setup() {
  Serial.begin(38400);

  LogToSerial(INFO, F("%CONTROLLER-I-STATE, going from UNINITIALISED to RUNNING"));
  controller.state = WellController::RUNNING;

  // Set the State Machines' initial states
  controller.readSensors();
  if (vp.cisternSensor) {
    vp.state = VacuumPump::STANDBY;
    LogToSerial(VERBOSE, F("%VACUUMPUMP-V-STATE, cistern is full"));
    LogToSerial(INFO,    F("%VACUUMPUMP-I-STATE, going from UNINITIALISED to STANDBY"));
  } else {
    vp.state = VacuumPump::PRIMING;
    LogToSerial(INFO, F("%VACUUMPUMP-I-STATE, going from UNINITIALISED to PRIMING"));
  }
    
  dp.state = DistributionPump::STANDBY;
  LogToSerial(INFO, F("%DISTPUMP-I-STATE, going from UNINITIALISED to STANDBY"));
}


// =============================================================================
// Function         : loop()
//
// Note             : This is the standard Arduino-defined loop function
// =============================================================================
void loop() {
  controller.tick(); 
   
  // should update this to take the /actual/ tick duration into account
  delay(TickInterval * 1000);
}
