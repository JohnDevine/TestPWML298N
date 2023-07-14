#include "BTS7960.h" // Include the header file

// Constructor definition
BTS7960::BTS7960()
{
}

void BTS7960::begin(int pwmPin1, int pwmPin2, int pwmFrequency, int minDutyCycle, int stopSwitch1Pin, int stopSwitch2Pin, float maxCurrent)
{
    // Initialize the data members with the given arguments
    _pwmPin1 = pwmPin1;
    _pwmPin2 = pwmPin2;
    _pwmFrequency = pwmFrequency;
    _minDutyCycle = minDutyCycle;
    _beginSwitchPin = stopSwitch1Pin;
    _endSwitchPin = stopSwitch2Pin;
    _maxCurrent = maxCurrent;

    // Set the PWM pins as outputs and the switch pins and current pin as inputs
    pinMode(_pwmPin1, OUTPUT);
    pinMode(_pwmPin2, OUTPUT);

    // Set the PWM frequency for both pins
    setPwmFrequency(_pwmFrequency);

    // set up the stop switches
    // initialize the pushbutton pin as an pull-up input
    // the pull-up input pin will be HIGH when the switch is open and LOW when the switch is closed.
    _beginSwitch = Bounce();                            // Instantiate a Bounce object
    _endSwitch = Bounce();                            // Instantiate a Bounce object
    _beginSwitch.attach(_beginSwitchPin, INPUT_PULLUP); // USE INTERNAL PULL-UP
    _beginSwitch.interval(5);                           // interval in ms
    _endSwitch.attach(_endSwitchPin, INPUT_PULLUP); // USE INTERNAL PULL-UP
    _endSwitch.interval(5);                           // interval in ms

    // set up the current sensor
    if (!_currentSensor.begin())
    {
        Serial.println("Failed to find INA219 chip");
        while (1)
        {
            delay(10);
        }
    }
    // Need to wait for the current to come good
    /*
    _waitForCurrentSensor = 0;
    while (_waitForCurrentSensor < 2000) // Loop for up to 2 seconds
    {
        displayCurrent();        // Display the current
        if (getCurrent() < 2000) // If current < 2000mw keep trucking
        {
            break;
        }
    }
    */
    resetAcceleration();
    _currentDutyCycle = 0;
    _requestedDutyCycle = 0;
    _motorDirection = motorDirection::unknown;
}

// Member function definitions

void BTS7960::setPwmFrequency(int frequency)
{
    // Set the PWM frequency for both pins using analogWriteFreq()
    analogWriteFreq(frequency);
}

void BTS7960::forward(int requestedDutyCycle, long time, long accelDuration, typeOfAcceleration accelType)
{
    setAcceleration(accelDuration, accelType);
    _currentDutyCycle = requestedDutyCycle;
    _requestedDutyCycle = requestedDutyCycle;
    // Drive forward with a given speed by setting one pin HIGH and one pin LOW
    analogWrite(_pwmPin1, _currentDutyCycle);
    analogWrite(_pwmPin2, LOW);
    _motorDirection = motorDirection::forward;
    _motorRequestedDuration = time; // this is how long we request the motor to run for in ms.
    _motorRunTimeTillNow = 0;       // start the timer
}

void BTS7960::run()
{
    float retrievedCurrent;
    int retrievedDutyCycle;
    retrievedCurrent = getCurrent();
    // Update the stop switches
    _beginSwitch.update(); // Update the Bounce instance (YOU MUST DO THIS EVERY LOOP)
    _endSwitch.update(); // Update the Bounce instance (YOU MUST DO THIS EVERY LOOP)
    if (_motorRequestedDuration == 0)
    {
        return;
    }
    // Run the motor doing acceleration, deacceleration and stopping at requested time.
    // This function should be called in the loop() function.
    // This function will check if the motor has been running for the requested time and stop it if it has.
    // It will also check if the current has exceeded the maximum current and stop it if it has.
    // It will also check if the stop switches have been pressed and stop it if they have.
    if (_motorRunTimeTillNow >= _motorRequestedDuration)
    {
        stop();
        return;
    }

    else if (retrievedCurrent > _maxCurrent)
    {

        stop();
        return;
    }
    else if (_beginSwitch.fell() && _motorDirection == motorDirection::backward)
    {
        stop();
        return;
    }
    else if (_endSwitch.fell() && _motorDirection == motorDirection::forward)
    {
        stop();
        return;
    }
    // See if duty cycle has changed (under acceleration)
    retrievedDutyCycle = calculateDutyCycle();

    if (retrievedDutyCycle != _currentDutyCycle)
    {
        if (_motorDirection == motorDirection::forward)
        {
            analogWrite(_pwmPin1, retrievedDutyCycle);
            analogWrite(_pwmPin2, LOW);
        }
        else if (_motorDirection == motorDirection::backward)
        {
            analogWrite(_pwmPin2, retrievedDutyCycle);
            analogWrite(_pwmPin1, LOW);
        }
        _currentDutyCycle = retrievedDutyCycle;
    }
}
void BTS7960::stop()
{
    analogWrite(_pwmPin1, LOW);
    analogWrite(_pwmPin2, LOW);
    _motorRequestedDuration = 0;
    resetAcceleration();
    _currentDutyCycle = 0;
    _requestedDutyCycle = 0;
}

void BTS7960::backward(int requestedDutyCycle, long time, long accelDuration, typeOfAcceleration accelType)
{
    setAcceleration(accelDuration, accelType);
    _currentDutyCycle = requestedDutyCycle;
    _requestedDutyCycle = requestedDutyCycle;
    // Drive backward with a given speed by setting one pin HIGH and one pin LOW
    analogWrite(_pwmPin2, _currentDutyCycle);
    analogWrite(_pwmPin1, LOW);
    _motorDirection = motorDirection::backward;
    _motorRequestedDuration = time; // this is how long we request the motor to run for in ms.
    _motorRunTimeTillNow = 0;       // start the timer
}

float BTS7960::getCurrent()
{
    float currentMilliAmps;
    currentMilliAmps = _currentSensor.getCurrent_mA();
    return currentMilliAmps;
}
void BTS7960::displayCurrent()
{

    float shuntvoltage = 0;
    float busvoltage = 0;
    float current_mA = 0;
    float loadvoltage = 0;
    float power_mW = 0;
    char buffer[200];

    shuntvoltage = _currentSensor.getShuntVoltage_mV();
    busvoltage = _currentSensor.getBusVoltage_V();
    current_mA = _currentSensor.getCurrent_mA();
    power_mW = _currentSensor.getPower_mW();
    loadvoltage = busvoltage + (shuntvoltage / 1000);

    // Use sprintf to print the shuntvoltage, busvoltage, current_mA, power_mW and loadvoltage to the buffer

    sprintf(buffer, "shuntV:   %3.2f , BusV: %3.2f ,LoadV: %3.2f, Current_mA: %3.2f ,Power mW: %3.2f  \n", shuntvoltage, busvoltage, loadvoltage, current_mA, power_mW);

    Serial.print(buffer);
}
bool BTS7960::setAcceleration(long accelDuration, typeOfAcceleration accelType)
{
    // Set the acceleration duration and type
    // The acceleration duration is the time in ms to go from 0 to full speed
    // The acceleration type is either none, rampUp, rampDown or rampUpDown
    // The function returns true if the acceleration duration is valid and false if it is not
    //
    resetAcceleration();
    if (accelDuration <= 0 || accelType == typeOfAcceleration::none)
    {
        return false;
    }
    _accelDuration = accelDuration;
    _accelType = accelType;
    return true;
}
void BTS7960::resetAcceleration()
{
    // Reset the acceleration duration and type to 0 and linear
    _accelDuration = 0;
    _accelType = typeOfAcceleration::none;
}
int BTS7960::calculateDutyCycle()
{

    // Calculate the current duty cycle based on the acceleration type and duration
    // The function returns the new duty cycle

    // If the acceleration duration is 0 or there is no acceleration then return the current duty cycle
    if (_accelDuration == 0 || _accelType == typeOfAcceleration::none)
    {
        return _currentDutyCycle;
    }

    /* Calculate the ramp.
    For a rampup
    The ramp starts at _minDutyCycle and goes till _requestedDutyCycle
    It does this from 0 to _accelDuration
    The calculation then is _minDutyCycle +
    (_requestedDutyCycle - _minDutyCycle) * (_motorRunTimeTillNow / _accelDuration)

    */
    if (_accelType == typeOfAcceleration::rampUp)
    {
        return calculateBeginningDutyCycle();
    }
    if (_accelType == typeOfAcceleration::rampDown)
    {
        return calculateEndingDutyCycle();
    }

    if (_accelType == typeOfAcceleration::rampUpDown)
    {

        if (_motorRunTimeTillNow <= _accelDuration)
        {
            // See if we are in the beginning acceleration time and do the beginning logic.
            return calculateBeginningDutyCycle();
        }
        else
        {
            // See if we are in the ending acceleration time and do the ending logic.
            return calculateEndingDutyCycle();
        }
    }

    // Just a catch all here.
    return _requestedDutyCycle;
}
int BTS7960::calculateBeginningDutyCycle()
{
    int internalCalculatedDutyCycle;
    // code to calculate forward duty cycle
    // Are we outside the acceleration area go back to requested duty cycle
    if (_motorRunTimeTillNow >= _accelDuration)
    {
        return _requestedDutyCycle;
    }

    internalCalculatedDutyCycle = (_motorRunTimeTillNow * _requestedDutyCycle) / _accelDuration;
    if (internalCalculatedDutyCycle < _minDutyCycle)
    {
        internalCalculatedDutyCycle = _minDutyCycle;
    }

    return internalCalculatedDutyCycle;
}

int BTS7960::calculateEndingDutyCycle()
{
    int internalCalculatedDutyCycle;
    float internalTimeLeftForRampDown;
    // float internalStartRampDown;

    // code to calculate backward duty cycle
    // Are we outside the deacceleration area go back to requested duty cycle
    if (_motorRunTimeTillNow <= (_motorRequestedDuration - _accelDuration))
    {
        return _requestedDutyCycle;
    }

    // internalStartRampDown = _motorRequestedDuration - _accelDuration;
    internalTimeLeftForRampDown = _motorRequestedDuration - _motorRunTimeTillNow;
    internalCalculatedDutyCycle = int((internalTimeLeftForRampDown / _accelDuration) * _requestedDutyCycle);

    if (internalCalculatedDutyCycle < _minDutyCycle)
    {
        internalCalculatedDutyCycle = _minDutyCycle;
    }
    return internalCalculatedDutyCycle;
}

bool BTS7960::motorIsRunning()
{
    // Return true if the motor is running and false if it is not
    if (_currentDutyCycle == 0)
    {
        return false;
    }
    return true;
}