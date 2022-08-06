  class TBMotorController
{
protected:
    int _ena, _in1, _in2, _enb, _in3, _in4;
    int _currentSpeed;
    double _motorAConst, _motorBConst;
public:
    TBMotorController(int ena, int in1, int in2, int enb, int in3, int in4, double motorAConst, double motorBConst);
    void move(int leftSpeed, int rightSpeed, int minAbsSpeed);
    void move(int speed);
    int move(int speed, int minAbsSpeed);
    void turnLeft(int speed, bool kick);
    void turnRight(int speed, bool kick);
    void stopMoving();
};

TBMotorController::TBMotorController(int ena, int in1, int in2, int enb, int in3, int in4, double motorAConst, double motorBConst)
{
    _motorAConst = motorAConst;
    _motorBConst = motorBConst;
    
	_ena = ena;
	_in1 = in1;
	_in2 = in2;
	_enb = enb;
	_in3 = in3;
	_in4 = in4;
	
	pinMode(_ena, OUTPUT);
	pinMode(_in1, OUTPUT);
	pinMode(_in2, OUTPUT);
    
	pinMode(_enb, OUTPUT);
	pinMode(_in3, OUTPUT);
	pinMode(_in4, OUTPUT);
}

int TBMotorController::move(int speed, int minAbsSpeed)
{
    int direction = 1;
    
    if (speed < 0)
    {
        direction = -1;
        
        speed = min(speed, -1*minAbsSpeed);
        speed = max(speed, -255);
    }
    else
    {
        speed = max(speed, minAbsSpeed);
        speed = min(speed, 255);
    }
    
    // if (speed == _currentSpeed) return -1;
    
    int realSpeed = max(minAbsSpeed, abs(speed));
    digitalWrite(_in1, speed > 0 ? HIGH : LOW);
    digitalWrite(_in2, speed > 0 ? LOW : HIGH);
    digitalWrite(_in3, speed > 0 ? HIGH : LOW);
    digitalWrite(_in4, speed > 0 ? LOW : HIGH);
    analogWrite(_ena, realSpeed * _motorAConst);
    analogWrite(_enb, realSpeed * _motorBConst);
    
    _currentSpeed = direction * realSpeed;
    return realSpeed;
}


void TBMotorController::move(int speed)
{
    if (speed == _currentSpeed) return;
    
    if (speed > 255) speed = 255;
    else if (speed < -255) speed = -255;
    
    digitalWrite(_in1, speed > 0 ? HIGH : LOW);
    digitalWrite(_in2, speed > 0 ? LOW : HIGH);
    digitalWrite(_in3, speed > 0 ? HIGH : LOW);
    digitalWrite(_in4, speed > 0 ? LOW : HIGH);
    analogWrite(_ena, abs(speed) * _motorAConst);
    analogWrite(_enb, abs(speed) * _motorBConst);
    
    _currentSpeed = speed;
}

void TBMotorController::stopMoving()
{
    digitalWrite(_in1, LOW);
    digitalWrite(_in2, LOW);
    digitalWrite(_in3, LOW);
    digitalWrite(_in4, LOW);
    digitalWrite(_ena, HIGH);
    digitalWrite(_enb, HIGH);
    
    _currentSpeed = 0;
}
