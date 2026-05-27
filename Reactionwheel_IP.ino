#include <SimpleFOC.h>

// ===================== PID BALANCE =====================
struct PID {
  float Kp;
  float Ki;
  float Kd;
  float prevError;
  float integral;
  float outputLimit;
  float derivativeFilterAlpha; // 0 = no derivative, 1 = full smoothing
  float prevDerivative;
  unsigned long lastTime;
};


PID pidAngle;
PID pidSpeed;

void pidInit(PID &pid, float Kp, float Ki, float Kd, float outputLimit, float derivativeFilterAlpha = 0.0f) {
  pid.Kp = Kp;
  pid.Ki = Ki;
  pid.Kd = Kd;
  pid.prevError = 0.0f;
  pid.integral = 0.0f;
  pid.outputLimit = outputLimit;
  pid.derivativeFilterAlpha = derivativeFilterAlpha;
  pid.prevDerivative = 0.0f;
  pid.lastTime = millis();
}

float pidUpdate(PID &pid, float setpoint, float measurement) {
  unsigned long now = millis();
  float dt = (now - pid.lastTime) / 1000.0f;
  if (dt <= 0.0f) dt = 0.001f;
  pid.lastTime = now;

  // Error
  float error = setpoint - measurement;

  // Proportional
  float Pout = pid.Kp * error;

  // Integral with anti-windup
  pid.integral += error * dt;
  float Iout = pid.Ki * pid.integral;

  // Derivative (on error)
  float derivative = (error - pid.prevError) / dt;
  pid.prevError = error;

  // Low-pass filter derivative if needed
  derivative = pid.derivativeFilterAlpha * pid.prevDerivative +
               (1.0f - pid.derivativeFilterAlpha) * derivative;
  pid.prevDerivative = derivative;

  float Dout = pid.Kd * derivative;

  // Total output
  float output = Pout + Iout + Dout;

  // Clamp
  if (output > pid.outputLimit) output = pid.outputLimit;
  else if (output < -pid.outputLimit) output = -pid.outputLimit;

  // Anti-windup correction
  if ((output >= pid.outputLimit && error > 0) ||
      (output <= -pid.outputLimit && error < 0)) {
    pid.integral -= error * dt;
  }

  return output;
}



// ===================== LIMITS =====================
float angleLimit = 1.5 * PI;  // kill switch if tipped too far

// ===================== BLDC Motor and Driver =====================
BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);

// Motor encoder (reaction wheel shaft)
MagneticSensorSPI encoder = MagneticSensorSPI(10, 14, 0xFFFF); // AS5048A

// Pendulum encoder
Encoder pendulum = Encoder(2, 3, 400); 
void doA(){ pendulum.handleA(); }
void doB(){ pendulum.handleB(); }

// ===================== PHYSICAL PARAMETERS =====================
float g = 9.81;        // gravity (m/s^2)
float J_p = 0.0016226898508; // from cad
float V_max = 12.0;    // max motor voltage

// Control gains
float k_su = 10.0;     // swing-up gain
// LQR gains
float k_lqr[3] = {100.0, 0, 0}; // [angle, pendulum vel, wheel vel]

// ===================== SETUP =====================
void setup() {

  Serial.begin(115200);

  // Kp, Ki, Kd, outputLimit (V), derivative filter alpha (0=no filtering, 0.8=smooth)
  pidInit(pidAngle, 150.0f, 0.0f, 4.0f, 12.0f, 0.8f);
  pidInit(pidSpeed, 0.0005f, 0.0000004f, 0.00001f, 0.05f, 0.8f);

  pinMode(23, OUTPUT);
  digitalWrite(23, LOW);

  // Init motor encoder
  encoder.init();

  // Init pendulum encoder
  pendulum.quadrature = Quadrature::ON;
  pendulum.pullup = Pullup::USE_EXTERN;
  pendulum.init();
  pendulum.enableInterrupts(doA, doB);

  // Motor control type
  motor.controller = MotionControlType::torque;
  motor.linkSensor(&encoder);

  // Driver
  driver.voltage_power_supply = 12; 
  motor.voltage_limit = 12;
  driver.init();
  motor.linkDriver(&driver);

  // Init motor and FOC
  motor.init();
  motor.initFOC();
}

// ===================== VELOCITY FILTERING =====================
float alpha = 1; // smoothing factor (0 = no update, 1 = no smoothing)
float filtered_pendulum_vel = 0;
float filtered_pendulum_vel_prev = 0;

// ===================== MAIN LOOP =====================
long loop_count = 0;


void loop() {
  motor.loopFOC();

  // Run control loop ~ every 25 ms
  if(loop_count++ > 5){
    pendulum.update();

    // Shift so 0 rad = upright, ±PI = hanging down
    float pendulum_angle = constrainAngle(pendulum.getAngle() - M_PI);

    //Serial.println(pendulum.getAngle());

    float target_voltage;
    if (abs(pendulum_angle) < 0.5) { 
      //filter pendulumvelocity for less jitter
      filtered_pendulum_vel_prev = filtered_pendulum_vel;
      filtered_pendulum_vel = alpha * pendulum.getVelocity() + (1 - alpha) * filtered_pendulum_vel_prev;

      
      // LQR balance
      //target_voltage = controllerLQR(pendulum_angle, filtered_pendulum_vel, motor.shaftVelocity());//pendulum.getVelocity(), 
      float target_voltage_1 = pidUpdate(pidSpeed, 0.0f, motor.shaftVelocity());
      //Serial.println(target_voltage_1);
      // Inner loop - wheel velocity control
      target_voltage = pidUpdate(pidAngle, - target_voltage_1, pendulum_angle); // target is 0 rad (upright) //pendulum.getVelocity(), 
    } 
    else {
      // Swing-up
      target_voltage = swingUpControl(pendulum_angle, pendulum.getVelocity());
      //Serial.println(target_voltage);
    }

    // Kill switch
    if (abs(pendulum.getAngle()) > angleLimit) {
      motor.move(0);
      while (true) delay(100);
    }

    motor.move(target_voltage);
    loop_count = 0;
  }
}

// ===================== HELPER FUNCTIONS =====================
float constrainAngle(float x){
  x = fmod(x + M_PI, _2PI);
  if (x < 0) x += _2PI;
  return x - M_PI;
}

float sign(float x) {
  return (x > 0) - (x < 0);
}

// ===================== ENERGY SWING-UP =====================
float pendulumEnergy(float theta, float theta_dot) {
  // theta = 0 at upright, ±PI at hanging down
  float theta_norm = fmod(theta + M_PI, _2PI) - M_PI; // normalize to (-PI, PI]
  //Serial.println(theta_norm);
  // Potential energy: zero at upright (cos(0) = 1)
  // Kinetic energy: 0.5 * J_p * ω²
  return 0.5f * J_p * theta_dot * theta_dot 
       + 0.170 * g * 0.1 * (1.0f - cos(theta_norm));
}

float swingUpControl(float theta, float theta_dot) {
  // Current mechanical energy
  float E = pendulumEnergy(theta, theta_dot);

  // Target energy for upright position
  float E_target = 0.0f; // zero at upright

  // Energy-based swing-up torque/voltage
  // sign(theta_dot * cos(theta)) determines direction to push
  float u = k_su * (E - E_target) * ((theta_dot * cos(theta) >= 0) ? 1.0f : -1.0f);

  //Serial.println(u);

  // Saturate at 60% of max motor voltage
  if (u > 0.4f * V_max) u = 0.4f * V_max;
  if (u < -0.4f * V_max) u = -0.4f * V_max;

  if(abs(theta) > 2.9)
    u = 0;
  return -u;
}

// ===================== LQR BALANCE =====================
float controllerLQR(float p_angle, float p_vel, float m_vel){
  float u = k_lqr[0]*p_angle + k_lqr[1]*p_vel + k_lqr[2]*m_vel;
  if (abs(u) > motor.voltage_limit * 0.7) 
    u = sign(u) * motor.voltage_limit* 0.7;
  return -u;
}

