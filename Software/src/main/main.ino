#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <AFMotor.h>
#include <simpleFusion.h>


#include <math.h>

SimpleFusion fuser; 
Adafruit_MPU6050 mpu;

// sensor setup
sensors_event_t a, g, temp;

// time difference
unsigned long prevMillis = 0.0;
unsigned long currMillis = 0.0;
double dt;

typedef struct {
  float Kp, Ki, Kd;
  float integral;
  float prev_error;
  float out_min, out_max;
} PID_t;


// Initialize PID constants
PID_t pitchPID = {
  .Kp = 1.0f, .Ki = 0.0f, .Kd = 0.0f,
  .integral = 0.0f, .prev_error = 0.0f,
  .out_min = -255.0f, .out_max = 255.0f
};

// velocity setup
double prevVelocity;

// motor setup
AF_DCMotor motor1(1);
AF_DCMotor motor2(4);

// Constants
int STALL_SPEED = 40.0;

void serialEvent();
float pid_compute(PID_t *pid, float setpoint, float measurement, float dt);
int accel_or_deccel(double velocity, double prev_velocity);
void dir_motor(double velocity); 
void accel_motors(double speed);
void deaccel_motors(double speed, double prev_speed);
// void applyMPUOffsets();

void setup() {

  Serial.begin(115200);

  /*!
 *    @brief Initializes filter parameters. 
 *    @param filterUpdateRate The frequency of filter updates up to 1000000 Hertz
 *    @param pitchGyroFavoring The amount that the gyroscope is favored in the pitch direction as a decimal percent less than 1
 * 		@param rollGyroFavoring The amount that the gyroscope is favored in the roll direction as a decimal percent less than 1 
 *		@returns false if any gyro favoring is an invalid value, true if they are valid.
  */
  fuser.init(100, 0.98, 0.98);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("MPU6050 Found!");
  
  // Apply offset
  // applyMPUOffsets();

  // Initialize the motor
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  prevVelocity = 0.0;

  // MPU Configuration
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  

  delay(100);
}

void MPU_sensor_readings(sensors_event_t a, sensors_event_t g, sensors_event_t temp) {
  Serial.print("Acceleration X: ");
	Serial.print(a.acceleration.x);
	Serial.print(", Y: ");
	Serial.print(a.acceleration.y);
	Serial.print(", Z: ");
	Serial.print(a.acceleration.z);
	Serial.println(" m/s^2");

	Serial.print("Rotation X: ");
	Serial.print(g.gyro.x);
	Serial.print(", Y: ");
	Serial.print(g.gyro.y);
	Serial.print(", Z: ");
	Serial.print(g.gyro.z);
	Serial.println(" rad/s");

	Serial.print("Temperature: ");
	Serial.print(temp.temperature);
	Serial.println(" degC");

	Serial.println("");
	
}

void loop() {

  // serialEvent(&pitchPID);

  if (fuser.shouldUpdateData()) {

    FusedAngles fusedAngles;

    currMillis = millis();
    dt = (currMillis - prevMillis)/1000.0; // converts ms to seconds

    // MPU sensor values
    mpu.getEvent(&a, &g, &temp);

    // MPU_sensor_readings(a, g, temp);
    // MPU_calibrate(a, g);

    ThreeAxis accel;
    accel.x = a.acceleration.x;
    accel.y = a.acceleration.y;
    accel.z = a.acceleration.z;

    ThreeAxis gyro;
    gyro.x = g.gyro.x;
    gyro.y = g.gyro.y;
    gyro.z = g.gyro.z;

    fuser.getFilteredAngles(accel, gyro, &fusedAngles, UNIT_DEGREES);

    Serial.print(" Pitch: ");
    Serial.println(fusedAngles.pitch);
    // Serial.print(" Roll : ");
    // Serial.println(fusedAngles.roll);

    double con_out = pid_compute(&pitchPID, 0.0, fusedAngles.pitch, dt);
    if (con_out > 0){
      con_out = speed_offset(con_out);
    } else {
      con_out =  -1.0 * speed_offset(abs(con_out));
    }
    Serial.print("PID output ");
    Serial.println(con_out);


    // Set the motor direction
    dir_motor(con_out);

    // Accelerate or Decelerate the motor
    accel_or_deccel(con_out, prevVelocity);

    prevVelocity = con_out;
    prevMillis = currMillis;

    // while (fusedAngles.pitch > 5.0 || fusedAngles.pitch < -5.0) {
    //   motor1.run(RELEASE);
    //   motor2.run(RELEASE);
    // }
  }

}

void serialEvent(PID_t *pid) {
  String cmd = Serial.readStringUntil('\n');
  if (cmd.startsWith("P=")) pid->Kp = cmd.substring(2).toFloat();
  else if (cmd.startsWith("I=")) pid->Ki = cmd.substring(2).toFloat();
  else if (cmd.startsWith("D=")) pid->Kd = cmd.substring(2).toFloat();
}

void MPU_calibrate(sensors_event_t &a, sensors_event_t &g) {
  // Replace with your calibration offsets (raw values from the sketch)
  const int16_t axOff = 1252;
  const int16_t ayOff = -2506;
  const int16_t azOff = 1118;
  const int16_t gxOff = -83;
  const int16_t gyOff = -90;
  const int16_t gzOff = -16;

  // Convert from raw LSB to physical units:
  // MPU6050_RANGE_8_G → 4096 LSB/g, 1 g ≈ 9.80665 m/s²
  const float accelLSB = 9.80665f / 4096.0f;

  // MPU6050_RANGE_500_DEG → 65.5 LSB/deg/s → ~0.00106 rad/s per LSB
  const float gyroLSB = 500.0f / 32768.0f * (PI / 180.0f); // in rad/s per LSB

  // Remove zero error (biases) from accelerometer
  a.acceleration.x -= axOff * accelLSB;
  a.acceleration.y -= ayOff * accelLSB;
  a.acceleration.z -= azOff * accelLSB;

  // Remove zero error (biases) from gyroscope
  g.gyro.x -= gxOff * gyroLSB;
  g.gyro.y -= gyOff * gyroLSB;
  g.gyro.z -= gzOff * gyroLSB;
}

void antiwindup(PID_t *pid, float derivative) {
  if (pid->integral * pid->Ki > pid->out_max)
        pid->integral = pid->out_max / pid->Ki;
  else if (pid->integral * pid->Ki < pid->out_min)
        pid->integral = pid->out_min / pid->Ki;
}

float pid_compute(PID_t *pid, float setpoint, float measurement, float dt) {
  float error = setpoint - measurement;
  pid->integral += error * dt;
  float derivative = (error-pid->prev_error)/dt;

  antiwindup(pid, derivative);

  float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

  pid->prev_error = error;

  return output;
  
}

int accel_or_deccel(double velocity, double prev_velocity) {
  if (velocity > prev_velocity) {
    accel_motors(abs(velocity), abs(prev_velocity));
  } else {
    deaccel_motors(abs(velocity), abs(prev_velocity));
  }

}

double speed_offset(double i) {
    if (i >= 30.0) {
      return i;  
    } else  if (i > 0.0) {
      return STALL_SPEED;
    } else {
      return 0.0;
    }
}

void dir_motor(double velocity) {
  if (velocity > 0) {
    motor1.run(FORWARD);
    motor2.run(FORWARD);
    Serial.println("Dir: Forward");
  } else if (velocity < 0) {
    motor1.run(BACKWARD);
    motor2.run(BACKWARD);
    Serial.println("Dir: Backward");
  } else {
    motor1.run(RELEASE);
    motor2.run(RELEASE);
    Serial.println("Dir: Release");
  }
}

void accel_motors(double speed, double prev_speed) {
  Serial.println("Accelerating");
  speed = speed_offset(speed);
  prev_speed = speed_offset(prev_speed);
  for (double i = prev_speed; i <= speed; i++) {
    motor1.setSpeed(speed_offset(i));
    motor2.setSpeed(speed_offset(i));  
    delay(10);
  }
}

void deaccel_motors(double speed, double prev_speed) {
  Serial.println("Deccelerating");
    speed = speed_offset(speed);
    prev_speed = speed_offset(prev_speed);
  for (double i = prev_speed; i >= speed; i--) {
    motor1.setSpeed(speed_offset(i));
    motor2.setSpeed(speed_offset(i));  
    delay(10);
	}
}
