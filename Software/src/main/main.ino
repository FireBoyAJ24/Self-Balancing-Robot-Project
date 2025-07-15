#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include <simpleFusion.h>

SimpleFusion fuser; 
Adafruit_MPU6050 mpu;

double pid_const[] = {0.0, 0.0, 0.0};

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

float pid_compute(PID_t *pid, float setpoint, float measurement, float dt);

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

  PID_t pitchPID = {
    .Kp = 25.0f, .Ki = 1.2f, .Kd = 0.8f,
    .integral = 0.0f, .prev_error = 0.0f,
    .out_min = -255.0f, .out_max = 255.0f
  };

  if (fuser.shouldUpdateData()) {

    FusedAngles fusedAngles;

    currMillis = millis();
    dt = (currMillis - prevMillis)/1000.0; // converts ms to seconds

    // MPU sensor values
    mpu.getEvent(&a, &g, &temp);

    MPU_sensor_readings(a, g, temp);

    ThreeAxis accel;
    accel.x = a.acceleration.x;
    accel.y = a.acceleration.y;
    accel.z = a.acceleration.z;

    ThreeAxis gyro;
    gyro.x = g.gyro.x;
    gyro.y = g.gyro.y;
    gyro.z = g.gyro.z;

    fuser.getFilteredAngles(accel, gyro, &fusedAngles, UNIT_DEGREES);

    float con_out = pid_compute(&pitchPID, 0.0, fusedAngles.pitch, dt);

    Serial.print(" Pitch: ");
    Serial.print(fusedAngles.pitch);
    Serial.print(" Roll : ");
    Serial.println(fusedAngles.roll);
    
    prevMillis = currMillis;
  }

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
