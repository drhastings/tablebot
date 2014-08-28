#include <uart.h>
#include <I2Cdev.h>
#include <mpu6050.h>
#include <millis.h>
#include <dmp.h>
#include <motors.h>
#include <helper_3dmath.h>

volatile uint8_t interrupt = 0;

volatile uint8_t results[4];
uint8_t result_index;

uint8_t settling = 255;
uint8_t stopped = 1;

char input_buffer[16];

float target_heading, last_heading;
float new_target;

uint8_t scanning = 0;
float scan_to;
int8_t scan_direction;

uint8_t shortest = 255;
uint8_t last_reading;

float near, far;

float table_edge, last_edge;
uint8_t found_an_edge = 0, ready_to_scan = 0;

uint8_t turning;
uint16_t reversing;

float target_rotational_velocity; // in radians per hundreth.

int16_t target_speed = 0;

float headingP = 500;
float headingD = 7000;
float headingI = 5;

float Ierror;

float integrated_theta_error, last_theta_error;

Quaternion orientation, inv, point;

void initADC();

void quaternion_product(Quaternion *result, Quaternion * p,
      Quaternion *q);

void get_conjugate(Quaternion *result, Quaternion *q);

float normalized_difference(const float angle_a, const float angle_b);

int main()
{
  // Start serial
  initUART(10);
  // Start mpu
  initMPU();
  // Start system timer
  initMillis();
  // Start adc to fill results array global variable
  initADC();
  // Start PWM timers
  initMotors();

  // Configgure interrupt on INT0 from mpu
  EICRA = 1<<ISC01 | 1<<ISC00;
  EIMSK = 1<<INT0;

  // Start Interrupts
  sei();

  // load dmp firmware
  dmpInitialize();

  // so we can tell how much time has elapsed later
  int now = millis();

  send("Initialized\n", 12);

  uint8_t fifo_bytes[42];

  uint16_t count = 0;

  // start dmp
  setDMPEnabled(1);

  while(1)
  {
    // Waits for sample
    if (interrupt || count > 42)
    {
      interrupt = 0;
      uint8_t status = getIntStatus();

      count = getFIFOCount();

      // If interrupt is due to overflow
      if (status & 0x10)
      {
        resetFIFO();
      }

      // get mpu data
      getFIFOBytes(fifo_bytes, 42);

      // unit vector in forward direction as purely imaginary quaternion
      point.w = 0;
      point.x = 0;
      point.y = 1;
      point.z = 0;

      // get quaternion from mpu data
      dmpGetQuaternion(&orientation, fifo_bytes);

      // get the inverse (conjugate since its normalized)
      get_conjugate(&inv, &orientation);

      // rotate point by quaternion multiplication
      quaternion_product(&point, &orientation, &point);

      quaternion_product(&point, &point, &inv);

      // Get angle off rotated vector around Z axis in world space
      float heading = atan2f(point.x, point.y);

      // read table sensors
      uint8_t back_left = (PINB >> 1) & 1;
      uint8_t back_right = (PINB >> 2) & 1;
      uint8_t front_left = (PINB >> 0) & 1;
      uint8_t front_right = (PIND >> 3) & 1;
      uint8_t front_middle = (PIND >> 7) & 1;
      
      // If there is something in the serial buffer
      if (available())
      {
        // clear the input buffer, fetch a line and parse it
        memset(input_buffer, 0, sizeof(input_buffer));
        getLine(input_buffer);
        switch (input_buffer[0])
        {
          case 'p':
            headingP = atof(&input_buffer[1]);
            send("Setting P to: ", 14);
            sendFloat(headingP);
            sendLn();
            break;
          case 'i':
            headingI = atof(&input_buffer[1]);
            send("Setting I to: ", 14);
            sendFloat(headingI);
            sendLn();
            break;
          case 'd':
            headingD = atof(&input_buffer[1]);
            send("Setting D to: ", 14);
            sendFloat(headingD);
            sendLn();
            break;
          case 's':
            target_speed = atof(&input_buffer[1]);
            send("Setting s to: ", 14);
            sendFloat(target_speed);
            sendLn();
            break;
          case 'r':
            target_rotational_velocity = atof(&input_buffer[1]);
            send("Setting r to: ", 14);
            sendFloat(target_rotational_velocity);
            sendLn();
            break;
          case 'h':
            target_heading = atof(&input_buffer[1]);
            send("Setting h to: ", 14);
            sendFloat(target_heading);
            sendLn();
            break;
          default:
            break;
        }
      }

      // Use sensor input and time elapsed to decide on state
      // ie, rotational velocity and linear velocity.

      // for debugging, send sensor values

/*      sendInt(front_left);
      sendInt(front_middle);
      sendInt(front_right);
      sendInt(back_left);
      sendInt(back_right);
      sendLn();*/

      if (settling)
      {
        if (fabs(normalized_difference(heading, last_heading)) < 0.00001)
        {
          settling--;

          if (!settling)
          {
            send("Ok, settled.\n", 13);
            target_heading = heading;
            stopped = 0;
          }
        }
      }
      else if (!front_left && !front_right && !front_middle)
      {
        target_speed = -120;
        target_rotational_velocity = 0;
      }
      else if (!front_left || results[2] > 170)
      {
        target_speed = -110;
        target_rotational_velocity = -.005;
      }
      else if (!front_right || results[3] > 170)
      {
        target_speed = -110;
        target_rotational_velocity = .005;
      }
      else if (front_right && front_left && !front_middle && !reversing)
      {
        target_speed = 0;
        target_rotational_velocity = 0;
        reversing = 150;
        
        table_edge = normalized_difference(heading, M_PI /2);

        float angle_diff = fabs(normalized_difference(table_edge, last_edge));

        if (!found_an_edge)
        {
          found_an_edge = 1;

          last_edge = table_edge;
        }
        else if (angle_diff > M_PI / 3 && angle_diff < 2 * M_PI / 3)
        {
          ready_to_scan = 1;

          near = 0;

          far = 0;

          found_an_edge = 0;
        }
      }
      else if (reversing)
      {
        target_rotational_velocity = 0;
        target_speed = -150;
        reversing--;
        if (!reversing)
        {
          target_heading = normalized_difference(heading, M_PI / 2);

          if (ready_to_scan)
          {
            ready_to_scan = 0;

            target_heading = normalized_difference(target_heading, -M_PI / 4);

            scan_to = normalized_difference(target_heading, M_PI);

            scan_direction = -1;

            scanning = 1;
          }
        }
      }
      else if (scanning)
      {
        if (fabs(normalized_difference(target_heading, scan_to)) < 0.004)
        {
          if (new_target)
          {
            target_heading = new_target;
          }
          
          new_target = 0;

          shortest = 255;

          scanning = 0;
        }
        else
        {
          target_speed = 70;
          target_rotational_velocity = 0.001 * scan_direction;

//          sendInt(results[0]);
//          sendLn();

          int16_t deriv = results[0] - last_reading;

          if (results[0] < shortest && deriv < -3 && fabs(normalized_difference( heading, target_heading)) < 0.2)
          {
            far = 0;

            near = heading;

            shortest = results[0];

            send("near: ", 6);
            sendFloat(heading);
            sendLn();
          }

          if (!far && near && deriv > 3)
          {
            far = heading;

            send("far: ", 5);
            sendFloat(heading);
            sendLn();
            new_target = (near + far) / 2;
            if (fabs(near - far) > M_PI)
            {
              new_target -= M_PI;
            }
          }

          last_reading = results[0];
        }
      }
      else
      {
        target_speed = 150;
        target_rotational_velocity = 0;
      }

      // Add the velocity to current heading to get new heading
      target_heading += target_rotational_velocity;

      // From here down is the PID loop that keeps that maintains heading
      float heading_error = normalized_difference(heading, target_heading);

      float theta = heading - last_heading;

      float offset;

      if (fabs(heading_error) > 0.006 || (fabs(heading_error) > 0.1 && target_speed == 0))
      {
        offset = headingP * heading_error;

        offset += headingD * theta;

        if (fabs(heading_error) < 0.03)
        {
          Ierror += heading_error;
        }

        offset += headingI * Ierror;
      }
      else
      {
        offset = 0;
      }

      last_heading = heading;

      if (!stopped)
      {
        motora(target_speed - offset);
        motorb(target_speed + offset);
      }
      else
      {
        motora(0);
        motorb(0);
      }
    }
  }
}

ISR (INT0_vect)
{
  interrupt = 1;
}

// Used to find heading from quaternion
void quaternion_product(Quaternion *result, Quaternion * p,
      Quaternion *q)
{
  Quaternion temp;

  temp.w = p->w * q->w - p->x * q->x - p->y * q->y - p->z * q->z;
  temp.x = p->w * q->x + p->x * q->w + p->y * q->z - p->z * q->y;
  temp.y = p->w * q->y + p->y * q->w + p->z * q->x - p->x * q->z;
  temp.z = p->w * q->z + p->z * q->w + p->x * q->y - p->y * q->x;

  *result = temp;
}

void get_conjugate(Quaternion *result, Quaternion *q)
{
  result->w = q->w;
  result->x = -q->x;
  result->y = -q->y;
  result->z = -q->z;
}

// Utility function to get angles normalized to -pi to pi
float normalized_difference(const float angle_a, const float angle_b)
{
  float ret = angle_a - angle_b;

  while (ret > M_PI)
  {
    ret -= 2 * M_PI;
  }

  while (ret < -M_PI)
  {
    ret += 2 * M_PI;
  }

  return ret;
}

void initADC()
{
  ADMUX = (1<<REFS0) | (1<<ADLAR) | 0x0;

  ADCSRA = (1<<ADEN) | (1<<ADSC) | (1<<ADIE) | 0x7;
}

ISR(ADC_vect)
{
  results[result_index] = ADCH;

  result_index++;

  if (result_index > 4)
  {
    result_index = 0;
  }

  ADMUX = (ADMUX & 0xF8) | result_index;

  ADCSRA |= (1<<ADSC);
}
