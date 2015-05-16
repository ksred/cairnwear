/*    
 *   "Arduino NeoPixel Ring Compass"
 *  Written by: David Ratliff  -  August 2013
 *  Released into the public domain.  The code is free for you to use, change, make your own, etc...
 * However, it is NOT WARRANTEED, GUARANTEED, or even promised to work as may be expected.
 *
 * That said...
 *  The following sketch was written for use with the Adafruit NeoPixel Ring and the 
 * Adafruit LSM303DLHC Compass/Accelerometer to create a simple "Compass" which will point North
 * (**Assuming the ring is oriented correctly!)
 *
 *  This sketch is written to use the SCL & SCK pins on the LSM303 Breakout Board
 *  And, communicating with the NeoPixel Ring on Digital Pin 5 of Arduino UNO R.3
 *      (This can of course be easily changed below.)
 *  
 *  This is an extremely simple and quickly written sketch to prove it could be done and make my video.
 *  --- It does not compensate for 3rd axis rotation. If not held rather horizontally...
 *      compass will typically point "down"  -- PLEASE, feel free to correct this & post updates!  :)
 *
 *  --- The methods used below were also heavily influenced by the
 *      "City Bike NeoPixel Helmet" tuorial & code found on learn.adafruit.com
 *
 *    Thank you!
 */


#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
#include <SPI.h>
#include <Adafruit_GPS.h>

// *** Uncomment the following line for debugging - to use serial monitor
#define debug true
#define LAT_LON_SIZE 311

#define PIN 10  // Pin NeoWheel is on


/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM9DS0 compass = Adafruit_LSM9DS0();

// Initiate NeoPixel Wheel
Adafruit_NeoPixel strip = Adafruit_NeoPixel(16, PIN, NEO_GRB + NEO_KHZ800);

// If using hardware serial, comment
// out the above two lines and enable these two lines instead:
Adafruit_GPS GPS(&Serial1);
HardwareSerial mySerial = Serial1;

//  Assign pixels to heading directions

const int NNW = 15;
const int NW = 14;
const int WNW = 13;
const int W = 12;
const int WSW = 11;
const int SW = 10;
const int SSW = 9;
const int S = 8;
const int SSE = 7;
const int sE = 6;
const int ESE = 5;
const int E = 4;
const int ENE = 3;
const int NE = 2;
const int NNE = 1;
const int N = 0;
// variable and initial value for "North" indicator
int indicator = 0;

//  brightness level / RGB value used for LEDs
const byte level = 255;    // valid vallues between 0 - 255... but 0 would be a tad hard to see
const byte levelLow = 160;

float heading;

void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}

void setup(void) 
{

  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  // the following serial functions only used if "#define debug true" at beginning of sketch is UNCOMMENTED 
   while (!Serial); // flora & leonardo
  
  Serial.begin(9600);
  Serial.println("LSM raw read demo");
  
  // Try to initialise and warn if we couldn't detect the chip
  if (!compass.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS0. Check your wiring!");
    while (1);
  }
  Serial.println("Found LSM9DS0 9DOF");
  Serial.println("");
  Serial.println("");
  
  // one cycle of red around the ring to signal we're good to go!
  colorWipe(strip.Color(55, 0, 0), 50); // Wipe RED around LED WHEEL - Verfies magnetometer ready
}

void loop()                     // run over and over again
{
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
      if (c) Serial.print(c);
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
  
  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // every 300 milliseconds, update the heading/distance indicators
  if (millis() - timer > 300) { 
    timer = millis(); // reset the timer
    
    //Serial.print("\nTime: ");
    //Serial.print(GPS.hour, DEC); Serial.print(':');
    //Serial.print(GPS.minute, DEC); Serial.print(':');
    //Serial.print(GPS.seconds, DEC); Serial.print('.');
    //Serial.println(GPS.milliseconds);
    //Serial.print("Date: ");
    //Serial.print(GPS.day, DEC); Serial.print('/');
    //Serial.print(GPS.month, DEC); Serial.print("/20");
    //Serial.println(GPS.year, DEC);
    //Serial.print("Fix: "); Serial.print((int)GPS.fix);
    //Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
    if (GPS.fix) {
      Serial.print("GPS FIX");
      //Serial.print("Location: ");
      //Serial.print(GPS.latitude, 2); Serial.print(GPS.lat);
      //Serial.print(", "); 
      //Serial.print(GPS.longitude, 2); Serial.println(GPS.lon);
      
      float fLat = decimalDegrees(GPS.latitude, GPS.lat);
      float fLon = decimalDegrees(GPS.longitude, GPS.lon);
      
      int closest_loc = find_closest_location(fLat, fLon);
      float targetLat = pgm_read_float(&lat_lon[closest_loc][0]);
      float targetLon = pgm_read_float(&lat_lon[closest_loc][1]);
      
      //Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      //Serial.print("Angle: "); Serial.println(GPS.angle);
      //Serial.print("Altitude: "); Serial.println(GPS.altitude);
      //Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      compass.read();

      const float Pi = 3.14159;

      // Calculate the angle of the vector y,x
      heading = (atan2((int)compass.magData.y,(int)compass.magData.x) * 180) / Pi;
 
      // Normalize to 0-360
      if (heading < 0)
      {
        heading = 360 + heading;
      }
      Serial.print("Heading: ");
      Serial.println(heading);
      setIndicator();
      showIndicator();
      delay(0);    // just to let things settle a bit
//      if ((calc_bearing(fLat, fLon, targetLat, targetLon) - heading) > 0) {
//        headingDirection(calc_bearing(fLat, fLon, targetLat, targetLon)-heading);
//        
//       }
//      else {
//        headingDirection(calc_bearing(fLat, fLon, targetLat, targetLon)-heading+360);
//      }
      
      //headingDistance((double)calc_dist(fLat, fLon, targetLat, targetLon));
      //Serial.print("Distance Remaining:"); Serial.println((double)calc_dist(fLat, fLon, targetLat, targetLon));
    }
  }
}

void setIndicator()
{  // broken up into section to save processor time
  if ((heading >= 11.25)&&(heading < 101.25));  // NNE to E 
  {
    if ((heading >= 11.25) && (heading < 33.75))
    {
      indicator = NNE;
    }  //end if NNE

      if ((heading >= 33.75) && (heading < 56.25))
    {
      indicator = NE;
    }  //end if NE

      if ((heading >= 56.25) && (heading < 78.75))
    {
      indicator = ENE;
    }  //end if ENE

      if ((heading >= 78.75) && (heading < 101.25))
    {
      indicator = E;
    }  //end if E
  }    //end if NNE to E

    if ((heading >= 101.25) && (heading < 191.25))    // ESE to S
  {
    if ((heading >= 101.25) && (heading < 123.75))
    {
      indicator = ESE;
    }  //end if ESE

      if ((heading >= 123.75) && (heading < 146.25))
    {
      indicator = sE;    // Do not change to SE -  "SE" is apparrently a library-used keyword
    }  //end if sE

      if ((heading >= 146.25) && (heading < 168.75))
    {
      indicator = SSE;
    }  //end if SSE

      if ((heading >= 168.75) && (heading < 191.25))
    {
      indicator = S;
    }   //end if S
  }    //end if ESE to S
  if ((heading < 281.25) && (heading > 191.25))    // SSW to W
  {

    if ((heading >= 191.25) && (heading < 213.75))
    {
      indicator = SSW;
    }  //end if SSW

      if ((heading >= 213.75) && (heading < 236.25))
    {
      indicator = SW;
    }   //end if SW

      if ((heading >= 236.25) && (heading < 258.75))
    {
      indicator = WSW;
    }  //end if WSW

      if ((heading >= 258.75) && (heading < 281.25))
    {
      indicator = W;
    }    //end if W
  }    //end if SSW to W

    if ((heading >= 281.25) || (heading < 11.25))    // WNW to N
  {
    if ((heading >= 281.25) && (heading < 303.75))
    {
      indicator = WNW;
    }    //end if WNW

      if ((heading >= 303.75) && (heading < 326.25))
    {
      indicator = NW;
    }  //end if NW

      if ((heading >= 326.25) && (heading < 348.75))
    {
      indicator = NNW;
    }  //end if NNW

      if ((heading >= 348.75) || (heading < 11.25))
    {
      indicator = N;
    }   //end if N

  }  // end if WNW to N

}  // end void setIndicator()

void showIndicator()
{
  // set a little border to highlight the North indicator
  int indicatorLeft = indicator - 1;
  int indicatorRight = indicator + 1;

  // scale / normalize to 0 - 15

  if (indicatorLeft < 0)
  {
    indicatorLeft += 16; 
  }    //end if <0

  if (indicatorRight > 15)
  {
    indicatorRight -= 16; 
  }    //end if <15

  // Assign colors to 'dem thare variables!
  colorWipe(strip.Color(0, 0, 0), 0);             //set All Blue (background dial color)
  strip.setPixelColor(indicator, 0, 0, level);        // set indicator RED
  strip.setPixelColor(indicatorLeft, 0, 0, levelLow);    // set indicator border GREEN
  strip.setPixelColor(indicatorRight, 0, 0, levelLow);   // set indicator border GREEN

  strip.show();                                   // Push bits out!

}  // end void showIndicator()

// Fill the pixels one after the other with a color    
// * Borrowed from ADAFRUIT Example Code
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

unsigned long calc_dist(float flat1, float flon1, float flat2, float flon2)
{
  float dist_calc=0;
  float dist_calc2=0;
  float diflat=0;
  float diflon=0;

  diflat=radians(flat2-flat1);
  flat1=radians(flat1);
  flat2=radians(flat2);
  diflon=radians((flon2)-(flon1));

  dist_calc = (sin(diflat/2.0)*sin(diflat/2.0));
  dist_calc2= cos(flat1);
  dist_calc2*=cos(flat2);
  dist_calc2*=sin(diflon/2.0);
  dist_calc2*=sin(diflon/2.0);
  dist_calc +=dist_calc2;

  dist_calc=(2*atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));

  dist_calc*=6371000.0; //Converting to meters
  return dist_calc;
}

//returns the location in the lat_lon array of the closest lat lon to the current location
int find_closest_location(float current_lat, float current_lon)
{  
  int closest = 0;
  unsigned long minDistance = -1;
  unsigned long tempDistance;
  for (int i=0; i < LAT_LON_SIZE; i++) {
    float target_lat = pgm_read_float(&lat_lon[i][0]);
    float target_lon = pgm_read_float(&lat_lon[i][1]);

    tempDistance = calc_dist(current_lat, current_lon, target_lat, target_lon);
    
    /*
    Serial.print("current_lat: ");
    Serial.println(current_lat, 6);
    Serial.print("current_lon: ");
    Serial.println(current_lon, 6); 
    Serial.print("target_lat: ");
    Serial.println(target_lat, 6);
    Serial.print("target_lon: ");
    Serial.println(target_lon, 6);  
    
    Serial.print("tempDistance: ");
    Serial.println(tempDistance);
    Serial.print("Array Loc: ");
    Serial.println(i); 
    */
    
    if ((minDistance > tempDistance) || (minDistance == -1)) {
      minDistance = tempDistance;
      closest = i;

    }
 
  }
  return closest;
}

// Convert NMEA coordinate to decimal degrees
float decimalDegrees(float nmeaCoord, char dir) {
  uint16_t wholeDegrees = 0.01*nmeaCoord;
  int modifier = 1;

  if (dir == 'W' || dir == 'S') {
    modifier = -1;
  }
  
  return (wholeDegrees + (nmeaCoord - 100.0*wholeDegrees)/60.0) * modifier;
}


int calc_bearing(float flat1, float flon1, float flat2, float flon2)
{
  float calc;
  float bear_calc;

  float x = 69.1 * (flat2 - flat1); 
  float y = 69.1 * (flon2 - flon1) * cos(flat1/57.3);

  calc=atan2(y,x);

  bear_calc= degrees(calc);

  if(bear_calc<=1){
    bear_calc=360+bear_calc; 
  }
  return bear_calc;
}








