/*********************
 *10 to GPS Module TX*
 *09 to GPS Module RX*
 *********************/
 
//source: http://www.instructables.com/id/How-to-Communicate-Neo-6M-GPS-to-Arduino/

#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <SparkFun_MAG3110.h>

SoftwareSerial mySerial(10, 11);
TinyGPS gps;

MAG3110 mag = MAG3110(); //Instantiate MAG3110



//51.36513, -0.18960

  float destinationLat = 51.365173;
  float destinationLon = -0.189486;

void setup()  
{
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  // set the data rate for the SoftwareSerial port
  mySerial.begin(9600);
  
  pinMode(13, OUTPUT); // forward
  pinMode(12, OUTPUT); // right
  
  delay(1000);

  mag.initialize(); // Initializes the mag sensor
  mag.start();      // Puts the sensor in active mode
}

// get latitude
float latitude(TinyGPS &gps) {
  float flat, flon;
  gps.f_get_position(&flat, &flon);
  return flat;
}

// get lognitude
float longitude(TinyGPS &gps) {
  float flat, flon;
  gps.f_get_position(&flat, &flon);
  return flon;
}

// debug function: get current lat and long
void gpsdump(TinyGPS &gps)
{
  float flat, flon;
  gps.f_get_position(&flat, &flon);
  Serial.print("Lat/Long(float): "); printFloat(flat, 5); Serial.print(", "); printFloat(flon, 5);
}

// not sure why this needs to be here, but removing it breaks the code...
void printFloat(double f, int digits = 2);

// print float to serial
void printFloat(double number, int digits)
{
  // Handle negative numbers
  if (number < 0.0) 
  {
     Serial.print('-');
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;
  
  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  Serial.print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    Serial.print("."); 

  // Extract digits from the remainder one at a time
  while (digits-- > 0) 
  {
    remainder *= 10.0;
    int toPrint = int(remainder);
    Serial.print(toPrint);
    remainder -= toPrint;
  }
}

// for converting between degrees and radians
const double pi= 3.14159265358979;

// find distance between two lat-long pairs
double distance(double lat1, double long1, double lat2, double long2) {
  // an application of the Haversine formula
  
  // returns answer in metres
  
  double R = 6371e3; // radius of the Earth, in metres (change to km for answer to be in km, etc)

  double lat1Radians = toRadians(lat1);
  double lat2Radians = toRadians(lat2);

  double deltaLat = toRadians(lat2-lat1);
  double deltaLong = toRadians(long2-long1);

  double a = sin(deltaLat/2) * sin(deltaLat/2) +
          cos(lat1Radians) * cos(lat2Radians) *
          sin(deltaLong/2) * sin(deltaLong/2);
  double c = 2 * atan2(sqrt(a), sqrt(1-a));

  double d = R * c;
  
  return d;
}

// find bearing between two lat-long pairs
double bearing(float lat1, double long1, double lat2, double long2) {
  // returns answer in radians (can easily be converted to degrees if necessary)
  
  double y = sin(long2-long1) * cos(lat2);
  double x = cos(lat1)*sin(lat2) -
          sin(lat1)*cos(lat2)*cos(long2-long1);
  
  double bearing = atan2(y, x);
  
  return toDegrees(bearing);
}

// convert degrees to radians (cpp trig functions use radians)
double toRadians(double degrees) {
  return degrees * pi / 180;
}

// convert radians to degrees
double toDegrees(double radian) {
  double degree = radian / pi * 180;
  if ( degree < 0) {
    degree += 360;
  }
  return degree;
}

// move car forward a certain amount of cm
void moveForward(int distance) { // distance is in cm
  digitalWrite(13, HIGH);
  delay(distance * 9.32);
  digitalWrite(13, LOW);
}

// turn car right a certian amount of degrees
void turnRight(int rotation) { // rotation is in degrees
  digitalWrite(12, HIGH);
  digitalWrite(13, HIGH);
  delay(rotation * 11.76);
  digitalWrite(12, LOW);
  digitalWrite(13, LOW);
}

void calibrateMag()
{
  

  if(!mag.isCalibrated()) //If we're not calibrated
  {
    if(!mag.isCalibrating()) //And we're not currently calibrating
    {
      Serial.println("Entering calibration mode");
      mag.enterCalMode(); //This sets the output data rate to the highest possible and puts the mag sensor in active mode
    }
    else
    {
      //Must call every loop while calibrating to collect calibration data
      //This will automatically exit calibration
      //You can terminate calibration early by calling mag.exitCalMode();
      mag.calibrate(); 
    }
  }
  else
  {
    Serial.println("Calibrated!");
  }
}

float readMagnetometerHeading() {
  int x, y, z;
  
  return mag.readHeading();
}

void loop() // run over and over
{
  if(!mag.isCalibrated())
  {
    // turn on motors for magnetometer calibration
    digitalWrite(12, HIGH);
    digitalWrite(13, HIGH);
    calibrateMag();
  }

  else { // check that the magnetometer is calibrated

    // turn off motors from calibration
    digitalWrite(12, LOW);
    digitalWrite(13, LOW);
    
    bool newdata = false;
    unsigned long start = millis();
    while (millis() - start < 5000) 
    {
      if (mySerial.available()) 
      
      {
        //Serial.println("Searching for GPS data...");
        char c = mySerial.read();
        //Serial.print(c);  // uncomment to see raw GPS data
        if (gps.encode(c)) 
        {
          Serial.println("GPS data found.");
          newdata = true;
          break;  // uncomment to print new data immediately!
        }
      }
    }
      
    if (newdata) 
    {
      float lat = latitude(gps);
      float lon = longitude(gps);
  
      float magHeading = readMagnetometerHeading();

      // face approximately north (perhaps should be less approximate in the future
      digitalWrite(12, HIGH);
      digitalWrite(13, HIGH);
      while(magHeading > 15 || magHeading < -15) {
        // not facing north; keep moving and get another reading
        magHeading = readMagnetometerHeading();
      }
      digitalWrite(12, LOW);
      digitalWrite(13, LOW);

      // test code
      destinationLat = lat + 0;
      destinationLon = lon + 100;
  
      Serial.println("--- New data ---");
      
      Serial.println("");
      Serial.println("Current latitude: ");
      printFloat(lat, 6);
      Serial.println("");
      Serial.println("Current longitude: ");
      printFloat(lon, 6);
  
      Serial.println("");
      Serial.println("Destination latitude: ");
      printFloat(destinationLat, 6);
      Serial.println("");
      Serial.println("Destination longitude: ");
      printFloat(destinationLon, 6);
  
      Serial.println("");
      Serial.println("Bearing: ");
      printFloat(bearing(lat, lon, destinationLat, destinationLon));
      Serial.println("");
      Serial.println("Distance: ");
      printFloat(distance(lat, lon, destinationLat, destinationLon));
      Serial.println("");

      // move towards destination
      turnRight(bearing(lat, lon, destinationLat, destinationLon) * 5);
      delay(1000);
      moveForward(distance(lat, lon, destinationLat, destinationLon) * 100);
  
      Serial.println("Destination reached.");
    }
    delay(1000);
    Serial.println(" Recalculating position...");
  }
}
