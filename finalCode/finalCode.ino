/*********************
 *10 to GPS Module TX*
 *09 to GPS Module RX*
 *********************/
 
//source: http://www.instructables.com/id/How-to-Communicate-Neo-6M-GPS-to-Arduino/

#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <SparkFun_MAG3110.h>

MAG3110 mag = MAG3110(); //Instantiate MAG3110
SoftwareSerial mySerial(10, 11);
TinyGPS gps;

//51.36513, -0.18960

  float destinationLat = 51.364501;
  float destinationLon = -0.189784;

void setup()  
{
  // make the car face north!

  
  
  // Oploen serial communications and wait for port to open:
  Serial.begin(9600);
  // set the data rate for the SoftwareSerial port
  mySerial.begin(9600);
  delay(1000);
  Serial.println("uBlox Neo 6M");
  Serial.print("Testing TinyGPS library v. "); Serial.println(TinyGPS::library_version());
  Serial.println("by Mikal Hart");
  Serial.println();
  Serial.print("Sizeof(gpsobject) = "); 
  Serial.println(sizeof(TinyGPS));
  Serial.println(); 
  
  pinMode(13, OUTPUT); // forward
  pinMode(12, OUTPUT); // right

  mag.initialize(); //Initializes the mag sensor
  
  delay(1000);
}

float latitude(TinyGPS &gps) {
  float flat, flon;
  gps.f_get_position(&flat, &flon);
  return flat;
}

float longitude(TinyGPS &gps) {
  float flat, flon;
  gps.f_get_position(&flat, &flon);
  return flon;
}






// unused
void gpsdump(TinyGPS &gps)
{
  float flat, flon;
  gps.f_get_position(&flat, &flon);
  Serial.print("Lat/Long(float): "); printFloat(flat, 5); Serial.print(", "); printFloat(flon, 5);
}





void printFloat(double f, int digits = 2);



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


const double pi= 3.14159265358979;
double distance(double lat1, double long1, double lat2, double long2) { //find distance between two lat-long pairs
  //an application of the Haversine formula
  
  //returns answer in metres
  
  double R = 6371e3; //radius of the Earth, in metres (change to km for answer to be in km, etc)

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

double distanceAlternative(double lat1, double long1, double lat2, double long2) { //find distance between two lat-long pairs (alternate, easier method)
  //an application of the cosine rule
  
  //returns answer in metres
  
  double R = 6371e3; //radius of the Earth, in metres (change to km for answer to be in km, etc)

  double lat1Radians = toRadians(lat1);
  double lat2Radians = toRadians(lat2);
  
  double deltaLong = toRadians(long2-long1);
  
  double d = acos( sin(lat1Radians) * sin(lat2Radians) + cos(lat1Radians) * cos(lat2Radians) * cos(deltaLong) ) * R;

  return d;
}

double bearing(float lat1, double long1, double lat2, double long2) { //find bearing between two lat-long pairs
  //untested so far...
  
  //returns answer in radians (can easily be converted to degrees if necessary)
  
  double y = sin(long2-long1) * cos(lat2);
  double x = cos(lat1)*sin(lat2) -
          sin(lat1)*cos(lat2)*cos(long2-long1);
  
  double bearing = atan2(y, x);
  
  return toDegrees(bearing);
}

void calibrateMag() {
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

double toRadians(double degrees) { //convert degrees to radians (cpp functions accept radians only)
  return degrees * pi / 180;
}

double toDegrees(double radian) {
  double degree = radian / pi * 180;
  if ( degree < 0) {
    degree += 360;
  }
  return degree;
}


void moveForward(int distance) { // distance is in cm
  digitalWrite(13, HIGH);
  delay(distance * 9.32);
  digitalWrite(13, LOW);
}

void turnRight(int lat, int lon, int destinationLat, int destinationLon) { // rotation is in degrees
  
  float targetBearing = bearing(lat, lon, destinationLat, destinationLon);
  if (targetBearing>180)
  {
    targetBearing = targetBearing-360;
  }
  digitalWrite(12, HIGH);
  digitalWrite(13, HIGH);
  float currentHeading = mag.readHeading();
  while(currentHeading < targetBearing - 7 || currentHeading > targetBearing + 7)
  {
    currentHeading = mag.readHeading();
  }
  digitalWrite(12, LOW);
  digitalWrite(13, LOW);
}

void loop() // run over and over
{
  if(!mag.isCalibrated())
  {
    calibrateMag();
  }
  else {
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
  
      //destinationLat = lat + 0;
      //destinationLon = lon + 100;
  
      Serial.println("");
      Serial.println("--- New data ---");
      
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
      Serial.println("Current bearing: ");
      float bearingValue = mag.readHeading();
      printFloat(bearingValue);
  
      Serial.println("");
      Serial.println("Target Bearing:  ");
      float targetBearing = bearing(lat, lon, destinationLat, destinationLon);
      printFloat(targetBearing);
      Serial.println("");
      Serial.println("Bearing to turn: ");
      bearingValue = bearing(lat, lon, destinationLat, destinationLon) - mag.readHeading();
      if (bearingValue > 360) {
        bearingValue -= 360;
      }
      else if (bearingValue < 0) {
        bearingValue += 360;
      }
      printFloat(bearingValue);
      Serial.println("");
      Serial.println("Distance to travel: ");
      printFloat(distance(lat, lon, destinationLat, destinationLon));
      Serial.println("");
      
      turnRight(lat, lon, destinationLat, destinationLon);
      delay(1000);
      moveForward(distance(lat, lon, destinationLat, destinationLon) * 100);
  
      Serial.println("Destination reached.");
    }
    delay(1000);
    Serial.println(" Recalculating position...");
  }
}
