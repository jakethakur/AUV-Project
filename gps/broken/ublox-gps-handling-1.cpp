 #include <SoftwareSerial.h>
SoftwareSerial GPS(10, 9);//serial port for the GPS
byte gps_set_sucess = 0 ;//global variable
uint8_t request[] = {0xB5, 0x62, 0x01, 0x02, 0x00, 0x00,0x00,0x00};//global variable
 
void setup()
{
  GPS.begin(9600); 
  // START OUR SERIAL DEBUG PORT
  Serial.begin(9600);
  Serial.println("GPS Demonstration Script");
  Serial.println("Initialising....");
  // THE FOLLOWING COMMAND SWITCHES MODULE TO 4800 BAUD
  // THEN SWITCHES THE SOFTWARE SERIAL TO 4,800 BAUD
  // lower baud rate gives increased reliability over software serial
  GPS.print("$PUBX,41,1,0007,0003,4800,0*13\r\n"); 
  GPS.begin(4800);
  GPS.flush();
  

  //  THIS COMMAND SETS SEA MODE AND CONFIRMS IT 
  Serial.println("Setting uBlox nav mode: ");
  uint8_t setNav[]={0xB5,0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x05, 0x01, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  
  sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));



//set output format to binary (UBX protocol)
  uint8_t setOut[]={0xB5,0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0xC0, 0x12, 0x00, 0x00, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  
  sendUBX(setOut, sizeof(setOut)/sizeof(uint8_t));

  
//activate nav-posllh (lat,long,height) messages on this port
  uint8_t setMsg[]={0xB5,0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  
  sendUBX(setMsg, sizeof(setMsg)/sizeof(uint8_t));

  
//deactivate nav-sol(ecef) messages 
  uint8_t setSol[]={0xB5,0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  
  sendUBX(setSol, sizeof(setSol)/sizeof(uint8_t));



 }
void loop()
{
    /*if(GPS.available())
    {
      // THIS IS THE MAIN LOOP JUST READS IN FROM THE GPS SERIAL AND ECHOS OUT TO THE ARDUINO SERIAL. TO BE USED WITH OTHER HARDWARE/SOFTWARE E.G. U-CENTER SOFTWARE
      Serial.write(GPS.read()); 
    }
    */
  
  
  //code below is used to print the GPS output in human readable format
    if(GPS.available()){
    delay(200);    
      uint8_t buf[60];//the packet from the GPS
      uint8_t datain;
      
      for (int x=0; x<60; x++){
        datain = GPS.read();
        buf[x]=datain;
      }
      // 4 bytes of longitude (1e-7)
    int32_t lon = (int32_t)buf[10] | (int32_t)buf[11] << 8 |  (int32_t)buf[12] << 16 | (int32_t)buf[13] << 24;
    
    // 4 bytes of latitude (1e-7)
    int32_t lat = (int32_t)buf[14] | (int32_t)buf[15] << 8 |  (int32_t)buf[16] << 16 | (int32_t)buf[17] << 24;
    
    // 4 bytes of altitude above MSL (mm)
    int32_t alt = (int32_t)buf[18] | (int32_t)buf[19] << 8 |  (int32_t)buf[20] << 16 | (int32_t)buf[21] << 24; 
    Serial.print("longitude:");
    Serial.println(lon);
    Serial.print("latitude:");
    Serial.println(lat);
    Serial.print("altitude:");
    Serial.println(alt);
    }
}    
 
 
// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  while(!gps_set_sucess){//keep trying until GPS acknowledges message
    uint8_t CK_A = 0x00; //checksum variables
    uint8_t CK_B = 0x00;
 
    for(int I=2;I<len-2;I++)//checksum algorithm (does not use first 2 bytes)
   {
     CK_A = CK_A + MSG[I];
     CK_B = CK_B + CK_A;
   }

    MSG[len-2] = CK_A;//saves checksum to last 2 bytes of message
    MSG[len-1] = CK_B;
    for(int i=0; i<len; i++) {
      GPS.write(MSG[i]);
    Serial.print(MSG[i], HEX);//for debugging
  }
  GPS.println();
    gps_set_sucess=getUBX_ACK(MSG);
  }
gps_set_sucess=0;//reset 'message acknowledged' variable  
}
 
 
// Calculate expected UBX ACK packet and parse UBX response from GPS
boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();
  Serial.print(" * Reading ACK response: ");
 
  // Construct the expected ACK packet    
  ackPacket[0] = 0xB5; // header
  ackPacket[1] = 0x62; // header
  ackPacket[2] = 0x05; // class
  ackPacket[3] = 0x01; // id
  ackPacket[4] = 0x02; // length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2]; // ACK class
  ackPacket[7] = MSG[3]; // ACK id
  ackPacket[8] = 0; // CK_A
  ackPacket[9] = 0; // CK_B
 
  // Calculate the checksums
  for (uint8_t i=2; i<8; i++) {
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }
 
  while (1) {
 
    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      Serial.println(" (SUCCESS!)");
      return true;
    }
 
    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) { 
      Serial.println(" (FAILED!)");
      return false;
    }
 
    // Make sure data is available to read
    if (GPS.available()) {
      b = GPS.read();
 
      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) { 
        ackByteID++;
        Serial.print(b, HEX);
      } 
      else {
        ackByteID = 0; // Reset and look again, invalid order
      }
 
    }
  }
}
