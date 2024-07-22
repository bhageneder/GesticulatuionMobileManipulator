// ----------------------------------------------------------
// Load Libraries
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
// ----------------------------------------------------------

// ----------------------------------------------------------
// Create global class variables
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); // uses the default address 0x40
WiFiServer server(80);
// ----------------------------------------------------------

// ----------------------------------------------------------
// Creating global fields
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define ssid "ESPNetwork" // Network SSID
#define password "password" // Network Password
#define SERVOCW  410 // PWM length to go continuous clockwise
#define SERVOSTOP  600  // PWM length to go continuous stop
#define SERVOCOUNTERCW 205  // PWM length to go continuous counter clockwise

String header; // Variable to store the HTTP request

String forwardState  = "off"; // Assign output variables
String leftState     = "off"; // Assign output variables
String rightState    = "off"; // Assign output variables

bool forward = false; // Assign output variables
bool left    = false; // Assign output variables
bool right   = false; // Assign output variables

unsigned long currentTime = millis(); // Current time
unsigned long previousTime = 0; // Previous time
const long timeoutTime = 10000; // Define timeout time in milliseconds (example: 2000ms = 2s)
// ----------------------------------------------------------

// ----------------------------------------------------------
// Method to transmit information back to external device / other esp to be read
void sendMessage(String message) {
  // Temporary serial print message
  Serial.println(message);
  // TODO: Create algorithm to send data
}
// ----------------------------------------------------------


void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(2000);
  sendMessage("Setting up servo");
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ); // Analog servos run at ~50 Hz
  sendMessage("Servo Setup");

  sendMessage("Connecting to: ");
  sendMessage(ssid);
  WiFi.softAP(ssid, password);

  sendMessage("");
  sendMessage("Wifi Connecting:");
  sendMessage("IP Addr: ");
  sendMessage((String)WiFi.softAPIP());
  server.begin();
  sendMessage("Wifi Connected");

}

// ----------------------------------------------------------
// Public methods to control servo motors

void moveForward() {
  sendMessage("Moving Forward");
  pwm.setPWM(0, 1, SERVOCW);
  pwm.setPWM(1, 0, SERVOCOUNTERCW);
}

void turnRight() {
  sendMessage("Turning Right");
  pwm.setPWM(0, 0, SERVOCW);
  pwm.setPWM(1, 0, SERVOCW);
}

void turnLeft() {
  sendMessage("Turning Left");
  pwm.setPWM(0, 0, SERVOCOUNTERCW);
  pwm.setPWM(1, 0, SERVOCOUNTERCW);
}

void moveStop() {
  sendMessage("Stoping Movement");
  pwm.setPWM(0, 0, SERVOSTOP);
  pwm.setPWM(1, 0, SERVOSTOP);
}

void mapServoPositionFromAngle() {
  // Do Something Here
}
// ----------------------------------------------------------


void loop(){
  WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    currentTime = millis();
    previousTime = currentTime;
    sendMessage("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected() && currentTime - previousTime <= timeoutTime) {  // loop while the client's connected
      currentTime = millis();
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();

            // turns the directions on and off
            if (header.indexOf("GET /forward/on") >= 0) {
              moveForward();
              forward       = true;
              forwardState  = "on";
            } else if (header.indexOf("GET /forward/off") >= 0) {
              moveStop();
              forward       = false;
              forwardState  = "off";
            } else if (header.indexOf("GET /left/on") >= 0) {
              turnLeft();
              left      = true;
              leftState = "on";
            } else if (header.indexOf("GET /left/off") >= 0) {
              moveStop();
              left      = false;
              leftState = "off";
            } else if (header.indexOf("GET /right/on") >= 0 ) {
              turnRight();
              right       = true;
              rightState  = "on";
            } else if (header.indexOf("GET /right/off") >= 0) {
              moveStop();
              right       = false;
              rightState  = "off";
            }
            
            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // CSS to style the on/off buttons 
            // Feel free to change the background-color and font-size attributes to fit your preferences
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #555555;}</style></head>");
            
            // Web Page Heading
            client.println("<body><h1>ESP32 Web Server</h1>");
            
            // Display current state, and ON/OFF buttons for Forward  
            client.println("<p>Forward State " + forwardState + "</p>");
            // If Forward is off, it displays the ON button       
            if (forward == false) {
              client.println("<p><a href=\"/forward/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/forward/off\"><button class=\"button button2\">OFF</button></a></p>");
            } 
               
            // Display current state, and ON/OFF buttons for Left  
            client.println("<p>Left State " + leftState + "</p>");
            // If Left is off, it displays the ON button       
            if (left == false) {
              client.println("<p><a href=\"/left/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/left/off\"><button class=\"button button2\">OFF</button></a></p>");
            }

            // Display current state, and ON/OFF buttons for Right  
            client.println("<p>Right State " + rightState + "</p>");
            // If Right is off, it displays the ON button       
            if (right == false) {
              client.println("<p><a href=\"/right/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/right/off\"><button class=\"button button2\">OFF</button></a></p>");
            }
            client.println("</body></html>");
            
            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
}   