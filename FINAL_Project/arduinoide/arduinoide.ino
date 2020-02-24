
// Load Wi-Fi library
#include <ESP8266WiFi.h>

// Replace with your network credentials
const char* ssid     = "iPhone";
const char* password = "chinconba";

// Set web server port number to 80
WiFiServer server(80);

// Variable to store the HTTP request
String header;

// Auxiliar variables to store the current output state
String output5State ;
String output4State;
String output6State ;
String output7State;
// Assign output variables to GPIO pins


// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

void setup() {
  Serial.begin(115200);


  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();
}

void loop(){
  WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    currentTime = millis();
    previousTime = currentTime;
    while (client.connected() && currentTime - previousTime <= timeoutTime) { // loop while the client's connected
      currentTime = millis();         
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
         
            
            // turns the GPIOs on and off
            if (header.indexOf("GET /5/on") >= 0) {
              Serial.println("NG0");
              output5State = "ON";
             
            } else if (header.indexOf("GET /5/off") >= 0) {
              Serial.println("NG1");
              output5State = "OFF";
             
            } else if (header.indexOf("GET /4/on") >= 0) {
              Serial.println("TA0");
              output4State = "ON";
              
            } else if (header.indexOf("GET /4/off") >= 0) {
              Serial.println("TA1");
              output4State = "OFF";
              
            }else if(header.indexOf("GET /6/on") >=0){
              Serial.println("SV0");
              output6State= "ON";
            }else if(header.indexOf("GET /6/off") >=0){
              Serial.println("SV1");
              output6State= "OFF";
            }
            else if(header.indexOf("GET /7/on") >=0){
              Serial.println("DR0");
              output7State= "ON";
            }else if(header.indexOf("GET /7/off") >=0){
              Serial.println("DR1");
              output7State= "OFF";
            }
            
            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // CSS to style the on/off buttons 
            // Feel free to change the background-color and font-size attributes to fit your preferences
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #77878A; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #195B6A;}</style></head>");
            
            // Web Page Heading
            client.println("<body style=\"background-color:#ecf0f1\"><h1>Long Khanh Smarthome mini</h1>");
            
            // Display current state, and ON/OFF buttons for GPIO 5  
            client.println("<p> BEDROOM " + output5State + "</p>");
            // If the output5State is off, it displays the ON button       
            if (output5State=="OFF") {
              client.println("<p><a href=\"/5/on\"><button class=\"button\">OFF</button></a></p>");
            } else {
              client.println("<p><a href=\"/5/off\"><button class=\"button button2\">ON</button></a></p>");
            } 
               
            // Display current state, and ON/OFF buttons for GPIO 4  
            client.println("<p>BATHROOM " + output4State + "</p>");
            // If the output4State is off, it displays the ON button       
            if (output4State=="OFF") {
              client.println("<p><a href=\"/4/on\"><button class=\"button\">OFF</button></a></p>");
            } else {
              client.println("<p><a href=\"/4/off\"><button class=\"button button2\">ON</button></a></p>");
            }

            client.println("<p>DOOR HOME " + output6State + "</p>");
            if (output6State=="OFF") {
              client.println("<p><a href=\"/6/on\"><button class=\"button\">OFF</button></a></p>");
            } else {
              client.println("<p><a href=\"/6/off\"><button class=\"button button2\">ON</button></a></p>");
            }

              client.println("<p>DOOR GARAGE " + output7State + "</p>");
            // If the output4State is off, it displays the ON button       
            if (output7State=="OFF") {
              client.println("<p><a href=\"/7/on\"><button class=\"button\">OFF</button></a></p>");
            } else {
              client.println("<p><a href=\"/7/off\"><button class=\"button button2\">ON</button></a></p>");
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
   
  }
}
