#include "MyDuckClass.h"
//#include "Arduino.h”
#include <WiFi.h>
MyDuckClass roboDuck(27,14,25,26);
double homeLat;
double homeLong;
float leftTopCorner[2] = {0, 0};
// Replace with your network credentials
const char* ssid = "ESP32-Access-Point";
const char* password = "123456789";
#include <TinyGPSPlus.h>
TinyGPSPlus gps;
// Set web server port number to 80
WiFiServer server(80);

// Variable to store the HTTP request
String header;

// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

bool calibrate = false;
bool waypoints = false;
bool spotlock = false;
bool returnhome = false;

void setup() {
    Serial.begin(115200);
    // Setting ESP32 as access point 
    Serial.print("Setting AP (Access Point)…");
    // Remove the password parameter, if you want the AP (Access Point) to be open
    WiFi.softAP(ssid, password);
  
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
    // Connect to Wi-Fi network with SSID and password
    /*Serial.print("Connecting to ");
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
    Serial.println(WiFi.localIP());*/
    server.begin();
 
}

void loop() {
  //roboDuck.movePropHeading(45);
  //roboDuck.movePropHeading(275);
  WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    currentTime = millis();
    previousTime = currentTime;
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected() && currentTime - previousTime <= timeoutTime) {  // loop while the client's connected
        if (gps.encode(Serial2.read()))
        {
          if (gps.location.isValid()){
            homeLat= (gps.location.lat());
            homeLong = (gps.location.lng());
            }
          }
          
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

            // handle get requests for four buttons
            // current button bool is set to true, all others are reset to false
            if(header.indexOf("GET /calibrate") >= 0){
              Serial.println("calibrate");
              calibrate = true;
              Serial.println(calibrate);
              waypoints = false;
              spotlock = false;
              returnhome = false;
            } else if (header.indexOf("GET /waypoints") >= 0){
                int headerLength = header.length();
                String foo = header.substring(26, headerLength);
                char chars[foo.length()];
                foo.toCharArray(chars, foo.length());
                Serial.println(foo);
                int count = 0;
                for(int i = 0; i < foo.length(); i++){
                  if (foo[i] == ','){
                    count++;
                  }
                }
                int arrayLen = (count + 1) / 2;
                Serial.println(count);
                
                const char *delimiter = ",";
                char *token;
                double currValue;
                double waypointPercentValues [arrayLen];
                token = strtok(chars, delimiter);
                currValue = atof(token);
                Serial.println(currValue);
                waypointPercentValues[0] = currValue;
                Serial.println("Index 0");
                Serial.println(waypointPercentValues[0]);
                int j = 1;
                while (token != NULL) {
                  if(j == arrayLen){
                    break;
                  }
                  Serial.println(token);
                  token=strtok(NULL, delimiter);
                  
                  currValue = atof(token);
                  Serial.println(currValue);
                  waypointPercentValues[j] = currValue;
                  Serial.println(j);
                  Serial.println(waypointPercentValues[j]);
                  j++;
                  // waypointPercentValues
                }
                Serial.println("waypoints");
                waypoints = true;
                Serial.println(waypoints);
                calibrate = false;
                spotlock = false;
                returnhome = false;
                int k;
                for(k = 0; k < arrayLen; k += 2){
                  double waypointGPSTargetLong = leftTopCorner[1] + (.0005 * waypointPercentValues[k]);
                  double waypointGPSTargetLat = leftTopCorner[0] + (.0005 * waypointPercentValues[k+1]);
                  Serial.println("New Long");
                  Serial.println(waypointGPSTargetLong);
                  Serial.println("New Lat");
                  Serial.println(waypointGPSTargetLat);
                }

               
            } else if (header.indexOf("GET /spotlock") >= 0){
                Serial.println("spotlock");
                spotlock = true;
                Serial.println(spotlock);
            } else if (header.indexOf("GET /sethome") >= 0){
                Serial.println("returnhome");
                returnhome = true;
                Serial.println(returnhome);
                homeLat = gps.location.lat();
                homeLong = gps.location.lng();
                float setHomeLat = homeLat;
                float setHomeLong = homeLong; 
                calibrate = false;
                waypoints = false;
                spotlock = false;
                leftTopCorner[0] = setHomeLat + .00005;
                leftTopCorner[1] = setHomeLong - .00005;
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
            client.println("<body><h1>Robo Duck</h1>");
            client.println("<center><div id='grid' style='width:80%; border:5px solid black;'><button style='background-color:red;' onclick='resetWaypoints()'>Reset Waypoints</button><button>Waypoints Left: <div id='numWaypoints'>10</div></button><canvas id='canvas' style='height:100%; width:100%; background-color:white;' /></div></center>");
            client.println("<div style='display:grid; grid-template-rows: 2fr 2fr; grid-template-columns: 2fr 2fr;'><a href='/calibrate'><button class='button'>Calibrate</button></a><a href='/waypoints/' onclick=\"location.href=this.href+'?waypoints='+waypoints;return false;\"><button class='button'>Waypoints</button></a><a href='/spotlock'><button class='button'>Spot Lock</button></a><a href='/sethome'><button class='button'>Set Home</button></a></div>"); 

            client.println("</body></html>");
            client.println("<script type='text/javascript'> \ 
                const gridInit = document.getElementById('grid');\
                const height = gridInit.clientHeight;\
                const width = gridInit.clientWidth;\
                var waypoints = [];\
                function hello(){ \
                  console.log('Hello');\
                }\
                function resetWaypoints(){\
                  var response = confirm('Are you sure you want to reset all waypoints?');\
                  console.log(response);\
                  if(response){\
                    waypoints = [];\
                    var canvas = document.getElementById('canvas');\
                    var ctx = canvas.getContext('2d');\
                    ctx.beginPath();\
                    ctx.clearRect(0, 0, canvas.width, canvas.height);\
                  }\
                }\
                function decrementWaypoints(){\
                  console.log('DecrementWaypoints');\
                  var divElement = document.getElementById('numWaypoints');\
                  var numWaypoints = divElement.innerHTML;\
                  if(numWaypoints == 0){\
                    alert('no more waypoints');\
                    return false;\
                  }\
                  else{\
                    numWaypoints--;\
                    divElement.innerHTML = numWaypoints;\
                    return true;\
                  }\
                }\
                function printMousePos(event) {\
                  var absX = event.clientX;\
                  var absY = event.clientY;\
                  console.log('clientX: ' + event.clientX + '- clientY: ' + event.clientY);\
                  var y = Math.round(document.getElementById('grid').getBoundingClientRect().top);\
                  var x = Math.round(document.getElementById('grid').getBoundingClientRect().left);\
                  console.log(x + 'hello' + y);\
                  var trueX = absX - x;\
                  var trueY = absY - y;\
                  console.log('trueX: ' + trueX + ' trueY: ' + trueY);\
                  console.log('percentX: ' + (trueX / width) + ' percentY: ' + (trueY / height));\ 
                  var canvasWidth = document.getElementById('grid').getBoundingClientRect().width;\
                  var canvasHeight = document.getElementById('grid').getBoundingClientRect().height;\
                  var trueWaypointXPct = (trueX / canvasWidth);\
                  var trueWaypointYPct = (trueY / canvasHeight);\
                  drawWaypoint(trueX, trueY, trueWaypointXPct, trueWaypointYPct);\      
                }\
                function drawWaypoint(x, y, trueWaypointXPct, trueWaypointYPct){\
                  var canvas = document.getElementById('canvas');\
                  var ctx = canvas.getContext('2d');\
                  ctx.beginPath();\
                  waypoints.push([trueWaypointXPct,trueWaypointYPct]);\
                  console.log(waypoints);\
                  var drawWaypoint = decrementWaypoints();\
                  if (drawWaypoint){\
                    ctx.arc(x, y, 10, 0, 2 * Math.PI);\
                    ctx.stroke(); \
                  }\
                }\
                var grid = document.getElementById('canvas');\
                grid.addEventListener('click', printMousePos);\
                </script> \
                ");
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
