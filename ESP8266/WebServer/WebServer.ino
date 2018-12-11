/* Create a WiFi access point and provide a web server on it.
 *
 * Code is modified from
 * https://42bots.com/tutorials/esp8266-example-wi-fi-access-point-web-server-static-ip-remote-control/
 */

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <FS.h>   // Include the SPIFFS library

// Configuration
IPAddress    apIP(192, 168, 4, 1);
const char *ssid = "MASTER";
const char *password = "";

// Define a web server at port 80 for HTTP
ESP8266WebServer server(80);

void handleRoot() {

  File file = SPIFFS.open("/manCtrl.html", "r");
  server.streamFile(file, "text/html");
}

int sat(int in){
  if(in>9){
    return 9;
  }
  if (in<-9){
    return -9;
  }
  else{
    return in;
  }
}

void handleInst(){
  char argBuf[80];
  for ( uint8_t i = 0; i < server.args(); i++ ) {
    (server.argName(i)).toCharArray(argBuf, sizeof(argBuf));
    if (strcmp(argBuf, "x")==0){
      Serial.printf("x%d", sat(server.arg(i).toInt()));
    }
    if (strcmp(argBuf, "y")==0){
      Serial.printf("y%d", sat(server.arg(i).toInt()));
    }
    if (strcmp(argBuf, "s")==0){
      Serial.printf("s%d", sat(server.arg(i).toInt()));
    }
  }
  server.send ( 200, "text/plain", "OK" );
}

void handleNotFound() {

  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += ( server.method() == HTTP_GET ) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";

  for ( uint8_t i = 0; i < server.args(); i++ ) {
    message += " " + server.argName ( i ) + ": " + server.arg ( i ) + "\n";
  }

  server.send ( 404, "text/plain", message );
}

void setup() {
  delay(1500);

  Serial.begin(115200);
  Serial.println();
  Serial.println("Configuring access point...");

  SPIFFS.begin(); 

  //set-up the custom IP address
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));   // subnet FF FF FF 00  
  
  /* You can remove the password parameter if you want the AP to be open. */
  WiFi.softAP(ssid, password);

  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
 
  server.on ( "/", handleRoot );
  server.on ( "/inst", handleInst );
  server.on ( "/inline", []() {
    server.send ( 200, "text/plain", "this works as well" );
  } );
  server.onNotFound ( handleNotFound );
  
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  
  server.handleClient();
}
