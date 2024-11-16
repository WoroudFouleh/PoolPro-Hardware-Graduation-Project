#include <ESP8266WiFi.h>

// Server - AP
const char* ssid = "Woroud Fouleh";
const char* password = "Woroud264";

WiFiServer server(80);

void setup() {
  Serial.begin(9600);
  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
 // Serial.print("AP IP address: ");
 // Serial.println(IP);

  server.begin();
}

void loop() {
  WiFiClient client = server.available();

  if (client) {
    //Serial.println("New Client.");
    String currentLine = "";
    String postData = "";
    bool isPost = false;
    int contentLength = 0;

    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
       // Serial.write(c);

        if (c == '\n') {
          // Empty line, end of headers
          if (currentLine.length() == 0) {
            if (isPost) {
              // Read POST data
              //Serial.println("Reading POST data...");
              while (postData.length() < contentLength) {
                if (client.available()) {
                  char c = client.read();
                  postData += c;
                }
              }
              //Serial.print("POST Data: ");
              Serial.println(postData);
            }

            // // Respond to the client
            // client.println("HTTP/1.1 200 OK");
            // client.println("Content-type:text/html");
            // client.println();
            // client.println("<html><body><h1>ESP32 Server</h1></body></html>");
            // client.println();
            break;
          } else {
            // Parse headers
            if (currentLine.startsWith("Content-Length: ")) {
              contentLength = currentLine.substring(16).toInt();
            }

            if (currentLine.startsWith("POST ")) {
              isPost = true;
            }

            currentLine = "";
          }
        } else if (c != '\r') {
          currentLine += c;
        }
      }
    }
    client.stop();
   // Serial.println("Client Disconnected.");
  }
}