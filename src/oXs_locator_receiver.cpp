#include "Arduino.h"
#include <ESP8266WebServer.h> 
#include <ESP8266WiFi.h> 
#include <WiFiClient.h>  // these are  libraries 
#include "lora_receiver.h"
#include "config.h"

#include <string> // header file for string
using namespace std; 

// to do 

bool wifiEnabled = false;
// We set a Static IP address
IPAddress local_IP LOCAL_IP;
// We set a Gateway IP address
IPAddress gateway GATEWAY;
IPAddress subnet SUBNET;


ESP8266WebServer server(80); 
 
// Make a wifi name and password as access points
const char *ssid = "oXs_locator";
const char *password = "";

// led
#define LED 2 // GPIO 2 on wmos d1 mini 
int statusLed = HIGH; // variable for LED status  ; HIGH = OFF
bool blinking = true ;
uint8_t ledState = STATE_NO_SIGNAL;
uint8_t prevLedState = STATE_NO_SIGNAL;

uint32_t lastBlinkMillis;

extern uint8_t oXsGpsPdop ;    // gps precision sent by oxs
extern uint8_t oXsLastGpsDelay ; // delay since last gps fix at oxs side

extern int oXsPacketRssi ;   // RSSI of last packet received by oXs

extern uint32_t loraLastPacketReceivedMillis ;
extern bool atLeastOnePacketReceived;  
extern uint32_t loraLastGpsPacketReceivedMillis ;
extern int loraRxPacketRssi ;
extern float loraRxPacketSnr ;
extern int32_t lastGpsLon ; 
extern int32_t lastGpsLat ;    

// values to put on html
uint8_t lonDegree ;
uint8_t lonMinute ;
uint8_t lonSeconde ;
uint16_t lonSecDec ;
uint8_t latDegree ;
uint8_t latMinute ;
uint8_t latSeconde ;
uint16_t latSecDec ;

uint8_t loraDebugCode ;
uint8_t countTxToDebug = 0 ; // just to debug
uint8_t countRxToDebug = 0 ; // just to debug

// text for html
String oXsLastGpsDelayText[8] = { "<h2>No GPS data</h2>" , "<h2>GPS data received less than 1s ago</h2>" , 
        "<h2>GPS data received between 1s and 10s ago</h2>" , "<h2>GPS data received between 10s and 1m ago</h2>" ,
        "<h2>GPS data received between 1m and 10m ago</h2>" , "<h2>GPS data received between 10m and 1h ago</h2>" ,
        "<h2>GPS data received between 1h and 10h ago</h2>" , "<h2>GPS data received more than 10h ago</h2>" } ;


void convertLonLat( int32_t GPS_LatLon, uint8_t &_degree , uint8_t & _minute , uint8_t & _seconde, uint16_t & _secDec  ) {
  uint32_t GPS_LatLonAbs ;
  uint32_t minute7decimals ;
  uint32_t seconde7decimals ;
  uint32_t secondeDec7decimals ;
  //Serial.print("LonLat= ") ; Serial.println( GPS_LatLon ) ;
  GPS_LatLonAbs = ( GPS_LatLon < 0 ? - GPS_LatLon : GPS_LatLon)  ; 
  // example : 40432955 => 4° 2' 35.865"
  _degree = ( GPS_LatLonAbs / 10000000 )  ;    // 4                          // extract the degre without  decimal
  minute7decimals = (GPS_LatLonAbs % 10000000 ) * 60;  // 0432955*60 = 25977300
  _minute =  minute7decimals / 10000000 ;  // 25977300 /10000000 = 2
  seconde7decimals = (minute7decimals % 10000000 ) * 60 ; // 5977300*60 = 358638000
  _seconde = seconde7decimals / 10000000 ; //358638000 / 10000000 = 35
  seconde7decimals = (minute7decimals % 10000000 ) * 60 ; // 5977300*60 = 358638000
  secondeDec7decimals =  ( seconde7decimals % 10000000 )  ; //8638000 
  _secDec = secondeDec7decimals / 10000 ; //8638000 / 10000 = 863

  //Serial.print("degree= ") ; Serial.println( (int32_t)_degree ) ;
}


String toNPos(int val ,uint8_t n) { // add space before to get a length = n0 
  //Serial.print("val=");Serial.print(val); 
  String result = String("           ") + String(val) ;
  //Serial.print(" result="); Serial.print(result);
  unsigned int pos =  result.length() - n ;
  //Serial.print(" pos="); Serial.print(pos);
  //Serial.print(" substring="); Serial.println(result.substring(pos));
  return result.substring(pos);
}

String toNPos0(int val ,uint8_t n) { // add 0 before to get a length = n0 
  String result = String("000000000") + String(val) ;
  unsigned int len =  result.length() ;
  //Serial.println(val); Serial.println(result); Serial.println(len); Serial.println(result.substring(len - n ));
  return result.substring(len - n);
}

String formatLatLongString(int32_t l){ // format required by google maps (7 decimals)
    String resp ;
    if (l < 0){
        l = -l;
        resp = String("-");
    }
    int32_t rest = l % 10000000l;
    return  resp + String(l/10000000l) + String(".") + toNPos0(rest,7); 
}

int counter = 0; //only for debugging to check that the html page change
// some Strings for html
const String HtmlHtml = String("<!DOCTYPE html><html><body><head>") + 
    String("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\" /></head>") +
    String("<h1>oXs locator</h1>");
const String htlmExplain1 = String("<p>To view the location on a map, first disconnect from oXs wifi (to get internet access) and then click this button</p>") ;
const String htlmExplain2 = String("<p>You can also copy next url, then disconnect from oXs wifi , open a browser and paste this url</p>") ;
const String HtmlHtmlClose = "</body></html>";
String htmlLat ;
String htmlLon ;

// Function / procedure to handle each client making a request
void response()
{
    counter++ ; 
    String htmlGpsDelay =   String("<h2>") + oXsLastGpsDelayText[oXsLastGpsDelay] + String("</h2>");
    String htmlGpsData = String("<p> </p>") ; 
    
    if (oXsLastGpsDelay != 0) {
        String htmlDop = String("<h4>Gps precision : ") + String(oXsGpsPdop) + String("</h4>");
        
        convertLonLat( lastGpsLon , lonDegree , lonMinute , lonSeconde , lonSecDec ) ; 
        //Serial.println(lastGpsLon);Serial.println(lonDegree); Serial.println(lonSeconde);Serial.println(lonSecDec);
        if (lastGpsLon >= 0) {
            htmlLon = String("<h4>Longitude = E ");
        } else {
            htmlLon = String("<h4>Longitude = O ");
        }
        htmlLon = htmlLon + toNPos(lonDegree,2) + "&deg " + toNPos(lonMinute,2) + "' " +
                    toNPos(lonSeconde,2) + "." + toNPos0(lonSecDec,3) + "</h4>" ;
        
        //Serial.print("lonDegree back= ") ; Serial.println(lonDegree) ; 
        convertLonLat( lastGpsLat , latDegree , latMinute , latSeconde , latSecDec ) ;
        if (lastGpsLat >= 0) {
            htmlLat = String("<h4>Latitude = N ");
        } else {
            htmlLat = String("<h4>Longitude = S ");
        }
        htmlLat = htmlLat + toNPos(latDegree,2) + "&deg " + toNPos(latMinute,2) + "' " +
                    toNPos(latSeconde,2) + "." + toNPos0(latSecDec , 3) + "</h4>" ;
        
        String latLonS = formatLatLongString(lastGpsLat) + String("%2C") + formatLatLongString(lastGpsLon);
        
        String htmlLink2Map = String("<a href=\"https://www.google.com/maps/search/?api=1&query=") +
                               latLonS + String("\"><button><h2>Link to google maps</h2></button></a>") ;
    
        String htmlCopy2Map = String("<p>https://www.google.com/maps/search/?api=1&query=") + latLonS  + String("</p>") + String("</h4>");
        
        
        htmlGpsData = htmlDop + htmlLon + htmlLat + htlmExplain1 + htmlLink2Map + htlmExplain2 + htmlCopy2Map ;
        //Serial.println(htmlGpsData);
    }

    String htmlLast ;
    String htmlOxsRssi ;
    String htmlLocRssi ;
    String htmlLocSnr ;
    if (atLeastOnePacketReceived == false) {
        htmlLast = String("<h2>No packet received from oXs!</h2>") ;
        htmlOxsRssi = String("<br/>") ;
        htmlLocRssi = htmlOxsRssi;
        htmlLocSnr = htmlOxsRssi;
    } else {
        uint32_t delayLastPacketReceived ;
        delayLastPacketReceived = millis() - loraLastPacketReceivedMillis ;
        if ( delayLastPacketReceived < 60000 )  {
            htmlLast = String("<h4>Last pack rec. ") + String((int) delayLastPacketReceived /1000) + String(" sec ago</h4>") ;
        } else {
            htmlLast = String("<h4>Last pack rec. ") + String((int) delayLastPacketReceived /60000) + String(" min ago</h4>") ;
        }
        htmlOxsRssi = String("<h4>oXs RSSI= ") + String(oXsPacketRssi) + "</h4>";
        htmlLocRssi = String("<h4>Locator RSSI= ") + String(loraRxPacketRssi) + String("</h4>");
        htmlLocSnr = String("<h4>Locator SNR= ") + String(loraRxPacketSnr) + String("</h4>");
    }
        
    //const String htmlTest = String("<br/>Counter = ") + String(counter) + String("<br/>") ;
    const String htmlRes = HtmlHtml + htmlGpsDelay +  htmlGpsData + htmlLast + htmlOxsRssi +
                            htmlLocRssi + htmlLocSnr  + HtmlHtmlClose;
    server.send(200, "text/html", htmlRes);
}


void handleLed(){    // set the colors based on the RF link
    // when no packet have been received, led is off
    // when a packet has been received recently, led is on
    // when a packet has been received but not recently, then led is blinking
    switch (ledState) {
        case STATE_NO_SIGNAL:
            statusLed = HIGH;
            lastBlinkMillis = millis(); // reset the timestamp for blinking
            break;
        case STATE_NO_RECENT_RECEIVE:
            if ((millis() - lastBlinkMillis) > 500) { // toggle led every x msec
                if (statusLed == LOW) {
                    statusLed = HIGH;
                } else {
                    statusLed = LOW;
                }
                lastBlinkMillis = millis(); // reset the timestamp for blinking
            }
            break;
        case STATE_RECENTLY_RECEIVED:
            statusLed = LOW;
            lastBlinkMillis = millis(); // reset the timestamp for blinking
            break;    
    }
    digitalWrite(LED, statusLed);  
}

#define BUTTON_OUT_GROUND 5 // = D1 on Wemos mini D1
#define BUTTON_IN_PULLUP 0  // = D3 on wemos mini D1

void setupWifi(){
    static bool alreadyReadHigh = false;
    if (millis() < 1000) return;
    // we wait to have first a level HIGH (before looking for a low level )
    if ((digitalRead(BUTTON_IN_PULLUP) == 1) and ( alreadyReadHigh == false)) {
        alreadyReadHigh = true;
        return;
    }     
    if (digitalRead(BUTTON_IN_PULLUP) == 0) {
        Serial.print("Setting soft-AP configuration ... ");
  Serial.println(WiFi.softAPConfig(local_IP, gateway, subnet) ? "Ready" : "Failed!");

  Serial.print("Setting soft-AP ... ");
  Serial.println(WiFi.softAP(ssid,password) ? "Ready" : "Failed!");
  delay(100);
  //WiFi.softAP(ssid);
  //WiFi.softAP(ssid, password, channel, hidden, max_connection)
  
  Serial.print("Visit this IP address in your browser = ");
  Serial.println(WiFi.softAPIP());
  delay(100);      
        
        //WiFi.softAP(ssid, password);
        //WiFi.softAPConfig (local_IP, gateway, subnet);
        //IPAddress apip2 = WiFi.softAPIP(); // Get the IP server
        //Serial.print("Connect your wifi laptop/mobile phone to this Access Point : ");
        //Serial.println(ssid);
        //Serial.print("Visit this IP : ");
        //Serial.print(apip2); // Prints the IP address of the server to be visited
        //Serial.println(" in your browser.");
        
        server.on("/", response); 
        //server.on("/LEDOn", handleLedOn);
        //server.on("/LEDOff", handleLedOff);
    
        server.begin(); // Start the server
        Serial.println("HTTP server beginned");
        wifiEnabled = true;
    }    
} 

void setup() {
    delay(1000); 
    Serial.begin(115200);
    Serial.println();
  
    pinMode(LED, OUTPUT);
    digitalWrite(LED, statusLed);
    pinMode(BUTTON_OUT_GROUND, OUTPUT);
    digitalWrite(BUTTON_OUT_GROUND, LOW);
    pinMode(BUTTON_IN_PULLUP , INPUT_PULLUP);
    delay(1000);
}
 
void debugButton(){
    static uint32_t lastRead;
    if ((millis() - lastRead) > 500) {
        Serial.print(wifiEnabled); Serial.println(digitalRead(BUTTON_IN_PULLUP));
        lastRead = millis();
    }
}

void loop()
{ 
    // fill loraRxPacketRssi , loraRxPacketSnr , lastGpsLat ,  lastGpsLon , oXsGpsPdop , oXsLastGpsDelay , oXsPacketRssi
    loraDebugCode = loraHandle();
    if (atLeastOnePacketReceived == true) { // manage the led state
        if ((millis() - loraLastPacketReceivedMillis ) < 1000 ) { 
            ledState = STATE_RECENTLY_RECEIVED;   
        } else {
            ledState = STATE_NO_RECENT_RECEIVE;
        }
    }
    handleLed(); // change led depending on ledState
    debugButton();
    if (wifiEnabled == false) setupWifi() ; // check if wifi button is pressed; when pressed, start wifi and change flag wifiEnabled to true
    if (wifiEnabled) {
        server.handleClient(); // this manage the wifi process (when client connect, it call response()) 
    }
}

    /*
    String htmlGetLocation = String("<p>Click the button to get your coordinates.</p>") +
                            String("<button onclick=\"printMsg()\">Msg</button>\n") +
                            String("<button onclick=\"getLocation()\">Try It</button><n") +
                            String("<p id=\"mymsg\"></p>\n") + 
                            
                            String("<p id=\"demo\"></p>\n") + 
                            String("<script>\n") +
                            String("\nconst x = document.getElementById(\"demo\");\n") +
                            String("\nconst y = document.getElementById(\"mymsg\");\n") +
                            
                            String("function printMsg() {\n") +
                            String("y.innerHTML = \"This is my message\";\n") +
                            String("}\n")+

                            String("function getLocation() {\n") +
                            String("if (navigator.geolocation) {\n") +
                            String("navigator.geolocation.getCurrentPosition(showPosition);\n") +
                            String("} else {\n") + 
                            String("x.innerHTML = \"Geolocation is not supported by this browser.\";\n") +
                            String("  }\n") +
                            String("}\n")+

                            String("function showPosition(position) {\n") +
                            String("x.innerHTML = \"Latitude: \" + position.coords.latitude + \"<br>Longitude: \" + position.coords.longitude;\n") +
                            String("}\n") +

                            String("</script>\n") ;
        */
