/*
   ----------------------------------------------------------------------------
   TECHNOLARP - https://technolarp.github.io/
   SERRURE MEDFAN 02 - https://github.com/technolarp/serrure_medfan_02
   version 1.0 - 09/2023
   ----------------------------------------------------------------------------
*/

/*
   ----------------------------------------------------------------------------
   Pour ce montage, vous avez besoin de 
   4 ou + led neopixel
   4 ou + switch reed
   1 MCP23017 en i2c
   ----------------------------------------------------------------------------
*/

/*
   ----------------------------------------------------------------------------
   PINOUT
   D0     NEOPIXEL
   D1     I2C SCL => SCL MCP23017
   D2     I2C SDA => SDA MCP23017

   MCP_A0 switch reed 1
   MCP_A1 switch reed 2
   MCP_A2 switch reed 3
   MCP_A3 switch reed 4
   ----------------------------------------------------------------------------
*/

/*
TODO version 1.1

ajouter du son
ajouter une commande d'actionneur

re-verouiller apres X secondes
scintillement tournant

meilleur choix de couleur par defaut (bleu / cyan)
*/

#include <Arduino.h>

// WIFI
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>

AsyncWebServer server(80);

// WEBSOCKET
AsyncWebSocket ws("/ws");

char bufferWebsocket[300];
bool flagBufferWebsocket = false;

// CONFIG
#include "config.h"
M_config aConfig;

#define BUFFERSENDSIZE 600
char bufferToSend[BUFFERSENDSIZE];

// FASTLED
#include <technolarp_fastled.h>
M_fastled aFastled;

// MCP23017
#include "technolarp_mcp23017.h"
M_mcp23017 aMcp23017;

// STATUTS DE L OBJET
enum {
  OBJET_OUVERT = 0,
  OBJET_FERME = 1,
  OBJET_OUVERTURE = 2,
  OBJET_BLINK = 5
};

// CODE ACTUEL DE LA SERRURE
uint8_t codeSerrureActuel[MAX_SIZE_CODE] = {4,4,4,4,4,4,4,4,4,4};

// DIVERS
bool uneFois = true;
bool glypheAll = false;
bool glypheOneTime = true;

uint32_t previousMillisCheck;
uint32_t previousMillisReset;
uint32_t previousMillisBrightness;

bool checkTimeoutFlag = false;

uint32_t lastDebounceTime[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint8_t lastPinState[16] = {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH,HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH};
uint8_t pinState[16] = {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH,HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH};
bool pinActive[16] = {true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true};

// HEARTBEAT
uint32_t previousMillisHB;
uint32_t intervalHB;

// FUNCTION DECLARATIONS
void serrureFermee();
void serrureOuverte();
void serrureOuverture();
void serrureBlink();
void checkReed();
void checkTimeout();
void showSparklePixel(uint8_t led);
void checkCharacter(char* toCheck, const char* allowed, char replaceChar);
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len); 
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len); 
void handleWebsocketBuffer();
void notFound(AsyncWebServerRequest *request);
void convertStrToRGB(const char * source, uint8_t* r, uint8_t* g, uint8_t* b);
void sendMaxLed();
void sendUptime();
void sendStatut();
void sendObjectConfig();
void writeObjectConfig();
void sendNetworkConfig();
void writeNetworkConfig();
uint16_t checkValeur(uint16_t valeur, uint16_t minValeur, uint16_t maxValeur);
uint8_t indexMaxValeur(uint8_t arraySize, uint8_t arrayToSearch[]);

/*
   ----------------------------------------------------------------------------
   SETUP
   ----------------------------------------------------------------------------
*/
void setup() 
{
  Serial.begin(115200);

  // VERSION
  delay(500);
  Serial.println(F(""));
  Serial.println(F(""));
  Serial.println(F("----------------------------------------------------------------------------"));
  Serial.println(F("TECHNOLARP - https://technolarp.github.io/"));
  Serial.println(F("SERRURE MEDFAN 02 - https://github.com/technolarp/serrure_mefan_02"));
  Serial.println(F("version 1.0 - 09/2023"));
  Serial.println(F("----------------------------------------------------------------------------"));
  
  // I2C RESET
  aConfig.i2cReset();
  
  // CONFIG OBJET
  Serial.println(F(""));
  Serial.println(F(""));
  aConfig.mountFS();
  aConfig.listDir("/");
  aConfig.listDir("/config");
  aConfig.listDir("/www");
  
  aConfig.printJsonFile("/config/objectconfig.txt");
  aConfig.readObjectConfig("/config/objectconfig.txt");

  aConfig.printJsonFile("/config/networkconfig.txt");
  aConfig.readNetworkConfig("/config/networkconfig.txt");

  aConfig.objectConfig.activeLeds = aConfig.objectConfig.nbSegments * aConfig.objectConfig.ledParSegment;
  aConfig.writeObjectConfig("/config/objectconfig.txt");

  // MCP23017
  aMcp23017.beginMcp23017(0);
  
  // FASTLED
  aFastled.setNbLed(aConfig.objectConfig.activeLeds);
  // animation led de depart
  aFastled.animationDepart(50, aFastled.getNbLed()*2, CRGB::Blue);

  // initialiser l'aleat
  randomSeed(ESP.getCycleCount());

  // WIFI
  WiFi.disconnect(true);

  Serial.println(F(""));
  Serial.println(F("connecting WiFi"));
  
  /**/
  // AP MODE
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAPConfig(aConfig.networkConfig.apIP, aConfig.networkConfig.apIP, aConfig.networkConfig.apNetMsk);
  bool apRC = WiFi.softAP(aConfig.networkConfig.apName, aConfig.networkConfig.apPassword);

  if (apRC)
  {
    Serial.println(F("AP WiFi OK"));
  }
  else
  {
    Serial.println(F("AP WiFi failed"));
  }

  // Print ESP soptAP IP Address
  Serial.print(F("softAPIP: "));
  Serial.println(WiFi.softAPIP());
  
  /*
  // CLIENT MODE POUR DEBUG
  const char* ssid = "SID";
  const char* password = "PASSWORD";
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  if (WiFi.waitForConnectResult() != WL_CONNECTED) 
  {
    Serial.println(F("WiFi Failed!"));
  }
  else
  {
    Serial.println(F("WiFi OK"));
  }
  */
  
  // Print ESP Local IP Address
  Serial.print(F("localIP: "));
  Serial.println(WiFi.localIP());
  
  // WEB SERVER
  // Route for root / web page
  server.serveStatic("/", LittleFS, "/www/").setDefaultFile("config.html");
  server.serveStatic("/config", LittleFS, "/config/");
  server.onNotFound(notFound);

  // WEBSOCKET
  ws.onEvent(onEvent);
  server.addHandler(&ws);

  // Start server
  server.begin();

  // RESET TIMEOUT
  previousMillisReset = millis();
  previousMillisBrightness = millis();
  
  // HEARTBEAT
  previousMillisHB = millis();
  intervalHB = 10000;

  // SERIAL
  Serial.println(F(""));
  Serial.println(F(""));
  Serial.println(F("START !!!"));
}
/*
   ----------------------------------------------------------------------------
   FIN DU SETUP
   ----------------------------------------------------------------------------
*/


/*
   ----------------------------------------------------------------------------
   LOOP
   ----------------------------------------------------------------------------
*/
void loop() 
{
  // AVOID WATCHDOG
  yield();
  
  // WEBSOCKET
  ws.cleanupClients();

  // FASTLED
  aFastled.updateAnimation();

  // CONTROL BRIGHTNESS
  aFastled.controlBrightness(aConfig.objectConfig.brightness);

  // gerer le statut de la serrure
  switch (aConfig.objectConfig.statutActuel)
  {
    case OBJET_FERME:
      // la serrure est fermee
      serrureFermee();
      break;

    case OBJET_OUVERT:
      // la serrure est ouverte
      serrureOuverte();
      break;

    case OBJET_OUVERTURE:
      // animation de changement d'etat
      serrureOuverture();
      break;
      
    case OBJET_BLINK:
      // blink led pour identification
      serrureBlink();
      break;
      
    default:
      // nothing
      break;
  }

  // traiter le buffer du websocket
  if (flagBufferWebsocket)
  {
    flagBufferWebsocket = false;
    handleWebsocketBuffer();
  }

  // HEARTBEAT
  if(millis() - previousMillisHB > intervalHB)
  {
    previousMillisHB = millis();

    // envoyer l'uptime
    sendUptime();
  }
}
/*
   ----------------------------------------------------------------------------
   FIN DU LOOP
   ----------------------------------------------------------------------------
*/


/*
   ----------------------------------------------------------------------------
   FONCTIONS ADDITIONNELLES
   ----------------------------------------------------------------------------
*/
void serrureFermee()
{
  if (uneFois)
  {
    uneFois = false;

    for (uint8_t i=0;i<MAX_SIZE_CODE;i++)
    {
      codeSerrureActuel[i]=5;
    }

    Serial.println(F("SERRURE FERMEE"));
  }

  checkReed();
  checkTimeout();
}

void serrureOuverte()
{
  if (uneFois)
  {
    uneFois = false;

    for (uint8_t i=0;i<MAX_SIZE_CODE;i++)
    {
      codeSerrureActuel[i]=6;
    }
    
    Serial.println(F("SERRURE OUVERTE"));
  }

  checkReed();
  checkTimeout();
}

void serrureOuverture()
{
  if (uneFois)
  {
    uneFois = false;
    Serial.println(F("SERRURE OUVERTURE"));

    aFastled.animationBlink02Start(75, 1550, aConfig.objectConfig.couleurs[6], CRGB::Black);
  }

  // fin de l'animation blink
  if(!aFastled.isAnimActive()) 
  {
    uneFois = true;

    // inverser le statut de la serrure
    aConfig.objectConfig.statutActuel=!aConfig.objectConfig.statutPrecedent;
    
    writeObjectConfig();
    sendObjectConfig();

    Serial.println(F("END OUVERTURE"));
  }
}

void serrureBlink()
{
  if (uneFois)
  {
    uneFois = false;
    Serial.println(F("SERRURE BLINK"));

    sendStatut();

    aFastled.animationBlink02Start(100, 3000, CRGB::Blue, CRGB::Black);
  }

  // fin de l'animation blink
  if(!aFastled.isAnimActive()) 
  {
    uneFois = true;

    aConfig.objectConfig.statutActuel = aConfig.objectConfig.statutPrecedent;

    writeObjectConfig();
    sendObjectConfig();

    Serial.println(F("END SERRURE "));
  }
}

void checkReed()
{
  int16_t lastPin=-1;
  
  // lire la valeur de chaque pin du mcp23017
  for (uint8_t i=0;i<aConfig.objectConfig.nbSegments;i++)
  {
    pinState[i]=aMcp23017.readPin(i);

    if (pinState[i]==0)
    {
      lastDebounceTime[i]=millis();
    }
    
    // on maj le tableau des debounce
    if ( (pinState[i] != lastPinState[i]) && (pinActive[i]==true) )
    {
      lastPinState[i]=pinState[i];      
      pinActive[i]=false;

      if (pinState[i]==0 )
      {
        lastPin=i;
      }
    }

    // on check si une pin doit etre reactivée
    uint32_t currentTime = millis();
    if ( (pinActive[i]==false) && ((currentTime - lastDebounceTime[i]) > aConfig.objectConfig.debounceTime) )
    {
      lastDebounceTime[i]=currentTime;
      pinActive[i]=true;
    }
  }
  
  // un reed a été activé, on change la couleur du glyphe
  if (lastPin>-1)
  {
    codeSerrureActuel[lastPin]+=1;
    codeSerrureActuel[lastPin]%=aConfig.objectConfig.nbCouleurs;

    previousMillisCheck = millis();
    previousMillisReset = previousMillisCheck;

    Serial.print("last pin: ");
    Serial.print(lastPin);
    Serial.print("  codeActuel: ");
    Serial.println(codeSerrureActuel[lastPin]);
  }

  // afficher les leds
  for (uint8_t i=0;i<aConfig.objectConfig.nbSegments;i++)
    {
      for (uint8_t j=0;j<aConfig.objectConfig.ledParSegment;j++)
      {
        aFastled.ledOn(i*aConfig.objectConfig.ledParSegment+j, aConfig.objectConfig.couleurs[codeSerrureActuel[i]], false);
      }
    }
    aFastled.ledShow();

  // check si tout les glyphes ont été activé
  bool checkGlyphes = true;
  for (uint8_t i=0;i<aConfig.objectConfig.nbSegments;i++)
  {
    if (codeSerrureActuel[i]>=5)
    {
      checkGlyphes = false;
    }
  }

  if (checkGlyphes && glypheOneTime)
  {
    Serial.println(F("checkGlyphes"));
    glypheAll = true;
    glypheOneTime = false;

    previousMillisCheck=millis();
  }

  // si tous les glyphes ont été activés
  if (glypheAll)
  {
    // attendre le délai de timeuutCheck
    if(millis() - previousMillisCheck > aConfig.objectConfig.timeoutCheck)
    {
      previousMillisCheck = millis();
      bool codeOK=true;
      for (uint8_t i=0;i<aConfig.objectConfig.nbSegments;i++)
      {
        Serial.print("codeActuel: ");
        Serial.print(codeSerrureActuel[i]);
        Serial.print("  code: ");
        Serial.println(aConfig.objectConfig.code[i]);
        if (codeSerrureActuel[i] != aConfig.objectConfig.code[i])
        {
          codeOK=false;
        }
      }

      if (codeOK)
      {
        Serial.println(F("code OK"));
        aConfig.objectConfig.statutPrecedent=aConfig.objectConfig.statutActuel;
        aConfig.objectConfig.statutActuel=OBJET_OUVERTURE;
        
      }
      else
      {
        Serial.println(F("code faux"));
      }
      uneFois = true;
    
      glypheAll=false;
      glypheOneTime=true;
    }
  }
}

void checkTimeout()
{
  // reset si timeout
  if(millis() - previousMillisReset > aConfig.objectConfig.timeoutReset)
  {
    previousMillisReset = millis();

    Serial.println(F("timeout"));    
    uneFois = true;
    
    glypheAll=false;
    glypheOneTime=true;
  }
}

// index of max value in an array
uint8_t indexMaxValeur(uint8_t arraySize, uint8_t arrayToSearch[])
{
  uint8_t indexMax=0;
  uint8_t currentMax=0;
  
  for (uint8_t i=0;i<arraySize;i++)
  {
    if (arrayToSearch[i]>=currentMax)
    {
      currentMax = arrayToSearch[i];
      indexMax = i;
    }
  }
  
  return(indexMax);
}

void checkCharacter(char* toCheck, const char* allowed, char replaceChar)
{
  for (uint8_t i = 0; i < strlen(toCheck); i++)
  {
    if (!strchr(allowed, toCheck[i]))
    {
      toCheck[i]=replaceChar;
    }
    Serial.print(toCheck[i]);
  }
  Serial.println("");
}

uint16_t checkValeur(uint16_t valeur, uint16_t minValeur, uint16_t maxValeur)
{
  return(min(max(valeur,minValeur), maxValeur));
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) 
{
   switch (type) 
    {
      case WS_EVT_CONNECT:
        Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
        // send config value to html
        sendObjectConfig();
        sendNetworkConfig();
        
        // send volatile info
        sendMaxLed();

        sendUptime();
        sendStatut();    
        break;
        
      case WS_EVT_DISCONNECT:
        Serial.printf("WebSocket client #%u disconnected\n", client->id());
        break;
        
      case WS_EVT_DATA:
        handleWebSocketMessage(arg, data, len);
        break;
        
      case WS_EVT_PONG:
      case WS_EVT_ERROR:
        break;
  }
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) 
{
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) 
  {
    data[len] = 0;
    sprintf(bufferWebsocket,"%s\n", (char*)data);
    Serial.print(len);
    Serial.print(bufferWebsocket);
    flagBufferWebsocket = true;
  }
}
  
void handleWebsocketBuffer()
{
    DynamicJsonDocument doc(JSONBUFFERSIZE);
    
    DeserializationError error = deserializeJson(doc, bufferWebsocket);
    if (error)
    {
      Serial.println(F("Failed to deserialize buffer"));
    }
    else
    {
        // write config or not
        bool writeObjectConfigFlag = false;
        bool sendObjectConfigFlag = false;
        bool writeNetworkConfigFlag = false;
        bool sendNetworkConfigFlag = false;
        
        // **********************************************
        // modif object config
        // **********************************************
        if (doc.containsKey("new_objectName"))
        {
          strlcpy(  aConfig.objectConfig.objectName,
                    doc["new_objectName"],
                    sizeof(aConfig.objectConfig.objectName));
  
          writeObjectConfigFlag = true;
          sendObjectConfigFlag = true;
        }
  
        if (doc.containsKey("new_objectId")) 
        {
          uint16_t tmpValeur = doc["new_objectId"];
          aConfig.objectConfig.objectId = checkValeur(tmpValeur,1,1000);
  
          writeObjectConfigFlag = true;
          sendObjectConfigFlag = true;
        }
  
        if (doc.containsKey("new_groupId")) 
        {
          uint16_t tmpValeur = doc["new_groupId"];
          aConfig.objectConfig.groupId = checkValeur(tmpValeur,1,1000);
          
          writeObjectConfigFlag = true;
          sendObjectConfigFlag = true;
        }
  
        if (doc.containsKey("new_activeLeds")) 
        {
          FastLED.clear(); 
          
          uint16_t tmpValeur = doc["new_activeLeds"];
          aConfig.objectConfig.activeLeds = checkValeur(tmpValeur,1,NB_LEDS_MAX);
          uneFois=true;
          
          writeObjectConfigFlag = true;
          sendObjectConfigFlag = true;
        }
        
        if (doc.containsKey("new_brightness"))
        {
          uint16_t tmpValeur = doc["new_brightness"];
          aConfig.objectConfig.brightness = checkValeur(tmpValeur,0,255);
          FastLED.setBrightness(aConfig.objectConfig.brightness);
          uneFois=true;
          
          writeObjectConfigFlag = true;
          sendObjectConfigFlag = true;
        }

        if (doc.containsKey("new_intervalScintillement"))
      {
        uint16_t tmpValeur = doc["new_intervalScintillement"];
        aConfig.objectConfig.intervalScintillement = checkValeur(tmpValeur,0,1000);
        aFastled.setIntervalControlBrightness(aConfig.objectConfig.intervalScintillement);
        
        writeObjectConfigFlag = true;
        sendObjectConfigFlag = true;
      }
      
      if (doc.containsKey("new_scintillementOnOff"))
      {
        uint16_t tmpValeur = doc["new_scintillementOnOff"];
        aConfig.objectConfig.scintillementOnOff = checkValeur(tmpValeur,0,1);
        aFastled.setControlBrightness(aConfig.objectConfig.scintillementOnOff);
        
        if (aConfig.objectConfig.scintillementOnOff == 0)
        {
          FastLED.setBrightness(aConfig.objectConfig.brightness);
        }
        
        writeObjectConfigFlag = true;
        sendObjectConfigFlag = true;
      }

        if (doc.containsKey("new_nbCouleurs"))
        {
          uint16_t tmpValeur = doc["new_nbCouleurs"];
          aConfig.objectConfig.nbCouleurs = checkValeur(tmpValeur,1,5);
          
          writeObjectConfigFlag = true;
          sendObjectConfigFlag = true;
        }

        if (doc.containsKey("new_couleurs")) 
        {
          JsonArray newCouleur = doc["new_couleurs"];
  
          uint8_t i = newCouleur[0];
          char newColorStr[8];
          strncpy(newColorStr, newCouleur[1], 8);
            
          uint8_t r;
          uint8_t g;
          uint8_t b;
            
          convertStrToRGB(newColorStr, &r, &g, &b);
          aConfig.objectConfig.couleurs[i].red=r;
          aConfig.objectConfig.couleurs[i].green=g;
          aConfig.objectConfig.couleurs[i].blue=b;
            
          writeObjectConfigFlag = true;
          sendObjectConfigFlag = true;
        }
  
        if (doc.containsKey("new_nbSegments"))
        {
          uint16_t tmpValeur = doc["new_nbSegments"];
          aConfig.objectConfig.nbSegments = checkValeur(tmpValeur,1,10);
          aConfig.objectConfig.activeLeds = aConfig.objectConfig.nbSegments * aConfig.objectConfig.ledParSegment;
          aFastled.setNbLed(aConfig.objectConfig.activeLeds);
          aFastled.allLedOff(false);
                    
          uneFois=true;
          
          writeObjectConfigFlag = true;
          sendObjectConfigFlag = true;
        }

        if (doc.containsKey("new_ledParSegment"))
        {
          uint16_t tmpValeur = doc["new_ledParSegment"];
          aConfig.objectConfig.ledParSegment = checkValeur(tmpValeur,1,5);
          aConfig.objectConfig.activeLeds = aConfig.objectConfig.nbSegments * aConfig.objectConfig.ledParSegment;
          aFastled.setNbLed(aConfig.objectConfig.activeLeds);
          aFastled.allLedOff(false);
          
          uneFois=true;
          
          writeObjectConfigFlag = true;
          sendObjectConfigFlag = true;
        }

        if (doc.containsKey("new_code"))
        {
          JsonArray newCodeToSet = doc["new_code"];
        
          uint8_t nouvellePosition = newCodeToSet[0];
          uint8_t nouvelleValeur = newCodeToSet[1];
          
          aConfig.objectConfig.code[nouvellePosition]=nouvelleValeur;
          
          writeObjectConfigFlag = true;
          sendObjectConfigFlag = true;
        }

        if ( doc.containsKey("new_resetCode") && doc["new_resetCode"]==1 )
        {
          for (uint8_t i=0;i<MAX_SIZE_CODE;i++)
          {
            aConfig.objectConfig.code[i] = 0;
          }
          
          writeObjectConfigFlag = true;
          sendObjectConfigFlag = true;
        }

        if ( doc.containsKey("new_aleatCode") && doc["new_aleatCode"]==1 )
        {
          for (uint8_t i=0;i<aConfig.objectConfig.nbSegments;i++)
          {
            uint8_t aRandom = random(0,(aConfig.objectConfig.nbCouleurs*10)-1);
            aRandom /= 10;
            aConfig.objectConfig.code[i]=aRandom;
          }
          
          writeObjectConfigFlag = true;
          sendObjectConfigFlag = true;
        }

        if (doc.containsKey("new_timeoutCheck"))
        {
          uint16_t tmpValeur = doc["new_timeoutCheck"];
          aConfig.objectConfig.timeoutCheck = checkValeur(tmpValeur,1000,10000);
          
          writeObjectConfigFlag = true;
          sendObjectConfigFlag = true;
        }
        
        if (doc.containsKey("new_timeoutReset"))
        {
          uint16_t tmpValeur = doc["new_timeoutReset"];
          aConfig.objectConfig.timeoutReset = checkValeur(tmpValeur,1000,30000);
          
          writeObjectConfigFlag = true;
          sendObjectConfigFlag = true;
        }

        if (doc.containsKey("new_debounceTime"))
        {
          uint16_t tmpValeur = doc["new_debounceTime"];
          aConfig.objectConfig.debounceTime = checkValeur(tmpValeur,50,1000);
          
          writeObjectConfigFlag = true;
          sendObjectConfigFlag = true;
        }

        if (doc.containsKey("new_statutActuel"))
        {
          aConfig.objectConfig.statutPrecedent=aConfig.objectConfig.statutActuel;
          
          uint16_t tmpValeur = doc["new_statutActuel"];
          aConfig.objectConfig.statutActuel=tmpValeur;

          uneFois=true;
          
          writeObjectConfigFlag = true;
          sendObjectConfigFlag = true;
        }
        
        // **********************************************
        // modif network config
        // **********************************************
        if (doc.containsKey("new_apName")) 
        {
          strlcpy(  aConfig.networkConfig.apName,
                    doc["new_apName"],
                    sizeof(aConfig.networkConfig.apName));
        
          // check for unsupported char
          const char listeCheck[] = "ABCDEFGHIJKLMNOPQRSTUVWYXZ0123456789_-";
          checkCharacter(aConfig.networkConfig.apName, listeCheck, 'A');
          
          writeNetworkConfigFlag = true;
          sendNetworkConfigFlag = true;
        }
        
        if (doc.containsKey("new_apPassword")) 
        {
          strlcpy(  aConfig.networkConfig.apPassword,
                    doc["new_apPassword"],
                    sizeof(aConfig.networkConfig.apPassword));
        
          writeNetworkConfigFlag = true;
          sendNetworkConfigFlag = true;
        }
        
        if (doc.containsKey("new_apIP")) 
        {
          char newIPchar[16] = "";
        
          strlcpy(  newIPchar,
                    doc["new_apIP"],
                    sizeof(newIPchar));
        
          IPAddress newIP;
          if (newIP.fromString(newIPchar))
          {
            Serial.println("valid IP");
            aConfig.networkConfig.apIP = newIP;
        
            writeNetworkConfigFlag = true;
          }
          
          sendNetworkConfigFlag = true;
        }
        
        if (doc.containsKey("new_apNetMsk")) 
        {
          char newNMchar[16] = "";
        
          strlcpy(  newNMchar,
                    doc["new_apNetMsk"],
                    sizeof(newNMchar));
        
          IPAddress newNM;
          if (newNM.fromString(newNMchar)) 
          {
            Serial.println("valid netmask");
            aConfig.networkConfig.apNetMsk = newNM;
        
            writeNetworkConfigFlag = true;
          }
        
          sendNetworkConfigFlag = true;
        }
        
        // actions sur le esp8266
        if ( doc.containsKey("new_restart") && doc["new_restart"]==1 )
        {
          Serial.println(F("RESTART RESTART RESTART"));
          ESP.restart();
        }
        
        if ( doc.containsKey("new_refresh") && doc["new_refresh"]==1 )
        {
          Serial.println(F("REFRESH"));
        
          sendObjectConfigFlag = true;
          sendNetworkConfigFlag = true;
        }
        
        if ( doc.containsKey("new_defaultObjectConfig") && doc["new_defaultObjectConfig"]==1 )
        {
          aConfig.writeDefaultObjectConfig("/config/objectconfig.txt");
          Serial.println(F("reset to default object config"));
        
          aFastled.allLedOff();
          aFastled.setNbLed(aConfig.objectConfig.activeLeds);          
          aFastled.setControlBrightness(aConfig.objectConfig.scintillementOnOff);
          aFastled.setIntervalControlBrightness(aConfig.objectConfig.intervalScintillement);
          
          sendObjectConfigFlag = true;
          uneFois = true;
        }
        
        if ( doc.containsKey("new_defaultNetworkConfig") && doc["new_defaultNetworkConfig"]==1 )
        {
          aConfig.writeDefaultNetworkConfig("/config/networkconfig.txt");
          Serial.println(F("reset to default network config"));          
          
          sendNetworkConfigFlag = true;
        }
        
        // modif config
        // write object config
        if (writeObjectConfigFlag)
        {
          writeObjectConfig();
        
          // update statut
          uneFois = true;
        }
        
        // resend object config
        if (sendObjectConfigFlag)
        {
          sendObjectConfig();
        }
        
        // write network config
        if (writeNetworkConfigFlag)
        {
          writeNetworkConfig();
        }
        
        // resend network config
        if (sendNetworkConfigFlag)
        {
          sendNetworkConfig();
        }
    }
 
    // clear json buffer
    doc.clear();
}

void notFound(AsyncWebServerRequest *request)
{
    request->send(404, "text/plain", "Not found");
}

void convertStrToRGB(const char * source, uint8_t* r, uint8_t* g, uint8_t* b)
{ 
  uint32_t  number = (uint32_t) strtol( &source[1], NULL, 16);
  
  // Split them up into r, g, b values
  *r = number >> 16;
  *g = number >> 8 & 0xFF;
  *b = number & 0xFF;
}

void sendMaxLed()
{
  char toSend[20];
  snprintf(toSend, 20, "{\"maxLed\":%i}", NB_LEDS_MAX);
  
  ws.textAll(toSend);
}

void sendUptime()
{
  uint32_t now = millis() / 1000;
  uint16_t days = now / 86400;
  uint16_t hours = (now%86400) / 3600;
  uint16_t minutes = (now%3600) / 60;
  uint16_t seconds = now % 60;
    
  char toSend[100];
  snprintf(toSend, 100, "{\"uptime\":\"%id %ih %im %is\"}", days, hours, minutes, seconds);

  ws.textAll(toSend);
}

void sendStatut()
{
  char toSend[100];
  snprintf(toSend, 100, "{\"statutActuel\":%i}", aConfig.objectConfig.statutActuel); 

  ws.textAll(toSend);
}

void sendObjectConfig()
{
  aConfig.stringJsonFile("/config/objectconfig.txt", bufferToSend, BUFFERSENDSIZE);
  ws.textAll(bufferToSend);
}

void writeObjectConfig()
{
  aConfig.writeObjectConfig("/config/objectconfig.txt");
}

void sendNetworkConfig()
{
  aConfig.stringJsonFile("/config/networkconfig.txt", bufferToSend, BUFFERSENDSIZE);
  ws.textAll(bufferToSend);
}

void writeNetworkConfig()
{
  aConfig.writeNetworkConfig("/config/networkconfig.txt");
}
/*
   ----------------------------------------------------------------------------
   FIN DES FONCTIONS ADDITIONNELLES
   ----------------------------------------------------------------------------
*/
