/**
 * ESP32 AWS Library
 * 
 * Functions to get the crawler coordinates from the Camera over AWS IoT
 * 
 * Authors: Vipul Deshpande, Jaime Burbano
 */


/*
  Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
  Permission is hereby granted, free of charge, to any person obtaining a copy of this
  software and associated documentation files (the "Software"), to deal in the Software
  without restriction, including without limitation the rights to use, copy, modify,
  merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
  permit persons to whom the Software is furnished to do so.
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "secrets.h"
#include <WiFiClientSecure.h>
#include <MQTTClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"
#include "AWS.h"

/* The MQTT topics that this device should publish/subscribe to */
#define AWS_IOT_PUBLISH_TOPIC   "esp32/pub" 
#define AWS_IOT_SUBSCRIBE_TOPIC "esp32/target"

WiFiClientSecure net = WiFiClientSecure();
MQTTClient client = MQTTClient(256);

myawsclass::myawsclass() {

}


void messageHandler(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);

//  StaticJsonDocument<200> doc;
//  deserializeJson(doc, payload);
//  const char* message = doc["message"];
}

void myawsclass::stayConnected() {
  client.loop();
}

void myawsclass::connectAWS() {

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.println("Connecting to Wi-Fi");

  while (WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print("Connecting...!");
  }

  Serial.print("CONNECTED...!\n");

  /* Configure WiFiClientSecure to use the AWS IoT device credentials */
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);

  /* Connect to the MQTT broker on the AWS endpoint we defined earlier */
  client.begin(AWS_IOT_ENDPOINT, 8883, net);

  /* Create a message handler */
  client.onMessage(messageHandler);

  Serial.print("Connecting to AWS IOT");

  while (!client.connect(THINGNAME)) {
    Serial.print(".");
    delay(100);
  }

  if(!client.connected())
  {
    Serial.println("AWS IoT Timeout!");
    return;
  }

  /* Subscribe to a topic */
  client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);

  Serial.println("AWS IoT Connected!");
}

void myawsclass::publishMessage(int16_t sensorValue) {

  StaticJsonDocument<200> doc;
  //doc["time"] = millis();
  doc["sensor"] = sensorValue;
  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer); /* print to client */

  client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer);
}

myawsclass awsobject = myawsclass();  /* creating an object of class aws */


