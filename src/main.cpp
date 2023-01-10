#include <Arduino.h>
// #include <sensorDriver.h>
#include <AWS.h>

void taskOne( void * parameter);
void taskTwo( void * parameter);
// void sensorValuePublishTask(void * parameter);
void awsConnectionTask(void * parameter);
void motorControlTask(void * parameter);


#define LED_BOARD 2 //change here the pin of the board to V2
// myawsclass* awsobjectPtr;

void setup(){
  pinMode(LED_BOARD, OUTPUT);
  Serial.begin(9600);
  awsobject.connectAWS();
  Serial.println("Serial started");
  delay(1000);
  // awsobjectPtr = new myawsclass();  /* creating an object of class aws */


  // xTaskCreate(
  //                   taskOne,          /* Task function. */
  //                   "TaskOne",        /* String with name of task. */
  //                   1024,              /* Stack size in bytes. */
  //                   NULL,             /* Parameter passed as input of the task */
  //                   1,                /* Priority of the task. */
  //                   NULL);            /* Task handle. */

  // xTaskCreate(
  //                   taskTwo,          /* Task function. */
  //                   "TaskTwo",        /* String with name of task. */
  //                   1024,              /* Stack size in bytes. */
  //                   NULL,             /* Parameter passed as input of the task */
  //                   1,                /* Priority of the task. */
  //                   NULL);            /* Task handle. */

  // xTaskCreate(
  //                   sensorValuePublishTask,          /* Task function. */
  //                   "SensorValuePublishTask",        /* String with name of task. */
  //                   512,              /* Stack size in bytes. */
  //                   NULL,             /* Parameter passed as input of the task */
  //                   1,                /* Priority of the task. */
  //                   NULL);            /* Task handle. */
  
  xTaskCreate(
                    awsConnectionTask,          /* Task function. */
                    "AwsConnectionTask",        /* String with name of task. */
                    2048,              /* Stack size in bytes. */
                    NULL,             /* Parameter passed as input of the task */
                    2,                /* Priority of the task. */
                    NULL);            /* Task handle. */

  xTaskCreate(
                    motorControlTask,          /* Task function. */
                    "MotorControlTask",        /* String with name of task. */
                    4096,              /* Stack size in bytes. */
                    NULL,             /* Parameter passed as input of the task */
                    3,                /* Priority of the task. */
                    NULL);            /* Task handle. */

}

void loop(){
delay(1000);
}

void taskOne( void * parameter )
{
    //example of a task that executes for some time and then is deleted
    for( int i = 0; i < 10; i++ )
    {
      Serial.println("Hello from task 1");
      
      //Switch on the LED
      digitalWrite(LED_BOARD, HIGH); 
      // Pause the task for 1000ms
      //delay(1000); //This delay doesn't give a chance to the other tasks to execute
      vTaskDelay(1000 / portTICK_PERIOD_MS); //this pauses the task, so others can execute
      // Switch off the LED
      digitalWrite(LED_BOARD, LOW);
      // Pause the task again for 500ms
      vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    Serial.println("Ending task: 1");
    vTaskDelete( NULL );
}
 
void taskTwo( void * parameter)
{
    //create an endless loop so the task executes forever
    for( ;; )
    {
        Serial.println("Hello from task: 2");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    Serial.println("Ending task 2"); //should not reach this point but in case...
    vTaskDelete( NULL );
}

/**
 * sensorValuePublishTask
 * 
 * 
 * @brief this task is responsible to publish the sensor values to the AWS Greengrass IoT
*/
// void sensorValuePublishTask( void * parameter)
// {
//   sclass sensorobject = sclass(); /* creating an object of class sensor */
//   sensorobject.SETUP();

//   int16_t* sensorValue;

//   // create an endless loop so the task executes forever
//   for( ;; )
//   {
//     sensorValue = sensorobject.reading();
//     Serial.print("sensor values is: ");
//     Serial.println(*sensorValue);
//     vTaskDelay(1000 / portTICK_PERIOD_MS);
//   }
//   Serial.println("Ending sensor value Publish Task");
//   vTaskDelete(NULL);
// }

/**
 * 
 * awsConnectionTask
 * 
 * @brief this task is responsible for connecting the aws iot greengrass
*/
void awsConnectionTask(void * parameter)
{
  awsobject.stayConnected();
  vTaskDelete(NULL);
  Serial.println("Task aws completed");
}

/**
 * motorControlTask
 * 
 * @brief this task responsible for controlling the motors
*/
void motorControlTask(void * parameter)
{
  int16_t sensorValue = 10;
  for(;;) {
    Serial.println("publishing sensor value");
    awsobject.publishMessage(sensorValue);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  vTaskDelete(NULL);
  Serial.println("Task aws completed");

}
