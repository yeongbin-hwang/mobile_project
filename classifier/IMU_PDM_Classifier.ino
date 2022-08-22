/*************************
 * BLE communication
 **************************/
#include <ArduinoBLE.h> // Arduino Standard BLE Library

typedef struct EmergencyStatus {
  byte flag = 0b00000000; // Refer to the "GATT Specification Supplement" for more detail
  byte EmergencyStatus_value[4]; // Byte-array used instead of float, to resolve 4-byte aligning issue
} EmergencyStatus;
EmergencyStatus itm;

BLEService HTService("183C"); // UUID of "Emergency Configuration" is 1809


BLECharacteristic EmergencyStatus_value("2A1E", BLENotify, sizeof(itm), true); // UUID of "Temperature Measurement Characteristic" is 2A1C. 
byte cccd_value_2[2]= {0b00000000, 0b00000001};
BLEDescriptor cccd_2("2902", cccd_value_2, 2);

/********************************* 
 *  IMU classifier 
***********************************/
#include <Arduino_LSM9DS1.h>

#include <TensorFlowLite.h>
#include <tensorflow/lite/micro/all_ops_resolver.h>
#include <tensorflow/lite/micro/micro_error_reporter.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/schema/schema_generated.h>
#include <tensorflow/lite/version.h>
#include "model.h"

int shake_num;
int shake_alarm;

uint32_t timer;
uint32_t init_timer;

const float accelerationThreshold = 2.5; // threshold of significant in G's
const int numSamples = 119;

int samplesRead = numSamples;

// global variables used for TensorFlow Lite (Micro)
tflite::MicroErrorReporter tflErrorReporter;
tflite::AllOpsResolver tflOpsResolver;

const tflite::Model* tflModel = nullptr;
tflite::MicroInterpreter* tflInterpreter = nullptr;
TfLiteTensor* tflInputTensor = nullptr;
TfLiteTensor* tflOutputTensor = nullptr;

constexpr int tensorArenaSize = 8 * 1024;
byte tensorArena[tensorArenaSize] __attribute__((aligned(16)));

  // array to map gesture index to a name
const char* GESTURES[] = {
  "hands-up",
  "rotate",
  "shake",
};

#define NUM_GESTURES (sizeof(GESTURES) / sizeof(GESTURES[0]))

/********************************* 
 *  voice classifier 
***********************************/
#include <PDM.h>
#include "tf_lite_model.h"       // TF Lite model file


#define PDM_SOUND_GAIN     255   // sound gain of PDM mic
#define PDM_BUFFER_SIZE    256   // buffer size of PDM mic

#define SAMPLE_THRESHOLD   700   // RMS threshold to trigger sampling
#define FEATURE_SIZE       32    // sampling size of one voice instance
#define SAMPLE_DELAY       20    // delay time (ms) between sampling

const char* VOICES[] = {
  "help",
  "ok",
  "no",
};  // words for each label
#define NUM_VOICES (sizeof(VOICES) / sizeof(VOICES[0]))

#define PREDIC_THRESHOLD   0.6   // prediction probability threshold for labels
#define RAW_OUTPUT         true  // output prediction probability of each label
#define NUMBER_OF_INPUTS   FEATURE_SIZE
#define NUMBER_OF_OUTPUTS  NUMBER_OF_LABELS
#define TENSOR_ARENA_SIZE  4 * 1024

float feature_data[FEATURE_SIZE];
volatile float rms;
bool voice_detected;
bool emergency_detected;

// global variables used for TensorFlow Lite (Micro)
tflite::MicroErrorReporter tflErrorReporter_2;
tflite::AllOpsResolver tflOpsResolver_2;

const tflite::Model* tflModel_2 = nullptr;
tflite::MicroInterpreter* tflInterpreter_2 = nullptr;
TfLiteTensor* tflInputTensor_2 = nullptr;
TfLiteTensor* tflOutputTensor_2 = nullptr;

constexpr int tensorArenaSize_2 = 4 * 1024;
byte tensorArena_2[tensorArenaSize_2] __attribute__((aligned(16)));

void setup() {
  Serial.begin(9600);
  while (!Serial);
  
  // initialize the PDM
  PDM.onReceive(onPDMdata);
  PDM.setBufferSize(PDM_BUFFER_SIZE);
  PDM.setGain(PDM_SOUND_GAIN);

  if (!PDM.begin(1, 16000)) {  // start PDM mic and sampling at 16 KHz
    Serial.println("Failed to start PDM!");
    //while (1);
  }
  // initialize the IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  if(!BLE.begin())
  {
    Serial.println("BLE start failed?");
  }
  
  // wait 1 second to avoid initial PDM reading
  delay(1000);
  
  // print out the samples rates of the IMUs
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  
  /***************************************************/
  /* imu model */
  /***************************************************/
  // get the TFL representation of the model byte array
  tflModel = tflite::GetModel(model);
  if (tflModel->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("Model schema mismatch!");
    while (1);
  }

  // Create an interpreter to run the model
  tflInterpreter = new tflite::MicroInterpreter(tflModel, tflOpsResolver, tensorArena, tensorArenaSize, &tflErrorReporter);

  // Allocate memory for the model's input and output tensors
  tflInterpreter->AllocateTensors();

  // Get pointers for the model's input and output tensors
  tflInputTensor = tflInterpreter->input(0);
  tflOutputTensor = tflInterpreter->output(0);

  /***************************************************/
  /* voice model */
  /***************************************************/
  
  // get the TFL representation of the model byte array
  tflModel_2 = tflite::GetModel(model_data);
  if (tflModel_2->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("Model schema mismatch!");
    while (1);
  }
  // Create an interpreter to run the model
  tflInterpreter_2 = new tflite::MicroInterpreter(tflModel_2, tflOpsResolver_2, tensorArena_2, tensorArenaSize_2, &tflErrorReporter_2);

  // Allocate memory for the model's input and output tensors
  tflInterpreter_2->AllocateTensors();

  // Get pointers for the model's input and output tensors
  tflInputTensor_2 = tflInterpreter_2->input(0);
  tflOutputTensor_2 = tflInterpreter_2->output(0);

  /***************************************************/
  /* BLE communication */
  /***************************************************/
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);
  
  // Set "Generic Access Profile" characteristics  
  BLE.setLocalName("EE595");
  BLE.setDeviceName("MyDevice"); // UUID 2A00
  BLE.setAppearance(0x0543); // UUID 2A01

  // Characteristics with "Notify" property has a CCCD descriptor already installed. To change the value, the only possible way is to completely replace it
  EmergencyStatus_value.descriptor("2902") = cccd_2;
  HTService.addCharacteristic(EmergencyStatus_value); // Optional. Refer to the "Health Thermometer Service" document for more detail

  // Install service to the device
  BLE.addService(HTService);

  // There can be many services installed. Choose one service to advertise
  BLE.setAdvertisedService(HTService);                                                                                                                                                                                                                                                                                                                                                                   
  BLE.advertise();

  Serial.println("Waiting for connection ...");

  shake_num = 0;
  shake_alarm = 0;

  init_timer = micros();
  timer = micros();
}

void loop() {
  // This device will be a "peripheral"(or a GATT Server), and a "central"(or a GATT Client) will be connected
  BLEDevice central = BLE.central();
  
  float aX, aY, aZ, gX, gY, gZ;

  int PDM_active;
  PDM_active = 0;
  if (central){
  //if(true){
    double ddt = (double)(micros() - init_timer) / 1000000;
    if(ddt < 10){
      return;
    }
    // wait for significant motion
    while (samplesRead == numSamples) {
      if (IMU.accelerationAvailable()) {
        // read the acceleration data
        IMU.readAcceleration(aX, aY, aZ);
  
        // sum up the absolutes
        float aSum = fabs(aX) + fabs(aY) + fabs(aZ);
  
        double dt = (double)(micros() - timer) / 1000000;
        
        // emergency detect (voice + motion)
        if (emergency_detected){
          Serial.println("emergency detection (voice + motion)");
          delay(1000);
          // Set the values for the "Intermediate Temperature Characteristic"
          itm.flag = 0b00000000;
          *(float*)&itm.EmergencyStatus_value = 10; // Use real thermometer value. User-defined data can be used instead
      
          // Write the data, which will automatically notify the "central"(client)
          EmergencyStatus_value.writeValue((uint8_t*)&itm, sizeof(itm));
          emergency_detected = false;
        } // motion  detect (motion)
        else if(shake_alarm == 1 && dt > 5){
          Serial.println("emergency detection (motion)");
          delay(1000);
          // Set the values for the "Intermediate Temperature Characteristic"
          itm.flag = 0b00000000;
          *(float*)&itm.EmergencyStatus_value = 20; // Use real thermometer value. User-defined data can be used instead
      
          // Write the data, which will automatically notify the "central"(client)
          EmergencyStatus_value.writeValue((uint8_t*)&itm, sizeof(itm));
          shake_alarm = 0;
          shake_num = 0;
          timer = micros();
        }
        
        // when voice detect(voice first recognizition).
        if (rms > SAMPLE_THRESHOLD){
          Serial.println("voice detection"); 
          PDM_active = 1;
          shake_num = 0;
          break;
        }
        // check if it's above the threshold
        if (aSum >= accelerationThreshold) {
          // reset the sample read count
          samplesRead = 0;
          break;
          
        }
      }
    }
    // classifier
    // voice activation
    if(PDM_active == 1){
      digitalWrite(LED_BUILTIN, HIGH);
      for (int i = 0; i < FEATURE_SIZE; i++) {  // sampling
        while (rms < 0);
        feature_data[i] = rms;
        delay(SAMPLE_DELAY);
      }
      
      for (int i = 0; i < FEATURE_SIZE; i++) {  
        tflInputTensor_2->data.f[i] = feature_data[i];
      }
  
      // predict the value
      TfLiteStatus invokeStatus = tflInterpreter_2->Invoke();
      if (invokeStatus != kTfLiteOk) {
        Serial.println("Invoke failed!");
        while (1);
        return;
      }
      // print out prediction results;
      Serial.println("Predicting the word:");
      for (int i = 0; i < NUM_VOICES; i++) {
        Serial.print("Label ");
        Serial.print(i);
        Serial.print(" = ");
        Serial.println(tflOutputTensor_2->data.f[i],6);
      }
      
      voice_detected = false;
      for (int i = 0; i < NUM_VOICES; i++) {
        if (tflOutputTensor_2->data.f[i] >= PREDIC_THRESHOLD) {
          Serial.print("Word detected: ");
          Serial.println(VOICES[i]);
          Serial.println("");
          voice_detected = true;
          timer = micros();
        }
      }
      if (!voice_detected) Serial.println("Word not recognized\n");
    
      // wait for 1 second after one sampling/prediction
      delay(1000);
      PDM_active = 0;
    }// IMU activation
    else{
      while (samplesRead < numSamples) {
        if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
          IMU.readAcceleration(aX, aY, aZ);
          IMU.readGyroscope(gX, gY, gZ);
    
          // normalize the IMU data between 0 to 1 and store in the model's
          // input tensor
          tflInputTensor->data.f[samplesRead * 6 + 0] = (aX + 4.0) / 8.0;
          tflInputTensor->data.f[samplesRead * 6 + 1] = (aY + 4.0) / 8.0;
          tflInputTensor->data.f[samplesRead * 6 + 2] = (aZ + 4.0) / 8.0;
          tflInputTensor->data.f[samplesRead * 6 + 3] = (gX + 2000.0) / 4000.0;
          tflInputTensor->data.f[samplesRead * 6 + 4] = (gY + 2000.0) / 4000.0;
          tflInputTensor->data.f[samplesRead * 6 + 5] = (gZ + 2000.0) / 4000.0;
    
          samplesRead++;
    
          if (samplesRead == numSamples) {
            // Run inferencing
            double dt = (double)(micros() - timer) / 1000000;
            if(dt > 4){
              shake_num = 0;
              shake_alarm = 0;
              voice_detected = false;
            }
            
            TfLiteStatus invokeStatus = tflInterpreter->Invoke();
            if (invokeStatus != kTfLiteOk) {
              Serial.println("Invoke failed!");
              while (1);
              return;
            }
    
            // Loop through the output tensor values from the model
            double max_value = 0;
            int max_num = 0;
            for (int i = 0; i < NUM_GESTURES; i++) {
              Serial.print(GESTURES[i]);
              Serial.print(": ");
              Serial.println(tflOutputTensor->data.f[i], 6);
              if(max_value < tflOutputTensor->data.f[i]){
                max_value = tflOutputTensor->data.f[i];
                max_num = i;
              }
            }
            // voice detection and hands-up
            if (voice_detected && max_num == 0 && max_value > 0.7){
              emergency_detected = true;
              voice_detected = false;
            }
            // if select shake
            else if(max_num == 2 && max_value > 0.7){
              shake_num = shake_num + 1;
            }
            else{
              shake_num = 0;
            }
    
            // when 5 shake detect
            if(shake_num == 5){
              shake_alarm = 1;
            }
            else if(shake_num > 5){
              shake_alarm = 0;
            }
            Serial.println(shake_num);
            Serial.println();
            timer = micros();
          }
        }
      } 
    }
  }
  else{
    Serial.println("BLE not connected");
  }
}

// callback function for PDM mic
void onPDMdata() {

  rms = -1;
  short sample_buffer[PDM_BUFFER_SIZE];
  int bytes_available = PDM.available();
  PDM.read(sample_buffer, bytes_available);

  // calculate RMS (root mean square) from sample_buffer
  unsigned int sum = 0;
  for (unsigned short i = 0; i < (bytes_available / 2); i++) sum += pow(sample_buffer[i], 2);
  rms = sqrt(float(sum) / (float(bytes_available) / 2.0));
}
// Simple handler for the connect event
void blePeripheralConnectHandler(BLEDevice central) {
  // central connected event handler
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
}

// Simple handler for the disconnect event
void blePeripheralDisconnectHandler(BLEDevice central) {
  // central disconnected event handler
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
}
