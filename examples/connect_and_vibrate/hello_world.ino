/*
 * Copyright 2020 Neosensory, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please note that while this Neosensory SDK has an Apache 2.0 license,
 * usage of the Neosensory API to interface with Neosensory products is
 * still  subject to the Neosensory developer terms of service located at:
 * https://neosensory.com/legal/dev-terms-service/
 *
 * connect_and_vibrate.ino - Example for using
 * NeosensoryBluefruit library. Requires a Neosensory
 * Buzz and an Adafruit Feather board (or other board
 * that works with Adafruit's Bluefruit library).
 *
 * If you have trouble connecting, you can try putting
 * your Buzz into pairing mode by holding the (+) and (-)
 * buttons simultaneously until the blue LEDs flash.
 *
 * Created by Mike V. Perrotta, January 23, 2020.
 *
*/

/**
 * Define the number of slices per model window. E.g. a model window of 1000 ms
 * with slices per model window set to 4. Results in a slice size of 250 ms.
 * For more info: https://docs.edgeimpulse.com/docs/continuous-audio-sampling
 */
#define EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW 3

#include <neosensory_arduinoble.h>
#include <PDM.h>
#include <hello_world_keywords_inference.h>

/** Audio buffers, pointers and selectors */
typedef struct {
    signed short *buffers[2];
    unsigned char buf_select;
    unsigned char buf_ready;
    unsigned int buf_count;
    unsigned int n_samples;
} inference_t;

static inference_t inference;
static bool record_ready = false;
static signed short *sampleBuffer;
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal
static int print_results = -(EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW);

/*
 * If there are multiple Buzz devices in proximity,
 * you can supply a Device ID for a specific Buzz.
 * You can obtain this by connecting over USB (9600 Baud)
 * and running the command "device info"
 */
NeosensoryBluefruit NeoBluefruit;
// NeosensoryBluefruit NeoBluefruit("F2 AD 50 EA 96 31");

void onReadNotify(/*BLEClientCharacteristic* chr, */const uint8_t* data, uint16_t len);
void onConnected(bool success);
void onDisconnected();
void inference_loop();
static rtos::Thread inference_thread(osPriorityLow);

int motor = 0;
float intensity = 0;
float **rumble_frames;

void setup() {
  Serial.begin(115200);

  Serial.println("Hello world");

  for (size_t ix = 0;ix < 7; ix++){
    Serial.println(ix);
    delay(1000);
  }

  NeoBluefruit.begin();
  NeoBluefruit.setConnectedCallback(onConnected);
  NeoBluefruit.setDisconnectedCallback(onDisconnected);
  NeoBluefruit.setReadNotifyCallback(onReadNotify);
  NeoBluefruit.startScan();
  set_rumble_frames();

    // summary of inferencing settings (from model_metadata.h)
    ei_printf("Inferencing settings:\n");
    ei_printf("\tInterval: %.2f ms.\n", (float)EI_CLASSIFIER_INTERVAL_MS);
    ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
    ei_printf("\tSample length: %d ms.\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT / 16);
    ei_printf("\tNo. of classes: %d\n", sizeof(ei_classifier_inferencing_categories) /
                                            sizeof(ei_classifier_inferencing_categories[0]));

    run_classifier_init();
    if (microphone_inference_start(EI_CLASSIFIER_SLICE_SIZE) == false) {
        ei_printf("ERR: Failed to setup audio sampling\r\n");
        return;
    }

    inference_thread.start(&inference_loop);
}

static bool is_connected_and_auth = false;
static bool first_successful_loop = true;
const char *prediction;
static int last_heard_helloworld = 1000;

void loop() {
  if (!is_connected_and_auth) {
    delay(1000);
    return;
  }

  if (first_successful_loop) {
    Serial.println("first_successful_loop");
    delay(1000);
    first_successful_loop = false;
    NeoBluefruit.vibrateMotor(motor, 0.1);
  }

  if (last_heard_helloworld < 2) { /*heard helloworld in last two frames?*/
    NeoBluefruit.vibrateMotor(motor, 1);
  }
  else {
    NeoBluefruit.vibrateMotor(motor, 0);
  }
  delay(50);
}

void set_rumble_frames() {
  rumble_frames = new float*[NeoBluefruit.max_frames_per_bt_package()];
  for (int i = 0; i < NeoBluefruit.max_frames_per_bt_package(); i++) {
    rumble_frames[i] = new float[NeoBluefruit.num_motors()];
    for (int j = 0; j < NeoBluefruit.num_motors(); j++) {
      rumble_frames[i][j] = (i % 2) == (j % 2);
    }
  }
}

void rumble() {
  NeoBluefruit.vibrateMotors(rumble_frames, NeoBluefruit.max_frames_per_bt_package());
  delay((NeoBluefruit.max_frames_per_bt_package()) * NeoBluefruit.firmware_frame_duration());
}

/* Callbacks */

void onConnected(bool success) {
  if (!success) {
    Serial.println("Attempted connection but failed, is your device in Pairing mode?");
    return;
  }
  Serial.println("Hello from sketch!");

  // Once we are successfully connected to the wristband,
  // send developer autherization command and commands
  // to stop sound-to-touch algorithm.
  NeoBluefruit.authorizeDeveloper();
  NeoBluefruit.acceptTermsAndConditions();
  NeoBluefruit.stopAlgorithm();
  Serial.println("Done setting up, now fully connected");

  is_connected_and_auth = true;
}

void onDisconnected(uint16_t conn_handle, uint8_t reason) {
  Serial.println("\nDisconnected");
}

void onReadNotify(/*BLEClientCharacteristic* chr, */const uint8_t* data, uint16_t len) {
  for (int i = 0; i < len; i++) {
    Serial.write(data[i]);
  }
}


/**
 * @brief      Printf function uses vsnprintf and output using Arduino Serial
 *
 * @param[in]  format     Variable argument list
 */
void ei_printf(const char *format, ...) {
    static char print_buf[1024] = { 0 };

    va_list args;
    va_start(args, format);
    int r = vsnprintf(print_buf, sizeof(print_buf), format, args);
    va_end(args);

    if (r > 0) {
        Serial.write(print_buf);
    }
}

/**
 * @brief      PDM buffer full callback
 *             Get data and call audio thread callback
 */
static void pdm_data_ready_inference_callback(void)
{
    int bytesAvailable = PDM.available();

    // read into the sample buffer
    int bytesRead = PDM.read((char *)&sampleBuffer[0], bytesAvailable);

    if (record_ready == true) {
        for (int i = 0; i<bytesRead>> 1; i++) {
            inference.buffers[inference.buf_select][inference.buf_count++] = sampleBuffer[i];

            if (inference.buf_count >= inference.n_samples) {
                inference.buf_select ^= 1;
                inference.buf_count = 0;
                inference.buf_ready = 1;
            }
        }
    }
}

/**
 * @brief      Arduino main function. Runs the inferencing loop.
 */
void inference_loop()
{
    // This is a structure that smoothens the output result
    // With the default settings 70% of readings should be the same before classifying.
    ei_classifier_smooth_t smooth;
    ei_classifier_smooth_init(&smooth, 3 /* no. of readings */, 1 /* min. readings the same (> than, so this is 2) */, 0.6 /* min. confidence */, 0.0 /* max anomaly */);

  while (1) {
    bool m = microphone_inference_record();
    if (!m) {
        ei_printf("ERR: Failed to record audio...\n");
        return;
    }

    signal_t signal;
    signal.total_length = EI_CLASSIFIER_SLICE_SIZE;
    signal.get_data = &microphone_audio_signal_get_data;
    ei_impulse_result_t result = {0};

    EI_IMPULSE_ERROR r = run_classifier_continuous(&signal, &result, debug_nn);
    if (r != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", r);
        return;
    }

    //if (++print_results >= (EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW)) {
        // print the predictions
//        ei_printf("Predictions ");
//        ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
//            result.timing.dsp, result.timing.classification, result.timing.anomaly);
//        ei_printf(": \n");
//        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
//            ei_printf("    %s: %.5f\n", result.classification[ix].label,
//                      result.classification[ix].value);
//        }
//#if EI_CLASSIFIER_HAS_ANOMALY == 1
//        ei_printf("    anomaly score: %.3f\n", result.anomaly);
//#endif

        // ei_classifier_smooth_update yields the predicted label
        prediction = ei_classifier_smooth_update(&smooth, &result);
        ei_printf("Result: %s ", prediction);
        // print the cumulative results
        ei_printf(" [ ");
        for (size_t ix = 0; ix < smooth.count_size; ix++) {
            ei_printf("%u", smooth.count[ix]);
            if (ix != smooth.count_size + 1) {
                ei_printf(", ");
            }
            else {
              ei_printf(" ");
            }
        }
        ei_printf("]\n");

        if (strcmp(prediction, "helloworld") == 0) {
          last_heard_helloworld = 0;
        }
        else {
          last_heard_helloworld++;
        }

        print_results = 0;
    //}
  }
}

/**
 * @brief      Init inferencing struct and setup/start PDM
 *
 * @param[in]  n_samples  The n samples
 *
 * @return     { description_of_the_return_value }
 */
static bool microphone_inference_start(uint32_t n_samples)
{
    inference.buffers[0] = (signed short *)malloc(n_samples * sizeof(signed short));

    if (inference.buffers[0] == NULL) {
        return false;
    }

    inference.buffers[1] = (signed short *)malloc(n_samples * sizeof(signed short));

    if (inference.buffers[0] == NULL) {
        free(inference.buffers[0]);
        return false;
    }

    sampleBuffer = (signed short *)malloc((n_samples >> 1) * sizeof(signed short));

    if (sampleBuffer == NULL) {
        free(inference.buffers[0]);
        free(inference.buffers[1]);
        return false;
    }

    inference.buf_select = 0;
    inference.buf_count = 0;
    inference.n_samples = n_samples;
    inference.buf_ready = 0;

    // configure the data receive callback
    PDM.onReceive(&pdm_data_ready_inference_callback);

    // optionally set the gain, defaults to 20
    PDM.setGain(80);

    PDM.setBufferSize((n_samples >> 1) * sizeof(int16_t));

    // initialize PDM with:
    // - one channel (mono mode)
    // - a 16 kHz sample rate
    if (!PDM.begin(1, EI_CLASSIFIER_FREQUENCY)) {
        ei_printf("Failed to start PDM!");
    }

    record_ready = true;

    return true;
}

/**
 * @brief      Wait on new data
 *
 * @return     True when finished
 */
static bool microphone_inference_record(void)
{
    bool ret = true;

    if (inference.buf_ready == 1) {
        ei_printf(
            "Error sample buffer overrun. Decrease the number of slices per model window "
            "(EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW)\n");
        ret = false;
    }

    while (inference.buf_ready == 0) {
        delay(1);
    }

    inference.buf_ready = 0;

    return ret;
}

/**
 * Get raw audio signal data
 */
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr)
{
    numpy::int16_to_float(&inference.buffers[inference.buf_select ^ 1][offset], out_ptr, length);

    return 0;
}

/**
 * @brief      Stop PDM and release buffers
 */
static void microphone_inference_end(void)
{
    PDM.end();
    free(inference.buffers[0]);
    free(inference.buffers[1]);
    free(sampleBuffer);
}
