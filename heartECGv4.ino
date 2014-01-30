/*
 * This version writes voltage data to Android every 0.5s
 */
#include <SoftwareSerial.h>
#include "foneastrapins.h"
#include <TimerOne.h>

#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8))
#define BUFFER 0
#define OTHER_BUFFER 1

const uint8_t SYNC_BYTE = 0xaa; //10101010
const int NO_OF_SYNC_BYTES = 10;
const int PAYLOAD_SIZE = 128; //(bytes)
const int MAX = NO_OF_SYNC_BYTES + PAYLOAD_SIZE;
const int NUM_SAMPLE = 50;
const int SAMPLING_INTERVAL = 4000; //millisecs.
//payload structure of MT_SINGLE_READING is: 
//MSG_TYPE (1 byte), SEQ_NO (2 bytes), MSG_DATA (PAYLOAD_SIZE - 4 bytes) CRC (1 byte)

const int PAYLOAD_BEGIN_INDEX = NO_OF_SYNC_BYTES;
const int MSG_DATA_BEGIN_INDEX = PAYLOAD_BEGIN_INDEX + 3;

//message types
const int MSG_READING = 1; //50 voltage readings per msg

SoftwareSerial dbgSerial(SW_UART_RX_PIN, SW_UART_TX_PIN);

uint8_t dataArray[MAX];

uint8_t buffer[NUM_SAMPLE * 2]; // NUM_SAMPLE * 2 because each sample requires 10 bytes
uint8_t other_buffer[NUM_SAMPLE * 2];

uint8_t msgType = MSG_READING;

uint8_t *volt_buf_ptr = buffer; // this pointer is used in the ISR to fill voltage values
uint8_t volt_buf = BUFFER; // To keep track of the buffer identity

uint16_t vol = 0;
uint8_t count = 0;
uint16_t seq_counter = 0;
bool buf_ready = true; // true when the buffer is empty
bool other_buf_ready = true; // same as above

uint8_t uni_counter = 0;

uint8_t t1[10];
uint8_t t2[10];
uint8_t* ptr;


void setup() {

  dbgSerial.begin(57600); 

  
  //set up the HW UART to communicate with the BT module
  
  // initialization baudrate. Different FA devices use different baudrates
  Serial.begin(115200);

  pinMode(A3_PIN, INPUT); // set as input to detect voltage - values
  pinMode(A7_PIN, INPUT); // set as input to detect voltage - touched/untouched

  pinMode(BT_PWR_PIN,OUTPUT); // bluetooth power pin
  pinMode(RED_LED_PIN, OUTPUT); // red LED light

  // fill data packet with sync bytes
  memset(dataArray,0,sizeof(dataArray));
  for(int i = 0;i < PAYLOAD_BEGIN_INDEX;i++) {
    dataArray[i]=SYNC_BYTE;
  }

  // turn on bluetooth
  digitalWrite(BT_PWR_PIN,HIGH);
  digitalWrite(RED_LED_PIN,LOW);
  
  Timer1.initialize(SAMPLING_INTERVAL); // set the timer to go off every 4 millisecond
  Timer1.attachInterrupt( timerIsr ); // attach the interrupt service routine here
  
}

void loop() {
  
  dbgSerial.print("HIHIHIIIHIHIIHI"); 

  if (digitalRead(2) == HIGH) {
    
    // check if any buffer needs to be written
    if (!buf_ready || !other_buf_ready) {
      
      if (!buf_ready) {
        // copy the voltage values to dataArray
        memcpy(&dataArray[MSG_DATA_BEGIN_INDEX], buffer, sizeof(buffer));
        // mark the buffer ready
        buf_ready = true;
      } else {
        // copy the voltage values to dataArray
        memcpy(&dataArray[MSG_DATA_BEGIN_INDEX], other_buffer, sizeof(other_buffer));
        // mark the buffer ready
        other_buf_ready = true;
      }
      ++seq_counter;
        
      dataArray[PAYLOAD_BEGIN_INDEX] = msgType;
      memcpy(&dataArray[PAYLOAD_BEGIN_INDEX + 1], &seq_counter,sizeof(seq_counter));
      
      //uint8_t crc = getCRC(dataArray + PAYLOAD_BEGIN_INDEX);
      uint8_t crc = getCRC(dataArray); // CRC actually starts from PAYLOAD_BEGIN_INDEX, in other words, excluding the sync bytes
      dataArray[MAX -1] = crc;	
      
      Serial.write(dataArray,sizeof(dataArray));
      //Serial.println();
      //Serial.print("seq #: ");
      //Serial.println(dataArray[PAYLOAD_BEGIN_INDEX + 1]);
      //Serial.print("write out dataArray: ");
      //Serial.println(dataArray[MSG_DATA_BEGIN_INDEX + 50]);
      Serial.flush();
    }
  }
}


uint8_t getCRC(uint8_t *buff) {
	uint8_t crc = 0;

        for(int i = PAYLOAD_BEGIN_INDEX; i< MAX-1; i++) {
	//for(int i = 0; i< MAX-1; i++) {
		crc ^= buff[i]; 
	}
	return crc;
}

/// --------------------------
/// ISR Timer Routine
/// Read voltage from A3
/// --------------------------
void timerIsr()
{
    // one of the buffers should be available every time we enter ISR
    //if (analogRead(A7_PIN) == 0) {
    if (digitalRead(2) == HIGH && analogRead(A7_PIN) == 0) {
      // check BT is on and fingers are touching the pads
      
      vol = analogRead(A3_PIN);
      volt_buf_ptr[count++] = lowByte(vol);
      volt_buf_ptr[count++] = highByte(vol);

/*      // testing purpose
      volt_buf_ptr[count++] = uni_counter;
      volt_buf_ptr[count++] = 0;

      uni_counter++;
      
      if (uni_counter >= 125) {
        uni_counter = 0;
      }
*/
      // if the buffer is full, switch to another one
      if (count >= NUM_SAMPLE * 2) {
        if (volt_buf == BUFFER) {
          buf_ready = false;
          if (other_buf_ready == true) {
            //Serial.print("switching to other_buffer: ");
            volt_buf_ptr = other_buffer;
            //Serial.println(volt_buf_ptr[0]);
            volt_buf = OTHER_BUFFER;
          } else {
            //Serial.println("other_buffer not ready to switch to");
            dbgSerial.println("Other buffer not ready for ISR"); 
          }
        } else {
          other_buf_ready = false;
          if (buf_ready == true) {
            //Serial.print("switching to buffer: ");
            volt_buf_ptr = buffer;
            //Serial.println(volt_buf_ptr[0]);
            volt_buf = BUFFER;
          } else {
            //Serial.println("buffer not ready to switch to");
            dbgSerial.println("Buffer not ready for ISR"); 
          }
        }
        count = 0;        
      } 
    }
}

