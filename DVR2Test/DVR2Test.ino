#define DEBUG_MODE 1
#ifdef DEBUG_MODE && DEBUG_MODE == 1
#define DPrint(fmt, ...) Serial.print(fmt, ##__VA_ARGS__);
#define DPrintln(fmt, ...) Serial.println(fmt, ##__VA_ARGS__);
#else
#define DPrint(...)
#define DPrintln(...)
#endif

#define DELAY_TIME 10 // unit: ms
#define BUFFER_SIZE 10

// Output Pin (LED 燈號)
#define GPS_TX_LED_OUTPIN 2
#define WARNING_RX_LED_OUTPIN 3
#define DIRECTION_LIGHT_LED_OUTPIN 4

// Input Pin
#define DIRECTION_LIGHT_CHECK_INPIN 5
#define START_SWITCH_INPIN 6 // GPS和方向燈 啟動/關閉
#define BUZZER_SWITCH_INPIN 7 // 蜂鳴器 啟動/關閉

boolean initFlag = false;
unsigned char gpsCount = 0, gprmcCount = 0, checkFrequencyCount = 0, buzzerCount = 0;
int buzzerSwitchStatus = 0, startSwitchStatus = 0;
unsigned long cycleTime = 0, oldTime = 0;

char recvBuffer[BUFFER_SIZE] = {0}; // RX LDW or FCW data
char warningMessage_1[7] = {'$', 'M', 'W', 'L', 'D', 'T', ','};   // $MWLDT,
char warningMessage_2[7] = {'$', 'M', 'W', 'F', 'C', 'W', ','};   // $MWFCW,
char warningMessage_3[5] = {'A', 'T', '$', 'P', 'D'};             // AT$PD=21;LDW:1<0x0D><0x0A>

unsigned char Period[3] = {0, 0, 0};
unsigned char Pos[2] = {0, 0};
unsigned char Neg[2] = {0, 0};

/*
$GPGGA,121252.000,3937.3032,N,11611.6046,E,1,05,2.0,45.9,M,-5.7,M,,0000*77 
$GPRMC,121252.000,A,3958.3032,N,11629.6046,E,15.15,359.95,070306,,,A*54 
$GPVTG,359.95,T,,M,15.15,N,28.0,K,A*04 
$GPGGA,121253.000,3937.3090,N,11611.6057,E,1,06,1.2,44.6,M,-5.7,M,,0000*72 
$GPGSA,A,3,14,15,05,22,18,26,,,,,,,2.1,1.2,1.7*3D 
$GPGSV,3,3,10,29,07,074,,30,07,163,28*7D

$GPRMC,<1>,<2>,<3>,<4>,<5>,<6>,<7>,<8>,<9>,<10>,<11>,<12>*hh
<7> 地面速率（000.0~999.9節，前面的0也將被傳輸）
*/

/* 測速照相
$GPRMC,05414.00,A,2350.7912,N,12029.0616,E,44.444,175.22,180517,,,A*6C
$GPRMC,05414.00,A,2450.7384,N,12029.0568,E,44.444,175.22,180517,,,A*6C
$GPRMC,05414.00,A,2450.5500,N,12029.05200,E,44.444,175.22,180517,,,A*6C
$GPRMC,05414.00,A,2450.4966,N,12029.0496,E,44.444,175.22,180517,,,A*6C
$GPRMC,05414.00,A,2450.3316,N,12029.05320,E,44.444,175.22,180517,,,A*6C
*/
// 53.4558(7) * 1.852 = 99 KM/H
char *gprmcStr[5] = {"$GPRMC,05414.00,A,2350.7912,N,12029.0616,E,53.4558,175.22,180517,,,A*6C\r\n",
                     "$GPRMC,05414.00,A,2450.7384,N,12029.0568,E,53.4558,175.22,180517,,,A*6C\r\n",
                     "$GPRMC,05414.00,A,2450.5500,N,12029.05200,E,53.4558,175.22,180517,,,A*6C\r\n",
                     "$GPRMC,05414.00,A,2450.4966,N,12029.0496,E,53.4558,175.22,180517,,,A*6C\r\n",
                     "$GPRMC,05414.00,A,2450.3316,N,12029.05320,E,53.4558,175.22,180517,,,A*6C\r\n"};

void setup()
{
  setupInit();
}

void loop()
{
  /***** Initial Flow *****/
  initFlow(); // only run once 

  /***** Check buzzer switch and start switch on off *****/
  checkSwitchStatus();
  
  if (buzzerSwitchStatus == HIGH && startSwitchStatus == LOW) // buzzer off, start on
  {
    /***** Send GPS to DVR *****/
    sendGPSData(); // 1s, 2s

    /***** Receive RX data from UART1 *****/
    receiveWarning(); // 10 ms

    /***** Spend time for one loop cycle *****/
    calculateCycleTime();
    DPrint("Cycle Time = ");
    DPrintln(cycleTime);

    /****** Check direction light frequency ******/
    int sensorVal_1 = digitalRead(DIRECTION_LIGHT_CHECK_INPIN); 
    float lowValueOfFrequency = 0.6; // unit: Hz
    float highValueOfFrequency = 1.5; // unit: Hz
    checkFrequencyRange(lowValueOfFrequency, highValueOfFrequency, sensorVal_1); // 100 ms
    
    delay(DELAY_TIME);
  }
  else if (buzzerSwitchStatus == LOW && startSwitchStatus == HIGH) // buzzer on, start off
  {
    digitalWrite(WARNING_RX_LED_OUTPIN, HIGH);
    
    delay(500);
  }
  else  // buzzer and start on, or buzzer and start off
  {
    delay(500);
  }
}

void setupInit()
{
  // Input Pin
  pinMode(BUZZER_SWITCH_INPIN, INPUT_PULLUP);
  pinMode(START_SWITCH_INPIN, INPUT_PULLUP);
  pinMode(DIRECTION_LIGHT_CHECK_INPIN, INPUT_PULLUP);

  // Output Pin
  pinMode(GPS_TX_LED_OUTPIN, OUTPUT);
  pinMode(WARNING_RX_LED_OUTPIN, OUTPUT);
  pinMode(DIRECTION_LIGHT_LED_OUTPIN, OUTPUT);

  Serial.begin(9600);   // USB Debug
  Serial1.begin(9600);  // UART Communication
}

void initFlow()
{
  if (initFlag == false)
  {
    digitalWrite(GPS_TX_LED_OUTPIN, LOW);
    digitalWrite(WARNING_RX_LED_OUTPIN, LOW);
    digitalWrite(DIRECTION_LIGHT_LED_OUTPIN, LOW);
    delay(200);
    
    for (int i = 0; i < 3 ; i++) 
    {
      digitalWrite(GPS_TX_LED_OUTPIN, HIGH);
      digitalWrite(WARNING_RX_LED_OUTPIN, HIGH);
      digitalWrite(DIRECTION_LIGHT_LED_OUTPIN, HIGH);
      delay(200);
      digitalWrite(GPS_TX_LED_OUTPIN, LOW);
      digitalWrite(WARNING_RX_LED_OUTPIN, LOW);
      digitalWrite(DIRECTION_LIGHT_LED_OUTPIN, LOW);
      delay(200);
    }

    oldTime = millis();

    buzzerSwitchStatus = digitalRead(BUZZER_SWITCH_INPIN);
    startSwitchStatus = digitalRead(START_SWITCH_INPIN);
    initFlag = true;
  }
}

void calculateCycleTime()
{
  unsigned long newTime = millis();
  cycleTime = newTime - oldTime;
  oldTime = newTime;
}

void checkSwitchStatus()
{
  int newBuzzerSwitchStatus = digitalRead(BUZZER_SWITCH_INPIN);
  int newStartSwitchStatus = digitalRead(START_SWITCH_INPIN);
  if (newBuzzerSwitchStatus != buzzerSwitchStatus || newStartSwitchStatus != startSwitchStatus)
  {
    // Output Pin reset
    digitalWrite(GPS_TX_LED_OUTPIN, LOW);
    digitalWrite(DIRECTION_LIGHT_LED_OUTPIN, LOW);
    digitalWrite(WARNING_RX_LED_OUTPIN, LOW);

    // Count reset
    gpsCount = 0;
    checkFrequencyCount = 0;
    buzzerCount = 0;

    // Other reset
    Period[0] = 0, Period[1] = 0, Period[2] = 0;
    Pos[0] = 0, Pos[1] = 0;
    Neg[0] = 0, Neg[1] = 0;

    // record new value
    buzzerSwitchStatus = newBuzzerSwitchStatus;
    startSwitchStatus = newStartSwitchStatus;
  }
}

void sendGPSData()
{
  // Send GPS Data (ones per second)
  gpsCount++;
  if (gpsCount == 100) // 100 * DELAY_TIME = 1000 ms = 1 s
  {
    DPrintln("Send GPS Data:");
    DPrintln("----------------------------------------------------------------------");
    DPrint("$GPGGA,160530.00,2404.04359,N,12025.99974,E,1,06,2.28,-13.3,M,16.1,M,,*4E\r\n");
    Serial1.print("$GPGGA,160530.00,2404.04359,N,12025.99974,E,1,06,2.28,-13.3,M,16.1,M,,*4E\r\n");
    DPrintln("----------------------------------------------------------------------");

    // GPS LED
    digitalWrite(GPS_TX_LED_OUTPIN, HIGH);
  } 
  else if (gpsCount >= 200) // 200 * DELAY_TIME = 2000 ms = 2s
  {
    DPrintln("Send GPS Data:");
    DPrintln("----------------------------------------------------------------------");
    // 測速照相測試，車速 99 km/h
    DPrint(gprmcStr[gprmcCount]);
    Serial1.print(gprmcStr[gprmcCount]);
    gprmcCount++;
    if (gprmcCount >= 5)
    {
      gprmcCount = 0;
    }
    DPrintln("----------------------------------------------------------------------");

    // GPS LED
    digitalWrite(GPS_TX_LED_OUTPIN, HIGH);
    gpsCount = 0;
  }
  else if (gpsCount == 125) // 125 * DELAY_TIME = 1250 ms = 1.25 s
  {
    // GPS LED
    digitalWrite(GPS_TX_LED_OUTPIN, LOW);
  }
  else if (gpsCount == 25) // 25 * DELAY_TIME = 250 ms = 0.25 s
  {
    // GPS LED
    digitalWrite(GPS_TX_LED_OUTPIN, LOW);
  }
}

void receiveWarning()
{
  // $MWLDT,+0*XX
  // $MWFCW,0,0
  // AT$PD=21;LDW:1<0x0D><0x0A>
  int count = 0;
  int str1Length = sizeof(warningMessage_1) / sizeof(char);
  int str2Length = sizeof(warningMessage_2) / sizeof(char);
  int str3Length = sizeof(warningMessage_3) / sizeof(char);
  
  buzzerCount++;
  if (buzzerCount > 50) // 50 * 10 = 500 ms
  {
    digitalWrite(WARNING_RX_LED_OUTPIN, LOW); 
  }
  
  if (Serial1.available() > 0)
  {
    recvBuffer[count] = Serial1.read();
    Serial.write(recvBuffer[count]);
    
    // new warning message+
    if (recvBuffer[count] == 'A') 
    {
      count++;
      
      while (Serial1.available() > 0 && count < str3Length)
      {
        recvBuffer[count] = Serial1.read();
        Serial.write(recvBuffer[count]);
        count++;
      }
      
      // AT$PD=21;LDW:1<0x0D><0x0A>
      count = 0;
      for (int i = 0; i < str3Length; i++)
      {
        if (recvBuffer[i] == warningMessage_3[i])
          count++;
      }

      if (count == str3Length) {
        digitalWrite(WARNING_RX_LED_OUTPIN, HIGH);
        DPrintln();
        DPrintln("Success: Get warning message");
        buzzerCount = 0;
      }
    }
    // new warning message-
    
    // old warning message+    
//    if (recvBuffer[count] == '$')
//    {
//      count++;
//
//      while (Serial1.available() > 0 && count < max(str1Length, str2Length))
//      {
//        recvBuffer[count] = Serial1.read();
//        Serial.write(recvBuffer[count]);
//        count++;
//      }
//
//      // $MWLDT,
//      count = 0;
//      for (int i = 0; i < str1Length; i++) 
//      {
//        if (recvBuffer[i] == warningMessage_1[i])
//          count++;
//      }
//      
//      if (count == str1Length) {
//        digitalWrite(WARNING_RX_LED_OUTPIN, HIGH);
//        buzzerCount = 0;
//      }
//
//      // $MWFCW,
//      count = 0;
//      for (int i = 0; i < str2Length; i++) 
//      {
//        if (recvBuffer[i] == warningMessage_2[i])
//          count++;
//      }
//      
//      if (count == str2Length) {
//        digitalWrite(WARNING_RX_LED_OUTPIN, HIGH);
//        buzzerCount = 0;
//      }
//    }
    // old warning message-
  }
}

void checkFrequencyRange(float lowValue, float highValue, int sensorVal)
{
  float lowerBound = ((1 / highValue) * 1000 / DELAY_TIME);
  float upperBound = ((1/ lowValue) * 1000 / DELAY_TIME);

  // Count
  if (sensorVal == LOW)
  {
    Neg[0] = Neg[0] + 1;
    if (Pos[0] > 0)
      Pos[1] = 1;     
  }
  else // HIGH
  {
    Pos[0] = Pos[0] + 1;
    if (Neg[0] > 0)
      Neg[1] = 1;
  }

  // Clear data
  if (Pos[0] > upperBound)
  {
    Pos[0]= 0;
    Period[0]= 0; 
  }
  if (Neg[0] > upperBound)
  {
    Neg[0]= 0;
    Period[1]= 0; 
  }

  // Record count information for HIGH
  if (Pos[1] == 1)
  {
//    DPrint('%');
//    DPrintln(Pos[0], DEC);
    Period[0] = Pos[0]; 
    Pos[0] = 0;
    Pos[1] = 0; 
  }

  // Record count information for LOW
  if (Neg[1] == 1)
  {
//    DPrint('%');
//    DPrintln(Neg[0], DEC);
    Period[1] = Neg[0];
    Neg[0] = 0;
    Neg[1] = 0;
  }

  // Check frequency range
  checkFrequencyCount++;
  if (checkFrequencyCount >= 10) // 100 ms
  {
    Period[2] = Period[0] + Period[1];
    
    if ((Period[2] >= lowerBound) && (Period[2] <= upperBound))
    {
      DPrintln("Success: Get direction light"); // %0
      DPrint("Count = ");
      DPrintln(Period[2], DEC);
      digitalWrite(DIRECTION_LIGHT_LED_OUTPIN, HIGH);
    }
    else
    {
      DPrintln("Failed: Get direction light"); // %1
      DPrint("Count = ");
      DPrintln(Period[2], DEC);
      digitalWrite(DIRECTION_LIGHT_LED_OUTPIN, LOW);
    }

    checkFrequencyCount = 0;
  }
}
