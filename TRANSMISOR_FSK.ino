// -----------------------------------------------------------------------------
//  PIN AND TIMING CONFIGURATION (PHYSICAL LAYER)
// -----------------------------------------------------------------------------
#define FSK_OUTPUT_PIN 8  // Pin that will send logic pulses to the modulator circuit (XR2206)

// Exact bit duration times (Equivalent to ~16 baud)
#define BIT_PERIOD_MS  62UL
#define BIT_PERIOD_US 500UL

// Separation times (Pauses to allow the receiver to process the information)
#define INTER_CHAR_DELAY_MS   500UL
#define INTER_WORD_DELAY_MS  2000UL

#define MAX_BUFFER_LENGTH  64
#define SYNC_CHARACTER '~' // Symbol sent to stabilize the receiver's PLL before transmitting data

// -----------------------------------------------------------------------------
//  GLOBAL VARIABLES
// -----------------------------------------------------------------------------
String txBuffer       = "";    // Temporary memory holding the word to be sent
bool   dataReady      = false; // Flag indicating when "Enter" has been pressed
bool   isTransmitting = false; // Flag to prevent overlapping transmissions

// -----------------------------------------------------------------------------
//  TRANSMISSION FUNCTIONS (LOW-LEVEL LOGIC)
// -----------------------------------------------------------------------------

/* Function that controls the wire's voltage for the exact duration of 1 bit.
   Note: FSK hardware often requires inverted voltage logic 
   (Logical 1 = LOW, Logical 0 = HIGH).
*/
void sendBit(bool bitValue) 
{
    // Inverted logic: If bitValue is true(1) sends LOW, if false(0) sends HIGH
    digitalWrite(FSK_OUTPUT_PIN, bitValue ? LOW : HIGH);
    
    // Holds the voltage for exactly 62.5 milliseconds
    delay(BIT_PERIOD_MS);
    delayMicroseconds(BIT_PERIOD_US);
}

/* Function that breaks down a character into 8 bits (zeros and ones) and sends them to the wire.
   The UART protocol requires a Start bit before the character, and a Stop bit at the end.
*/
void transmitCharacter(char character) 
{
    uint8_t dataByte = (uint8_t)character;

    sendBit(0);  // START BIT: Send a 0 (Receiver will detect a falling edge to LOW)

    // Send the 8 data bits (Starting with the Most Significant Bit - MSB)
    for (int i = 7; i >= 0; i--) 
    {
        sendBit((dataByte >> i) & 1); // Extracts the bit at position 'i' and sends it
    }
    
    sendBit(1);  // STOP BIT: Send a 1 (Receiver will detect a rising edge to HIGH)
}

// -----------------------------------------------------------------------------
//  UTILITY AND PC MENU FUNCTIONS
// -----------------------------------------------------------------------------

/* Purely visual function to print how the character looks in binary on the screen */
void printBinaryFormat(char character) 
{
    uint8_t value = (uint8_t)character;
    for (int i = 7; i >= 0; i--) 
    {
        Serial.print((value >> i) & 1);
        if (i == 4) Serial.print(' '); // Adds a space in the middle for easier reading
    }
}

void printMenu() 
{
    Serial.println(F("\n+---------------------------------------------------+"));
    Serial.println(F("|        FSK TRANSMISSION SYSTEM | XR2206           |"));
    Serial.println(F("+---------------------------------------------------+"));
    Serial.println(F("| [i] Configuration: START=0 | STOP=1 | MSB         |"));
    Serial.println(F("| [>] Waiting for data... Enter a word:             |"));
    Serial.println(F("+---------------------------------------------------+"));
}

/* Function that listens to what is typed on the computer (Serial Monitor) 
   and stores it in memory until the "Enter" key is pressed.
*/
void readSerialPort() 
{
    while (Serial.available()) 
    {
        char readChar = (char)Serial.read();
        
        // If it detects an "Enter" (Newline or carriage return), flag data as ready
        if (readChar == '\n' || readChar == '\r') 
        {
            if (txBuffer.length() > 0) dataReady = true;
        } 
        else 
        {
            // If it is a normal letter, append it to the final text
            if (txBuffer.length() < MAX_BUFFER_LENGTH) {
                txBuffer += readChar;
            }
        }
    }
}

// -----------------------------------------------------------------------------
//  MAIN ROUTINES (SETUP & LOOP)
// -----------------------------------------------------------------------------

void setup() 
{
    pinMode(FSK_OUTPUT_PIN, OUTPUT);
    
    // UART protocol dictates that the idle state (not talking) must be HIGH.
    // Since our physical logic is inverted, we output LOW.
    digitalWrite(FSK_OUTPUT_PIN, LOW);   
    
    Serial.begin(9600);
    printMenu();
}

void loop() 
{
    readSerialPort();

    // If LabVIEW sent an 'enter' and we are not busy transmitting something else...
    if (dataReady && !isTransmitting) 
    {
        isTransmitting = true;
        dataReady      = false;

        Serial.print(F("\n[!] Starting transmission of: \""));
        Serial.print(txBuffer);
        Serial.println(F("\""));

        // 1. Long silence for the receiver to reset and pay attention
        digitalWrite(FSK_OUTPUT_PIN, LOW);
        delay(INTER_WORD_DELAY_MS);

        // 2. Synchronization preamble (Locks the receiver circuit's PLL)
        Serial.println(F("[*] Emitting synchronization pulses (SYNC x2)..."));
        transmitCharacter(SYNC_CHARACTER); // Sends the '~' symbol
        digitalWrite(FSK_OUTPUT_PIN, LOW);
        delay(INTER_CHAR_DELAY_MS);
        transmitCharacter(SYNC_CHARACTER); // Sends the '~' symbol again
        digitalWrite(FSK_OUTPUT_PIN, LOW);
        delay(INTER_CHAR_DELAY_MS);

        // 3. Payload transmission (The actual word you typed)
        Serial.println(F("[*] Transmitting data frame:"));
        for (int i = 0; i < (int)txBuffer.length(); i++) 
        {
            char currentLetter = txBuffer.charAt(i);
            
            // Displays on the PC screen what is about to be sent over the wire
            Serial.print(F("    -> TX ['"));
            Serial.print(currentLetter);
            Serial.print(F("'] | ASCII: "));
            Serial.print((uint8_t)currentLetter);
            Serial.print(F(" | Binary: "));
            printBinaryFormat(currentLetter);
            Serial.println();

            // Physically sends the letter through pin 8
            transmitCharacter(currentLetter);
            
            // Returns to idle voltage
            digitalWrite(FSK_OUTPUT_PIN, LOW);
            
            // Pauses before sending the next letter
            if (i < (int)txBuffer.length() - 1) delay(INTER_CHAR_DELAY_MS);
        }

        // 4. End of transmission
        digitalWrite(FSK_OUTPUT_PIN, LOW);
        delay(INTER_WORD_DELAY_MS);

        Serial.println(F("[V] Transmission successfully completed."));
        
        // Clear memory to wait for a new word
        txBuffer = "";
        isTransmitting = false;
        printMenu();
    }
}