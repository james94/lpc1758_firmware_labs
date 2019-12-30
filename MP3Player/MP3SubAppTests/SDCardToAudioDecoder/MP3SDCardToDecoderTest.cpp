#include <MP3Player/MP3SubAppTests/SDCardToAudioDecoder/MP3SDCardToDecoderTest.h>

extern SemaphoreHandle_t spi1_bus_lock;
QueueHandle_t audio_queue;
QueueHandle_t volume_queue;
TaskHandle_t xDecoderHandle;
state decoder_state = playSong;
//SemaphoreHandle_t xSemaDREQRise;
SemaphoreHandle_t PlayPauseSem  = xSemaphoreCreateBinary();
SemaphoreHandle_t NextSem       = xSemaphoreCreateBinary();
SemaphoreHandle_t PrevSem       = xSemaphoreCreateBinary();
SemaphoreHandle_t MenuSem       = xSemaphoreCreateBinary();

track_metadata track001 = {"Hallelujah", "Logic", "Everybody", "Hip-Hop/Rap"};
track_metadata track002 = {"Everybody", "Logic", "Everybody", "Hip-Hop/Rap"};
track_metadata track003 = {"Confess", "Logic", "Everybody", "Hip-Hop/Rap"};
track_metadata track004 = {"Take It Back", "Logic", "Everybody", "Hip-Hop/Rap"};
track_metadata track005 = {"1-800-273-8255", "Logic", "Everybody", "Hip-Hop/Rap"};
track_metadata track006 = {"Black-SpiderMan", "Logic", "Everybody", "Hip-Hop/Rap"};



/**
 * MP3 SDCard Start
 */

ID3v1_t mp3;
//Data buffer of 512 bytes. VS1053b can take 32 bytes at a go.
uint8_t mp3_file_data[512] = {0};


void printMP3Data(uint8_t *mp3_data, uint16_t size)
{
    for(uint16_t j = 0; j < size; j++)
    {
        u0_dbg_printf("%c", *(mp3_data + j));
    }
}

//Utilizes "storage" object
void readMP3FileMetaData()
{
    uint32_t offset = 0;
    Storage::read("1:track001.mp3", mp3.buffer, sizeof(mp3.buffer), offset);

    if(offset == 0)
    {
        printMP3Data( mp3.title, (sizeof(mp3.title)/sizeof(mp3.title[0])) );
    }
}

void printTest()
{
    u0_dbg_printf("Hello from SD Card\n");
}

void sendMP3DataToDecoder(uint8_t *mp3_data)
{

    if(xQueueSend(audio_queue, mp3_data, 10) != pdTRUE)
    {
        //u0_dbg_printf("Failed to send mp3 file data to MP3 Decoder\n");
    }
    else
    {
        //u0_dbg_printf("SD Card Task sent audio data\n");
    }
}

void readMP3FileData(const char *filename, uint32_t *offset)
{
    FRESULT status;
    //Read 512 Byte File Data Chunks
    status = Storage::read(filename, mp3_file_data, (sizeof(mp3_file_data)/sizeof(mp3_file_data[0])), *offset);

    //Read the first 128 Bytes of File if offset == 0
//    if(*offset == 0)
//    {
//        //status = Storage::read(filename, mp3.buffer, sizeof(mp3.buffer), *offset);
//        //extractSongInfo(mp3.buffer);
//    }
    if(status != FR_OK)
    {
        u0_dbg_printf("Failed to read %s\n", filename);
    }
    else
    {
        //printMP3Data( data, (sizeof(data)/sizeof(data[0])) );

        sendMP3DataToDecoder(mp3_file_data);
        *offset = *offset + (sizeof(mp3_file_data)/sizeof(mp3_file_data[0]));
    }

}

void testReadSDCard(void *vParam)
{
    u0_dbg_printf("Entered testReadSDCard\n");
    const char *filename = "1:track005.mp3";
    uint32_t offset = 0;
    while(1)
    {
        //readMP3FileMetaData();
        if(spi1_bus_lock != NULL) {
            if(xSemaphoreTake(spi1_bus_lock, 100)) {
                readMP3FileData(filename, &offset);
                xSemaphoreGive(spi1_bus_lock);
            }
        }
        vTaskDelay(2);
    }
}





/**
 * MP3 SDCard End
 * MP3 Decoder Start
 */

parametric get_data;

//Write to VS1053b Register
//SCI: Data transfers are always 16 bit. When a new SCI operation comes in,
//DREQ goes low. We must wait for DREQ Rising Edge Interrupt again.
//xcs should be low for the duration of the operation.
//Algorithm based on SCI Write Description "7.4.2" of VS1053b Datasheet
void writeSciReg(GPIO *sci_cs, GPIO *dreq, uint8_t sci_reg_addr, uint16_t sci_cmd)
{
    //7.2: Check if DREQ Low, then wait until VS1053b is ready to receive SCI Write, break loop
    //u0_dbg_printf("Writing to SCI Reg as long as DREQ is high\n");
    //wait for DREQ Rising Edge Interrupt indicating VS1053b is available

    while(!dreq->read()); //wait for DREQ to go high indicating IC is available

    sci_cs->setLow(); //XCS line pulled low to select SCI mode


    // SCI write consists of WRITE instruction byte, reg address byte, one 16-bit data word
    ssp0_exchange_byte(0x02);//WRITE opcode transmitted via SI line 8-bits
    ssp0_exchange_byte(sci_reg_addr);//8-bit word address of register
    //sci_cmd bytes sent MSb first [15....8, 7....0]
    ssp0_exchange_byte((sci_cmd >> 8) & 0xFF);//Transmit MSB of the sci_cmd
    ssp0_exchange_byte(sci_cmd & 0xFF);//Transmit LSB of the sci_cmd

    while(!dreq->read()); //wait for DREQ to go high indicating command is done
    sci_cs->setHigh(); //XCS pulled high to end WRITE sequence
}

//Read the 16-bit value of a VS1053b register
// Algorithm based on SCI Read Description "7.4.1" of VS1053b Datasheet
uint16_t readSciReg(GPIO *sci_cs, GPIO *dreq, uint8_t sci_reg_addr)
{
    uint16_t result = 0;

    while(!dreq->read()); //wait DREQ to go high indicating IC is available
    //u0_dbg_printf("\nAble to take DREQ_Semaphore RE, IC ready for SCI Read\n");
    //SCI consists of READ instruction byte, address byte, 16-bit data word
    sci_cs->setLow(); //XCS line pulled low to select SCI mode


    ssp0_exchange_byte(0x03); //READ instruction transmitted via SI line 8-bits
    ssp0_exchange_byte(sci_reg_addr); //8-bit word address of register
    //Data bits are read at rising edge, so user should update data at falling edge
    //Bytes are received on SO line MSB first
    //vTaskDelay(10);//10ms delay

    result = ssp0_exchange_byte(0xFF);//read first byte (MSB)

    while(!dreq->read()); //wait for DREQ to go high indicating command is complete

    //u0_dbg_printf("\nAble to take DREQ_Semaphore RE, SCI Read 1st Byte Done\n");
    result = result << 8; //shift received data 8 spaces to left into MSB
    result |= ssp0_exchange_byte(0xFF);//read second byte (LSB)

    while(!dreq->read()); //wait for DREQ to go high indicating command is complete
        //u0_dbg_printf("\nAble to take DREQ_Semaphore RE, SCI Read 2nd Byte Done\n");

    sci_cs->setHigh(); //XCS line pulled high since data has been shifted out

    return result;
}

//SDI send data bytes consists of sending bytes from the buffer to the decoder
//Algorithm based on sending data bytes over SDI with MSb first from "7.3.1" of VS1053b datasheet
void sendSDIData(GPIO *sdi_cs, GPIO *dreq, uint8_t *buffer, uint32_t size)
{
    static uint32_t n_elements = 0;
    if(size == 32)
    {
        n_elements = size;
    }
    else if(size > 32)
    {
        n_elements = 32;
    }
    else if(size < 32)
    {
        n_elements = size;
    }

    //u0_dbg_printf("Sending Decoder Data over SDI as long as DREQ is high\n");

    while(size)
    {
        taskENTER_CRITICAL(); //Allow transfer to occur without interruption

        while(!dreq->read()); //Wait for DREQ to go high indicates IC is available
        sdi_cs->setLow(); //XDCS line pull low to select SDI mode

        //Feed 32 bytes of data to the VS1053b from SD Card
        for(uint32_t i = 0; i < n_elements; i++)
        {
            ssp0_exchange_byte(*buffer);
            buffer++;
            size--;
        }

        while(!dreq->read()); //Wait for DREQ to go high indicating command is complete
        sdi_cs->setHigh(); //XDCS line pull high cause done sending data

        taskEXIT_CRITICAL();//Just dumped 32 bytes into VS1053b, loop to dump 32 bytes again

//        while(!dreq->read()); //wait for DREQ to go high indicating IC is available
//        sdi_cs->setLow(); //XDCS line pull low to select SDI mode
//
//        //send 32 bytes at a time until total size of buffer is complete
//        for(uint32_t i = 0; i < n_elements; i++)
//        {
//            ssp0_exchange_byte(*buffer);
//            buffer++;
////            is_32_bytes_sent++;
//            size--;
//        }
//
//        while(!dreq->read()); //wait for DREQ to go high indicating SDI data done processing
//        sdi_cs->setHigh(); //XDCS line pull high cause done sending data
//
//        if(is_32_bytes_sent == 32)
//        {
//            while(!dreq->read()); //wait for DREQ to go high indicating SDI data done processing
//        sdi_cs->setHigh(); //XDCS line pull high cause done sending data
//            is_32_bytes_sent = 0; //clear back to zero
//        }

    }

//    while(!dreq->read()); //wait for DREQ to go high indicating SDI data done processing
//    sdi_cs->setHigh(); //Deselect Data
}

/**
 * //Sine test is initialized with 8-byte sequence 0x53 0xEF 0x6E "n" 0000
 *      //"n" defines sine test to use
 *
 *      //The Freq of sine output can be calculated from F = ( Fs x (S/128) )
 *
 *      //Example:
 *          //Sine Test is activated with "n" value 126, 0b0111_1110
 *              //Breaking "n" to its components: FsIdx = 0b011 = 3; Fs = 22050Hz;
 *                  //S = 0b11110 = 30; Final Sine Freq F = ( 22050Hz × (30/128) ) = 5168Hz
 *          //To exit the sine test, send the sequence 0x45 0x78 0x69 0x74 0000
 * //Sine test is exited with 8-byte sequence 0x45 0x78 0x69 "n" 0000
 */
void sineTest(GPIO *xreset, GPIO *sci_cs, GPIO *sdi_cs, GPIO *dreq)
{

//    u0_dbg_printf("\nreset MP3 decoder\n");

    //2. SM_TESTS is set in SCI_MODE reg
    //allow SDI tests: 0b0100_1000_0010_0000
    writeSciReg(sci_cs, dreq, SCI_MODE, SM_SDINEW | SM_TESTS);

//    u0_dbg_printf("\nset SM_TESTS in SCI_MODE reg\n");

    //Next A test command is sent to SDI bus
    //Each test started by sending 4-byte special command sequence followed by 4 zeros

    //3. Send Test Command [0x53, 0xEF, 0x6E, 0x7E, 0x00, 0x00, 0x00, 0x00] to start Sine Test
    uint8_t start_sine_test[8] = {0x53, 0xEF, 0x6E, 0x7E, 0x00, 0x00, 0x00, 0x00};
    uint32_t num_elements = sizeof(start_sine_test)/sizeof(start_sine_test[0]);
    sendSDIData(sdi_cs, dreq, start_sine_test, num_elements);

//    u0_dbg_printf("\nStarted sine test\n");

    //4. Send 0x45, 0x78, 0x69, 0x74, 0x00, 0x00, 0x00, 0x00 to exit sine test
    uint8_t end_sine_test[8] = {0x45, 0x78, 0x69, 0x74, 0x00, 0x00, 0x00, 0x00};
    num_elements = sizeof(end_sine_test)/sizeof(end_sine_test[0]);
    sendSDIData(sdi_cs, dreq, end_sine_test, num_elements);

    //writeSciReg(sci_cs, dreq, SCI_MODE, readSciReg(sci_cs, dreq, SCI_MODE) & ~SM_TESTS);

//    u0_dbg_printf("\nEnded sine test\n");
    //Note: sine test signals go through digital volume control, so it's possible to test channels separately
}

void vTestMP3Decoder(void *pvParameters)
{
    //Initialization
//    SSPDriver mp3Decoder;
    GPIO xdcs(P0_29); //Data Chip Select/BSYNC Pin
    GPIO xcs(P0_30); //Control Chip Select Pin (for accessing SPI Control/Status registers)
    GPIO dreq(P0_1); //Data Request Pin: Player asks for more data
    GPIO xreset(P1_22); //Reset is active low

    initDecoder(&xdcs, &xcs, &xreset, &dreq);

    while(1)
    {
        sineTest(&xreset, &xcs, &xdcs, &dreq);

        vTaskDelay(2);
    }

}

/**
 * Hardware Reset Decoder
 * 1. Pull xreset pin low
 * 2. Initialize Registers:
 *      A. SCI_MODE (set SM_SDINEW mode)
 *      B. SCI_BASS (set dB enhancement to 16dB below 60Hz, set treble enhancement to 10.5dB above 10Khz
 *      C. SCI_CLOCKF (set SC_FREQ to 0, so indicates XTALI is default at 12.288MHz)
 *      D. SCI_VOL (set left and right volumes at -18.0 dB)
 * 3. Pull xreset pin high
 */
void hardReset(GPIO *xreset, GPIO *sci_cs, GPIO *sdi_cs, GPIO *dreq)
{
//    When the XRESET -signal is driven low, VS1053b is reset and all the control registers and
//    internal states are set to the initial values. XRESET-signal is asynchronous to any external
//    clock. The reset mode doubles as a full-powerdown mode, where both digital and analog parts
//    of VS1053b are in minimum power consumption stage, and where clocks are stopped. Also
//    XTALO is grounded.
    vTaskDelay(10); //Keep Clear of anything prior
    xreset->setLow();
//    u0_dbg_printf("\nsetting xreset pin LOW to reset decoder\n");
    vTaskDelay(10);
    //Set reset pin high, so next time around, we can set pin low to reset device
    xreset->setHigh();

//    When XRESET is asserted, all output pins go to their default states. All input pins will go to
//    high-impedance state (to input state), except SO, which is still controlled by the XCS.
//    After a hardware reset (or at power-up) DREQ will stay down for around 22000 clock cycles,
//    which means an approximate 1.8 ms delay if VS1053b is run at 12.288 MHz. After this the
//    user should set such basic software registers as SCI_MODE, SCI_BASS, SCI_CLOCKF, and
//    SCI_VOL before starting decoding. See section 9.6 for details.


//    u0_dbg_printf("\nsetting SCI_MODE reg to default value: SM_SDINEW mode\n");



//    u0_dbg_printf("\nsetting SCI_BASS reg to 15dB enhancement below 60Hz\n");
//    u0_dbg_printf("\nsetting SCI_BASS reg to 10.5dB treble enhancement above 10kHz\n");

    //The VS1053b should be up and running, increase the internal clock multiplier and up SPI rate
    //SC_MULT [15:13] = 000 = Clock Multiplier
    //SC_ADD [12:11] = 00 = Allowed Multiplier Addition
    //SC_FREQ [10:0] = 00000000000 = Clock Frequency
    //SC_MULT multiplies XTALI to create a higher CLKI
    //
}

void softReset(GPIO *sci_cs, GPIO *dreq)
{
//    In some cases the decoder software has to be reset. This is done by activating bit SM_RESET
//    in register SCI_MODE (Chapter 9.6.1). Then wait for at least 2 us, then look at DREQ.
    writeSciReg(sci_cs, dreq, SCI_MODE, DEFAULT_MODE_SM_SDINEW | SM_RESET);

//    u0_dbg_printf("\nset 'sm_reset' bit inside 'sci_mode' register\n");
    writeSciReg(sci_cs, dreq, SCI_DECODE_TIME, 0x00);
    writeSciReg(sci_cs, dreq, SCI_DECODE_TIME, 0x00);

    vTaskDelay(10); //wait for 100 ms
}

/**
 * Sets Volume Register (SCI_VOL)
 * Controls volume for the player hardware
 * Most Significant Byte of volume register controls the left channel volume,
 * the low part controls the right channel volume. The channel volume sets the attenuation
 * from the maximum volume level in 0.5dB steps. Thus, max volume is 0x0000
 * and total silence is 0xFEFE
 */
void setVolume(GPIO *sci_cs, GPIO *dreq, uint8_t left_channel, uint8_t right_channel)
{
    uint16_t vol_level = 0;
    vol_level = left_channel;
    vol_level <<= 8;
    vol_level |= right_channel;
    writeSciReg(sci_cs, dreq, SCI_VOL, vol_level);
}

/**
 * Initialize SJ-One Board pins connected to Decoder
 */
bool initPins(GPIO *xdcs, GPIO *xcs, GPIO *xreset, GPIO *dreq)
{
    bool init_pins = false;
    //7.1.1 VS10xx Native Modes (New Mode)
    xdcs->setAsOutput(); //XDCS: SDI P0.29
    xcs->setAsOutput(); //XCS: SCI P0.30
    xreset->setAsOutput(); //XRESET: P1.22
    dreq->setAsInput();
    dreq->enablePullDown();

    xcs->setHigh(); //Deselect Control
    xdcs->setHigh(); //Deselect Data

    //Datasheet pg 12, max SCI reads are CLKI/7. Input clock is 12.288MHz
    //Internal clock multiplier is 1.0x after power up
    //Thus, max SPI speed is 1.75MHz. We'll use 1MHz to be safe
    ssp0_init(1);//set clock to 1MHz
    ssp0_set_max_clock(1);

    init_pins = true;
    return init_pins;
}

/**
 * Initialize Decoder:
 * - Software Reset
 * - Set Internal Registers, such as Clock for normal operation
 */
bool initDecoder(GPIO *xdcs, GPIO *xcs, GPIO *xreset, GPIO *dreq)
{
    //Initialize VS1053b chip
    bool init_decoder = false;

    initPins(xdcs, xcs, xreset, dreq);

    //Hardware Reset to shutdown and bring back up VS1053b
    hardReset(xreset, xcs, xdcs, dreq);

    softReset(xcs, dreq);

    //SCI_MODE controls the operation of VS1053b: 0x4882 (SM_SDINEW set), but allowed MPEG layers I,II,III
    //selects EarSpeaker normal mode
    writeSciReg(xcs, dreq, SCI_MODE, DEFAULT_MODE_SM_SDINEW | SM_LAYER | SM_EARSPEAKER_LO);

    //SCI_BASS
        //    VSBE is activated when SB_AMPLITUDE is non-zero. SB_AMPLITUDE should be set to the
        //    user’s preferences, and SB_FREQLIMIT to roughly 1.5 times the lowest frequency the user’s
        //    audio system can reproduce. For example setting SCI_BASS to 0x00f6 will have 15 dB enhancement
        //    below 60 Hz.

        //    Treble Control VSTC is activated when ST_AMPLITUDE is non-zero. For example setting
        //    SCI_BASS to 0x7a00 will have 10.5 dB treble enhancement at and above 10 kHz.
    uint16_t sci_cmd = 0x7af6;
    writeSciReg(xcs, dreq, SCI_BASS, sci_cmd);

    //From 7.6 of Datasheet, max SCI Reads are CLKI/7
    //Assume CLKI = 12.288MHz for VS1053b
    //SJ-One Board CCLK is 96MHz
    //VS1053's internal clock multiplier SCI_CLOCKF:SC_MULT is 1.0x after power up
    //For a max SPI rate of 1.8MHz = (CLKI/7) = (12.288/7) VS1053's default

    //Decoder XTALI (Internal Clock Frequency) is 12.288 (4.2 Recommended Op Conditions)
    //With CLOCKF set to 0x8800
    //Decoder CLKI (Internal Clock Multiplier) is 43.008 MHz
    //Decoder Master Clock Duty Cycle set between 50 and 60%
    //Up the Max SCI Reads to CLKI/7 = (12.288/7)*(3.5 + 1) = (12.288/7)*(4.5) = 7.899MHz
    //Max SPI speed is 55.3MHz
    //7.889MHz means that my board supplying SCK0 cannot go over this clock speed
    //0x8800 = XTALI = 12.288MHz, CLKI = 55.3MHz, SCI Reads = 7.899MHz
    //0x6000 = XTALI = 12.288MHz, CLKI = 36.864MHz, SCI Reads = 5.266MHz
    writeSciReg(xcs, dreq, SCI_CLOCKF, 0x8800); //Set multiplier to 3.0x

//    u0_dbg_printf("\nsetting SCI_CLOCKF to default: SC_MULT=4, SC_ADD=3, SC_FREQ=0\n");
//    u0_dbg_printf("\nSo, SCI_CLOCKF indicates XTALI is 12.288MHz\n");

    //0x0000 max volume
    //0xFEFE total silence
    //Levels with even balance left/right ears
    //Loud Level: 0x0404
    //Mid Level: 0x2424
    //Quiet Level: 0x5454
    //Off Level: 0xFEFE
    writeSciReg(xcs, dreq, SCI_VOL, 0x5454);

    uint8_t dummy_bytes[] = {0x00, 0x00};
    sendSDIData(xdcs, dreq, dummy_bytes, sizeof(dummy_bytes));
    //setVolume(xcs, 20, 20); //Set initial volume LOUD (20 = -10dB)
    //setVolume(xcs, dreq, 40, 40); // Set initial volume Mid Level Noise (20 = -10dB)
    //setVolume(xcs, 80, 80); //Set initial volume quiet (20 = -10dB)

    //Check status of VS1053b
//    uint16_t mp3_mode = readSciReg(xcs, dreq, SCI_MODE);
//    uint16_t mp3_status = readSciReg(xcs, dreq, SCI_STATUS);
//    uint16_t mp3_clock = readSciReg(xcs, dreq, SCI_CLOCKF);
//    uint16_t mp3_volume = readSciReg(xcs, dreq, SCI_VOL);

//    u0_dbg_printf("SCI_MODE (0x4800) = 0x%x\n", mp3_mode);
//    u0_dbg_printf("SCI_STATUS (0x48) = 0x%x\n", mp3_status);

    //Mask out the four version 4 bits
//    uint16_t vs_version = (mp3_status >> 4) & 0x000F;

    //The VS1053b should respond with 4. VS1001 = 0, VS1011 = 1, VS1002 = 2, VS1003 = 3
//    u0_dbg_printf("VS Version (VS1053 is 4) = %i\n", vs_version);
//
//    u0_dbg_printf("SCI_CLOCKF = 0x%x\n", mp3_clock);
//    u0_dbg_printf("SCI_VOL = 0x%x\n", mp3_volume);

    //Page 12 of datasheet, max SCI reads are CLKI/7. Input clock is 12.288MHz
    //Internal Clock Multiplier is 3.5x
    //Hence, max SPI speed is 6.144MHz
    //Hence, max SPI speed is 5MHz. 4MHz to be safe

    //Set SJ-One Board SPI bus speed to
    //if 4, then (96/20) = 4.8MHz
    //if 5, then (96MHz/18) = 5.333MHz
    //if 6, then (96MHz/14) = 6.857MHz
    ssp0_set_max_clock(5);

    vTaskDelay(10); // Settle Time

    //MP3 IC initialization complete
    init_decoder = true;
    return init_decoder;
}

//char *get_pacific_time()
//{
//    char tmp[80] = {0x0};
//    setenv("TZ", "PST8PST", 1); //set TZ
//    tzset(); //recognize TZ
//    time_t lt = time(NULL); // epoch seconds
//    struct tm *p = localtime(&lt); // get local time struct tm
//    strftime(tmp, 80, "%c", p); // format time use format string, %c
//    return tmp;
//}

bool receiveMP3DataFromSDCard(uint8_t *audio_data, uint16_t size)
{
    bool received = false;
//    uint8_t mp3_data = 0;
    if(xQueueReceive(audio_queue, audio_data, 100) != pdTRUE)
    {
        //u0_dbg_printf("\nFailed to rx audio frame from SD Card\n");
        received = false;
    }
    else
    {
        //u0_dbg_printf("Received audio frame from SD Card\n");

//        for(uint16_t i = 0; i < size; i++)
//        {
//                mp3_data = *(audio_data + i);
//                u0_dbg_printf("\n0x%x\n", mp3_data);
//        }

        received = true;
    }
    return received;
}

/**
 * Pulls 32 byte chunks from the SD card and throws them at the VS1053b
 * We check if DREQ Rising Edge Interrupt Triggered (data request pin)
 * If DREQ Rising Edge Interrupt didn't trigger, we determine if we need
 * new data or not. If yes, pull new data from SD card. Then throw data at
 * VS1053b until it is full.
 */
void playWholeFile(GPIO *sci_cs, GPIO *sdi_cs, GPIO *dreq, uint8_t *audio_filedata, uint16_t length)
{
        //Default Playback Mode
        uint8_t end_fill_byte = 0;
//        uint16_t bitrate_of_file = 0;
        bool is_sm_cancel_set = false;
        uint16_t sci_mode_data = 0, sci_hdat0_data = 0, sci_hdat1_data = 0;
        uint16_t bytes_sent = 1;
        bool is_hdat0_cleared = false, is_hdat1_cleared = false;
        uint16_t adjust_volume = 0x2424;

        do{
            //1. Send an audio file to VS1053b
            while(receiveMP3DataFromSDCard(audio_filedata, length))
            {
                if(decoder_state == volumeUp || decoder_state == volumeDown)
                {
                    //Pointer receives the address of adjust_volume variable from MP3Controller
                    if(xQueueReceive(volume_queue, &adjust_volume, 5) != pdTRUE)
                    {
                        u0_dbg_printf("\nFailed to receive desired volume\n");
                    }
                    else
                    {
                        if((adjust_volume == 0xFEFE) | (adjust_volume == 0xE4E4) | (adjust_volume == 0xC4C4) |
                           (adjust_volume == 0xA4A4) | (adjust_volume == 0x8484) | (adjust_volume == 0x6464) |
                           (adjust_volume == 0x4444) | (adjust_volume == 0x2424) | (adjust_volume == 0x0404) )
                        {
                            writeSciReg(sci_cs, dreq, SCI_VOL, adjust_volume);
                            u0_dbg_printf("0x%x\n", adjust_volume);
                        }
                        else {
                            adjust_volume = 0x2424;
                            writeSciReg(sci_cs, dreq, SCI_VOL, adjust_volume);

                        }
//                        u0_dbg_printf("\nReceived desired volume\n");
                        //Value received gets copied to address that pointer points to
                        //adjust_volume[desired_volume];

                    }
                }
                //Feed 512 bytes of data chunks to VS1053b in 32 byte chunks until song sent
                sendSDIData(sdi_cs, dreq, audio_filedata, length);
            }

    //        uint16_t mp3_clock = readSciReg(sci_cs, dreq, SCI_CLOCKF);
    //        u0_dbg_printf("\nPlayWholeFile SCI_CLOCKF = 0x%x\n", mp3_clock);
    //
    //        uint16_t mp3_volume = readSciReg(sci_cs, dreq, SCI_VOL);
    //        u0_dbg_printf("\nPlayWhole File SCI_VOL = 0x%x\n", mp3_volume);
    //
    //        sci_hdat0_data = readSciReg(sci_cs, dreq, SCI_HDAT0);
    //        u0_dbg_printf("\nHDAT0 Reg Data = %x\n", sci_hdat0_data);
    //        sci_hdat1_data = readSciReg(sci_cs, dreq, SCI_HDAT1);
    //        u0_dbg_printf("\nHDAT1 Reg Data = %x\n", sci_hdat1_data);
    //
    //        //Get Bitrate of file
    //        writeSciReg(sci_cs, dreq, SCI_WRAMADDR, BYTERATE_ADDR); //Send address of byteRate Reg
    //        get_data.byteRate = readSciReg(sci_cs, dreq, SCI_WRAM); //Read byteRate
    //        u0_dbg_printf("\nAverage Data Rate Bytes Per Second = %i\n", get_data.byteRate);
    //        bitrate_of_file = get_data.byteRate * 8;
    //        u0_dbg_printf("\nBitrate of Audio File = %i\n", bitrate_of_file);


            //2. Read extra parameter value endFillByte
            writeSciReg(sci_cs, dreq, SCI_WRAMADDR, ENDFILLBYTE_ADDR); //Send address of EndFillByte Reg
            get_data.endFillByte = readSciReg(sci_cs, dreq, SCI_WRAM); //Read EndFillByte Reg
            end_fill_byte = get_data.endFillByte & 0xFF;

//            u0_dbg_printf("\nendFillByte = %i\n", get_data.endFillByte);
//            u0_dbg_printf("\nLSB end_fill_byte = %i\n", end_fill_byte);


            //3. Send at least 2052 bytes of endFillByte[7:0]
            for(uint16_t i = 0; i < 2052; i++)
            {
                sendSDIData(sdi_cs, dreq, &end_fill_byte, sizeof(end_fill_byte));
            }

            //4. Set SCI_MODE bit SM_CANCEL
            writeSciReg(sci_cs, dreq, SCI_MODE, DEFAULT_MODE_SM_SDINEW | SM_CANCEL);

            do{
                //5. Send at least 32 bytes of endFillByte[7:0]
                for(uint16_t i = 0; i < 32; i++)
                {
                    sendSDIData(sdi_cs, dreq, &end_fill_byte, sizeof(end_fill_byte));
                    bytes_sent++;
                }
                //6. Read SCI_MODE. //If SM_CANCEL is still set, go to 5.
                sci_mode_data = readSciReg(sci_cs, dreq, SCI_MODE);
//                u0_dbg_printf("\nIs SM_CANCEL bit set in SCI_MODE? %i\n", sci_mode_data & SM_CANCEL);
                is_sm_cancel_set = sci_mode_data & SM_CANCEL;

                //If SM_CANCEL hasn't cleared after sending 2048 bytes
                if(bytes_sent > 2047)
                {
                    softReset(sci_cs, dreq); //do a software reset(rare)
                }
            }while(is_sm_cancel_set);

            //7. The song has now been successfully sent.
            //HDAT0 and HDAT1 should now both contain 0
            //to indicate that no format is being decoded
            sci_hdat0_data = readSciReg(sci_cs, dreq, SCI_HDAT0);
            is_hdat0_cleared = sci_hdat0_data == 0; //if HDAT0 reg contains zero, return true

//            u0_dbg_printf("\nHDAT0_reg = %i\n", sci_hdat0_data);

            sci_hdat1_data = readSciReg(sci_cs, dreq, SCI_HDAT1);
            is_hdat1_cleared = sci_hdat1_data == 0; //if HDAT1 reg contains zero, return true
//            u0_dbg_printf("\nHDAT1_reg = %i\n", sci_hdat1_data);
        }while(is_hdat0_cleared && is_hdat1_cleared); //loop if HDAT0 and HDAT1 contain zero

}

void vPlayMP3Music(void *pvParameters)
{
    //Initialization
    GPIO xdcs(P0_29); //Data Chip Select/BSYNC Pin
    GPIO xcs(P0_30); //Control Chip Select Pin (for accessing SPI Control/Status registers)
    GPIO dreq(P1_20); //Data Request Pin: Player asks for more data
    GPIO xreset(P1_19); //Reset is active low

    //Loud Level: 0x0404
    //Mid Level: 0x2424
    //Quiet Level: 0x5454
    //Off Level: 0xFEFE

    uint8_t audio_frame[512] = {0};
    uint16_t length = sizeof(audio_frame)/sizeof(audio_frame[0]);

    initDecoder(&xdcs, &xcs, &xreset, &dreq);


    while(1)
    {
        playWholeFile(&xcs, &xdcs, &dreq, audio_frame, length);

        vTaskDelay(2);
    }
}

void vPlayMP3Sample(void *pvParameters)
{
    //Initialization
//    SSPDriver mp3Decoder;
    GPIO xdcs(P0_29); //Data Chip Select/BSYNC Pin
    GPIO xcs(P0_30); //Control Chip Select Pin (for accessing SPI Control/Status registers)
    GPIO dreq(P0_1); //Data Request Pin: Player asks for more data
    GPIO xreset(P1_22); //Reset is active low

    unsigned char HelloMP3[] = { 0xFF, 0xF2, 0x40, 0xC0, 0x19, 0xB7, 0x00, 0x14, 0x02, 0xE6, 0x5C, /* ..@.......\ */
    0x01, 0x92, 0x68, 0x01, 0xF1, 0x5E, 0x03, 0x08, 0xF0, 0x24, 0x80, /* ..h..^...$. */
    0x05, 0x9E, 0x20, 0xC6, 0xFC, 0x12, 0x32, 0x5C, 0xBF, 0xF9, 0xB9, /* .. ...2\... */
    0x20, 0x4A, 0x7F, 0x85, 0xEC, 0x4C, 0xCD, 0xC7, 0x27, 0xFE, 0x5C, /*  J...L..'.\ */
    0x34, 0x25, 0xCB, 0xE6, 0xFF, 0xFF, 0x8E, 0x42, 0xE1, 0xA0, 0x5E, /* 4%.....B..^ */
    0xCA, 0x6E, 0x30, 0x9F, 0xFF, 0xF8, 0xC2, 0x12, 0x84, 0xB9, 0x7C, /* .n0.......| */
    0xDC, 0x61, 0x09, 0x4A, 0x7F, 0xFF, 0xFF, 0xF9, 0x7D, 0x32, 0x51, /* .a.J....}2Q */
    0x09, 0x7C, 0xE1, 0xA5, 0x6E, 0xB4, 0xFF, 0xFF, 0xFF, 0xFF, 0xD3, /* .|..n...... */
    0x34, 0x41, 0x91, 0xF0, 0x11, 0x8F, 0x00, 0x0F, 0x81, 0x9C, 0x10, /* 4A......... */
    0xEE, 0x59, 0xCE, 0x56, 0x67, 0xFF, 0xF2, 0x42, 0xC0, 0xEC, 0x53, /* .Y.Vg..B..S */
    0x09, 0x15, 0xF9, 0xAA, 0xA8, 0x0D, 0xD9, 0x40, 0x00, 0xCA, 0x34, /* .......@..4 */
    0x53, 0xD9, 0x18, 0xAB, 0x7D, 0xF7, 0x89, 0x3F, 0x11, 0x38, 0x94, /* S...}..?.8. */
    0x82, 0x59, 0x93, 0x20, 0x6A, 0x0C, 0xEE, 0x8E, 0x58, 0xFA, 0x38, /* .Y. j...X.8 */
    0x82, 0xCA, 0xF0, 0x58, 0xBB, 0xDA, 0x0C, 0x50, 0x56, 0x1F, 0xBB, /* ...X...PV.. */
    0x18, 0x5D, 0x8B, 0x9F, 0xDA, 0x71, 0x4F, 0xFF, 0xBD, 0xFE, 0xEF, /* .]...qO.... */
    0x69, 0x36, 0x86, 0x3C, 0x50, 0xBB, 0x0A, 0x07, 0x89, 0x54, 0xF0, /* i6.<P....T. */
    0x88, 0x9F, 0x90, 0x95, 0x30, 0x94, 0x2E, 0x7E, 0xF0, 0x64, 0x96, /* ....0..~.d. */
    0x79, 0x08, 0x3E, 0x20, 0x97, 0x28, 0x34, 0x9C, 0x09, 0x7F, 0xD2, /* y.> .(4.... */
    0xC0, 0x01, 0x75, 0xF8, 0x05, 0x6B, 0x5F, 0x41, 0x17, 0x0B, 0xE7, /* ..u..k_A... */
    0xFF, 0xF2, 0x40, 0xC0, 0x61, 0xE5, 0x0B, 0x16, 0x09, 0xC6, 0xC5, /* ..@.a...... */
    0x74, 0x7B, 0xCC, 0x94, 0x7A, 0xF7, 0x80, 0x76, 0xB2, 0xD2, 0xF8, /* t{..z..v... */
    0x39, 0x06, 0x38, 0xFD, 0x71, 0xC5, 0xDE, 0x3A, 0x38, 0xBF, 0xD5, /* 9.8.q..:8.. */
    0xF7, 0x12, 0x37, 0xCB, 0xF5, 0x63, 0x0C, 0x9B, 0xCE, 0x77, 0x25, /* ..7..c...w% */
    0xED, 0xFB, 0x3D, 0x6B, 0x35, 0xF9, 0x6D, 0xD7, 0xF9, 0x2C, 0xD1, /* ..=k5.m..,. */
    0x97, 0x15, 0x87, 0x93, 0xA4, 0x49, 0x4A, 0x18, 0x16, 0x07, 0xA1, /* .....IJ.... */
    0x60, 0xF7, 0x52, 0x94, 0xDB, 0x02, 0x16, 0x70, 0xB2, 0xD8, 0x80, /* `.R....p... */
    0x30, 0xC2, 0x94, 0x40, 0x81, 0x74, 0x5A, 0x19, 0x7A, 0x80, 0x60, /* 0..@.tZ.z.` */
    0x41, 0x21, 0x46, 0x95, 0xD5, 0xC4, 0x40, 0xD2, 0x01, 0xC0, 0x01, /* A!F...@.... */
    0xDA, 0xD9, 0xA0, 0xB1, 0x01, 0xFF, 0xF2, 0x42, 0xC0, 0x82, 0x10, /* .......B... */
    0x0B, 0x12, 0xF9, 0x9E, 0xC9, 0x7E, 0x7A, 0xC6, 0x95, 0x55, 0x09, /* .....~z..U. */
    0x8B, 0x19, 0x5E, 0x8B, 0x26, 0xCA, 0xEB, 0x68, 0x8A, 0x05, 0x8F, /* ..^.&..h... */
    0x36, 0xA5, 0xA5, 0x03, 0xB8, 0x9C, 0xED, 0x24, 0x51, 0x59, 0x90, /* 6......$QY. */
    0xF6, 0xC5, 0x7D, 0xB5, 0xAD, 0xAF, 0xF6, 0x3B, 0x18, 0xEF, 0x3F, /* ..}....;..? */
    0xFF, 0xFF, 0x4E, 0xDE, 0x16, 0x66, 0x0B, 0xAA, 0x33, 0x23, 0xDD, /* ..N..f..3#. */
    0x9C, 0x4E, 0x6E, 0x55, 0x22, 0x9D, 0xA2, 0x40, 0xA6, 0x36, 0x31, /* .NnU"..@.61 */
    0x69, 0xA5, 0xE1, 0xD9, 0x7F, 0xF7, 0xC6, 0xCC, 0x48, 0x00, 0x0E, /* i.......H.. */
    0x90, 0x16, 0x00, 0x0F, 0xDE, 0x6E, 0x80, 0x11, 0x0C, 0x9A, 0x4F, /* .....n....O */
    0x56, 0xDB, 0x88, 0xD3, 0xB2, 0x1C, 0x00, 0xE0, 0x2E, 0x3E, 0xAC, /* V........>. */
    0xFF, 0xF2, 0x40, 0xC0, 0x1C, 0xE5, 0x19, 0x13, 0x31, 0x4E, 0xCD, /* ..@.....1N. */
    0x9E, 0xC3, 0x06, 0x71, 0x03, 0x85, 0xE5, 0xB5, 0x6D, 0x88, 0x50, /* ...q....m.P */
    0x8E, 0x0E, 0x17, 0x3B, 0x19, 0xFB, 0x4E, 0x3B, 0x99, 0xEF, 0x4C, /* ...;..N;..L */
    0x9E, 0xF7, 0x7B, 0x31, 0x7C, 0x3C, 0x5F, 0xFF, 0xF4, 0xF8, 0xE3, /* ..{1|<_.... */
    0x92, 0x42, 0x07, 0x8E, 0x83, 0x8E, 0x0F, 0x05, 0x08, 0x91, 0xA3, /* .B......... */
    0x16, 0xE2, 0xDF, 0xB7, 0x62, 0x60, 0x48, 0x31, 0x3C, 0xFF, 0xD4, /* ....b`H1<.. */
    0x9E, 0x0C, 0x68, 0x00, 0x77, 0x54, 0xE3, 0x1E, 0x05, 0xC5, 0xF8, /* ..h.wT..... */
    0xEA, 0x8D, 0x82, 0x9D, 0x08, 0xA9, 0x06, 0x8D, 0x1E, 0x5D, 0x7C, /* .........]| */
    0x7F, 0x08, 0xC0, 0x50, 0x45, 0x42, 0xD0, 0x36, 0xF8, 0xB2, 0x4D, /* ...PEB.6..M */
    0x53, 0x0C, 0x80, 0x3B, 0x4D, 0xFF, 0xF2, 0x42, 0xC0, 0x2F, 0x3C, /* S..;M..B./< */
    0x25, 0x19, 0x29, 0xFE, 0xBC, 0x2E, 0xC4, 0xD0, 0x99, 0x4C, 0x48, /* %.)......LH */
    0xB0, 0x9C, 0x49, 0xD2, 0x1A, 0x2D, 0x02, 0xC2, 0x79, 0x69, 0x16, /* ..I..-..yi. */
    0x92, 0xA8, 0xC5, 0xAB, 0x45, 0x5A, 0x68, 0xE8, 0x75, 0x57, 0xCD, /* ....EZh.uW. */
    0xF1, 0xB9, 0xAA, 0x13, 0x88, 0xE4, 0x87, 0x42, 0x15, 0xB3, 0x58, /* .......B..X */
    0xF5, 0xA3, 0x46, 0xB1, 0xCF, 0xD3, 0x59, 0x7E, 0xBA, 0xB5, 0xA7, /* ..F...Y~... */
    0x6B, 0x0B, 0x17, 0x57, 0x6B, 0x5C, 0x4A, 0xCD, 0x53, 0x76, 0x2A, /* k..Wk\J.Sv* */
    0x1D, 0x28, 0xC5, 0x1C, 0x76, 0x5C, 0xDD, 0x0A, 0x00, 0x4B, 0xC0, /* .(..v\...K. */
    0x1B, 0xCA, 0xA8, 0xE9, 0x81, 0x5B, 0xA6, 0xDC, 0xA4, 0x59, 0x13, /* .....[...Y. */
    0xFC, 0xBA, 0x8F, 0x98, 0x79, 0x44, 0x25, 0xC9, 0x35, 0x38, 0xCA, /* ....yD%.58. */
    0xFF, 0xF2, 0x40, 0xC0, 0xB9, 0x7D, 0x1A, 0x13, 0x79, 0x6A, 0xC8, /* ..@..}..yj. */
    0x3E, 0xC4, 0x46, 0x94, 0x8D, 0x3C, 0x67, 0x85, 0xB1, 0xA8, 0x89, /* >.F..<g.... */
    0xC0, 0xF2, 0xE6, 0x2F, 0x9D, 0x7C, 0xC9, 0xB4, 0xBE, 0xCF, 0xE1, /* .../.|..... */
    0x7D, 0xFE, 0x1F, 0x03, 0x00, 0x12, 0x84, 0x72, 0x8C, 0xE7, 0xD8, /* }......r... */
    0x5E, 0xC9, 0xA9, 0x01, 0xBA, 0x9B, 0xC4, 0x10, 0x5C, 0x70, 0x2E, /* ^.......\p. */
    0x6C, 0x48, 0xE7, 0x8C, 0x15, 0x0B, 0x06, 0x01, 0xE5, 0xFF, 0xFF, /* lH......... */
    0xD4, 0x0D, 0x00, 0x0F, 0xCE, 0x58, 0x95, 0x61, 0xA8, 0x9E, 0x7B, /* .....X.a..{ */
    0x19, 0x98, 0xB0, 0xF0, 0xC6, 0x72, 0x82, 0xD5, 0x27, 0x06, 0x47, /* .....r..'.G */
    0x41, 0x22, 0x0F, 0x65, 0x93, 0xC9, 0x8A, 0x09, 0x19, 0x48, 0x1B, /* A".e.....H. */
    0xBD, 0xD6, 0x64, 0x1A, 0xAC, 0xFF, 0xF2, 0x42, 0xC0, 0xF1, 0x11, /* ..d....B... */
    0x25, 0x14, 0x22, 0x06, 0xBC, 0x0E, 0xD4, 0x4E, 0x99, 0x90, 0xA8, /* %."....N... */
    0xD8, 0xB7, 0xAD, 0x5D, 0x3E, 0xAF, 0x6E, 0xBE, 0x66, 0x83, 0xA4, /* ...]>.n.f.. */
    0xE3, 0xC2, 0xE0, 0x29, 0x43, 0x87, 0x5F, 0x4F, 0x27, 0x9C, 0x2C, /* ...)C._O'., */
    0xD0, 0x91, 0xF3, 0x87, 0x9B, 0x54, 0xED, 0xD1, 0xB4, 0xF3, 0x39, /* .....T....9 */
    0x87, 0x22, 0x06, 0x86, 0x0D, 0x71, 0xE4, 0x6F, 0x2A, 0x08, 0x04, /* ."...q.o*.. */
    0xC0, 0x03, 0x2A, 0xB1, 0xE2, 0x05, 0x4D, 0x64, 0xA1, 0x9C, 0xA6, /* ..*...Md... */
    0x0D, 0x41, 0xA6, 0xF2, 0x7A, 0xC1, 0x30, 0xC3, 0x38, 0x26, 0x09, /* .A..z.0.8&. */
    0x50, 0x08, 0xC4, 0xF6, 0x30, 0x0C, 0xA6, 0xA9, 0x17, 0x00, 0x13, /* P...0...... */
    0x0C, 0xDC, 0xC4, 0x2F, 0x28, 0xEB, 0x3F, 0xCD, 0x7A, 0x3D, 0x2F, /* .../(.?.z=/ */
    0xFF, 0xF2, 0x40, 0xC0, 0x18, 0x6F, 0x2E, 0x13, 0xA1, 0xF2, 0xBC, /* ..@..o..... */
    0x36, 0xCB, 0x4E, 0x99, 0x6E, 0xFC, 0xEE, 0xC5, 0xF0, 0xA0, 0xB7, /* 6.N.n...... */
    0x92, 0xD4, 0xEE, 0x79, 0x7C, 0x50, 0x5D, 0xE5, 0x04, 0x94, 0xA9, /* ...y|P].... */
    0x76, 0xCF, 0x6C, 0x70, 0xDD, 0x0D, 0xD4, 0xEE, 0xED, 0x98, 0xE8, /* v.lp....... */
    0xC8, 0x35, 0x36, 0x7A, 0x0C, 0x05, 0x80, 0x03, 0xBC, 0xBE, 0x91, /* .56z....... */
    0x00, 0x7C, 0xAE, 0x65, 0xB8, 0x91, 0xA3, 0x33, 0xBA, 0x68, 0x60, /* .|.e...3.h` */
    0xD4, 0x1A, 0x66, 0xF8, 0x43, 0xA0, 0x20, 0x89, 0xE7, 0x80, 0xD8, /* ..f.C. .... */
    0x1E, 0x4F, 0xA0, 0x04, 0x60, 0x06, 0x0A, 0xA4, 0x91, 0x24, 0xFA, /* .O..`....$. */
    0x9F, 0x57, 0x53, 0xF4, 0x7A, 0xDB, 0x5F, 0x56, 0xE3, 0x6E, 0x0B, /* .WS.z._V.n. */
    0x8B, 0x3A, 0x1C, 0xF9, 0x5E, 0xFF, 0xF2, 0x42, 0xC0, 0xB1, 0x00, /* .:..^..B... */
    0x38, 0x14, 0x09, 0xEE, 0xB4, 0x36, 0xD3, 0x4E, 0x99, 0xA4, 0x78, /* 8....6.N..x */
    0x94, 0x73, 0xC4, 0x66, 0x30, 0xF5, 0xEA, 0xDB, 0xBA, 0x67, 0x67, /* .s.f0....gg */
    0x95, 0x6B, 0xAB, 0x68, 0x5D, 0x08, 0xA1, 0x39, 0x56, 0xAB, 0x1E, /* .k.h]..9V.. */
    0xD5, 0x03, 0xE8, 0x01, 0x70, 0x00, 0xB3, 0x93, 0x33, 0x19, 0x8C, /* ....p...3.. */
    0x61, 0x8F, 0xBB, 0x5D, 0x24, 0x12, 0x63, 0xD3, 0x4B, 0x5D, 0x91, /* a..]$.c.K]. */
    0x08, 0x43, 0x22, 0x56, 0x1A, 0xC5, 0x10, 0x21, 0x84, 0xA8, 0xEA, /* .C"V...!... */
    0x80, 0xBF, 0x16, 0x8E, 0x3D, 0x46, 0x18, 0x9C, 0x6E, 0x9A, 0x91, /* ....=F..n.. */
    0xE6, 0xC9, 0x6F, 0xD2, 0x7D, 0x27, 0xD7, 0xE9, 0x6B, 0xFF, 0x0A, /* ..o.}'..k.. */
    0x03, 0x43, 0x89, 0xD5, 0xBF, 0x52, 0x97, 0x0A, 0x25, 0x95, 0x0D, /* .C...R..%.. */
    0xFF, 0xF2, 0x40, 0xC0, 0xF5, 0xC3, 0x41, 0x13, 0x81, 0xEE, 0xA8, /* ..@...A.... */
    0x5E, 0xD3, 0x44, 0x98, 0xFC, 0xCF, 0x97, 0xF9, 0x58, 0xB5, 0x33, /* ^.D.....X.3 */
    0xB1, 0x85, 0x47, 0x86, 0xD7, 0x98, 0x01, 0x3B, 0xA3, 0x4F, 0x7E, /* ..G....;.O~ */
    0x04, 0xA6, 0xC3, 0x39, 0x21, 0x70, 0x27, 0x62, 0xB5, 0x18, 0x10, /* ...9!p'b... */
    0x09, 0x99, 0x00, 0x8B, 0x7E, 0xF2, 0xBF, 0x52, 0x18, 0x26, 0x30, /* ....~..R.&0 */
    0x1C, 0xB0, 0x01, 0x49, 0x30, 0xE0, 0xC3, 0x11, 0x46, 0x05, 0xCC, /* ...I0...F.. */
    0x49, 0x14, 0x28, 0xB2, 0xED, 0x4B, 0x57, 0x5A, 0x2F, 0xB7, 0x46, /* I.(..KWZ/.F */
    0x63, 0x34, 0xD2, 0xDA, 0x9F, 0x56, 0x32, 0xB7, 0xA2, 0x25, 0xFF, /* c4...V2..%. */
    0x94, 0x28, 0x33, 0x7F, 0x3B, 0xC4, 0x50, 0xEC, 0xB1, 0xE2, 0x26, /* .(3.;.P...& */
    0xA1, 0xB7, 0x07, 0x7F, 0xFB, 0xFF, 0xF2, 0x42, 0xC0, 0x67, 0x6A, /* .......B.gj */
    0x4C, 0x13, 0xF9, 0x6A, 0x90, 0x7E, 0xDB, 0x44, 0x94, 0x3F, 0xFF, /* L..j.~.D.?. */
    0x14, 0xD6, 0x2A, 0xFF, 0xFF, 0xC1, 0x34, 0x8C, 0x48, 0x22, 0x00, /* ..*...4.H". */
    0x06, 0x8F, 0x21, 0xFD, 0x64, 0x60, 0x04, 0x92, 0x42, 0xEA, 0x74, /* ..!.d`..B.t */
    0x32, 0x37, 0xAA, 0x5A, 0x9F, 0x67, 0x01, 0x8B, 0x3F, 0x37, 0x31, /* 27.Z.g..?71 */
    0xDD, 0x06, 0x3C, 0x01, 0x34, 0x30, 0xE0, 0x5C, 0x78, 0x78, 0xCB, /* ..<.40.\xx. */
    0xD6, 0xF1, 0x31, 0x8A, 0x69, 0x61, 0x93, 0x92, 0x42, 0xCE, 0x4B, /* ..1.ia..B.K */
    0xC5, 0x02, 0x4E, 0x73, 0xC6, 0x24, 0x30, 0xCD, 0x08, 0x66, 0xC6, /* ..Ns.$0..f. */
    0x35, 0xAB, 0xA2, 0x3D, 0x2F, 0xB3, 0xBD, 0x34, 0x87, 0x13, 0xEE, /* 5..=/..4... */
    0x71, 0x45, 0x68, 0xFA, 0xEA, 0x05, 0x84, 0x41, 0x36, 0x4C, 0x9A, /* qEh....A6L. */
    0xFF, 0xF2, 0x40, 0xC0, 0xC9, 0x92, 0x56, 0x13, 0xD0, 0x6E, 0x70, /* ..@...V..np */
    0x54, 0xD3, 0xCC, 0x28, 0x06, 0xD7, 0x0E, 0xA4, 0x1D, 0x9C, 0x9D, /* T..(....... */
    0xD9, 0xA9, 0x88, 0x7B, 0xB5, 0xA3, 0x56, 0xB7, 0x4B, 0x4B, 0x5A, /* ...{..V.KKZ */
    0x9B, 0x2C, 0xA9, 0xAD, 0x6F, 0x99, 0x6C, 0xC0, 0x4C, 0x14, 0x14, /* .,..o.l.L.. */
    0xEF, 0xB4, 0x20, 0x91, 0x5F, 0xBC, 0x81, 0x41, 0x41, 0x5D, 0xD4, /* .. ._..AA]. */
    0x20, 0xBD, 0x05, 0x1A, 0x6F, 0xE2, 0x68, 0x56, 0x41, 0x41, 0x57, /*  ...o.hVAAW */
    0xF9, 0xBF, 0x89, 0x82, 0x8E, 0xC7, 0x8F, 0x0A, 0x0A, 0x09, 0x37, /* ..........7 */
    0xF1, 0x05, 0x0A, 0x0A, 0x0A, 0x0A, 0x09, 0x05, 0x37, 0xFF, 0x10, /* ........7.. */
    0x50, 0x50, 0x53, 0x65, 0xFF, 0xFF, 0xFD, 0x75, 0xDF, 0xFF, 0xFF, /* PPSe...u... */
    0x68, 0x4F, 0xFF, 0x84, 0x70, 0xFF, 0xF2, 0x42, 0xC0, 0x27, 0x50, /* hO..p..B.'P */
    0x5F, 0x17, 0xE8, 0x82, 0x3C, 0x11, 0x58, 0x18, 0x01, 0x55, 0x48, /* _...<.X..UH */
    0xBC, 0x52, 0xFC, 0x4A, 0x4C, 0x3C, 0xD5, 0xF6, 0x11, 0x2D, 0xBF, /* .R.JL<...-. */
    0xEA, 0x03, 0x5C, 0x57, 0x29, 0xBF, 0xC3, 0x75, 0x1C, 0xE6, 0xDD, /* ..\W)..u... */
    0xBF, 0xED, 0xEF, 0xD0, 0x98, 0x77, 0x71, 0x95, 0x73, 0xFF, 0xED, /* .....wq.s.. */
    0x54, 0xBE, 0xD5, 0xEE, 0xAE, 0xC2, 0xD5, 0x0B, 0xFF, 0xF1, 0x97, /* T.......... */
    0x8A, 0xE4, 0x42, 0x09, 0x99, 0xB1, 0xEA, 0x94, 0xDC, 0x78, 0xB5, /* ..B......x. */
    0x34, 0x0F, 0xF1, 0x8F, 0xFC, 0x15, 0xF6, 0xFA, 0xB1, 0x47, 0xA9, /* 4........G. */
    0x6C, 0x67, 0x43, 0x8B, 0xF2, 0x76, 0x22, 0xED, 0xDA, 0x85, 0xBA, /* lgC..v".... */
    0x2F, 0xC7, 0xF9, 0xCF, 0xFC, 0xDB, 0x46, 0x2E, 0x50, 0x0A, 0x84, /* /.....F.P.. */
    0xFF, 0xF2, 0x40, 0xC0, 0xC6, 0x4A, 0x59, 0x28, 0x2B, 0x19, 0xE0, /* ..@..JY(+.. */
    0x01, 0x89, 0x78, 0x00, 0x52, 0x85, 0x3C, 0x8E, 0x54, 0x9A, 0x48, /* ..x.R.<.T.H */
    0x5A, 0x72, 0x32, 0x94, 0xBF, 0x43, 0x4F, 0x24, 0x53, 0x4B, 0xEC, /* Zr2..CO$SK. */
    0x4B, 0x99, 0x0E, 0x66, 0x1F, 0xFF, 0xCE, 0x7F, 0xFF, 0x3F, 0x10, /* K..f.....?. */
    0xAE, 0x82, 0x62, 0x71, 0x34, 0x18, 0x59, 0x9B, 0x51, 0xC7, 0x59, /* ..bq4.Y.Q.Y */
    0xCE, 0xEE, 0xA5, 0xFE, 0x02, 0xBB, 0x30, 0x91, 0x49, 0xD5, 0x4B, /* ......0.I.K */
    0xF3, 0xDC, 0x9A, 0xA9, 0x57, 0x8E, 0x72, 0x10, 0xC0, 0x5D, 0x60, /* ....W.r..]` */
    0x67, 0xFC, 0x7D, 0xD6, 0xBA, 0xDD, 0xB3, 0x8B, 0x5A, 0x0A, 0x4C, /* g.}.....Z.L */
    0x41, 0x4D, 0x45, 0x33, 0x2E, 0x39, 0x33, 0xAA, 0xAA, 0xAA, 0xAA, /* AME3.93.... */
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0x54, 0x41, 0x47, 0x48, 0x65, 0x6C, /* .....TAGHel */
    0x6C, 0x6F, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, /* lo          */
    0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, /*             */
    0x20, 0x20, 0x20, 0x20, 0x20, 0x50, 0x61, 0x6E, 0x75, 0x2D, 0x4B, /*      Panu-K */
    0x72, 0x69, 0x73, 0x74, 0x69, 0x61, 0x6E, 0x20, 0x50, 0x6F, 0x69, /* ristian Poi */
    0x6B, 0x73, 0x61, 0x6C, 0x6F, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, /* ksalo       */
    0x20, 0x20, 0x56, 0x53, 0x44, 0x53, 0x50, 0x20, 0x54, 0x65, 0x73, /*   VSDSP Tes */
    0x74, 0x69, 0x6E, 0x67, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, /* ting        */
    0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, /*             */
    0x20, 0x20, 0x20, 0x4D, 0x50, 0x33, 0x20, 0x48, 0x65, 0x6C, 0x6C, /*    MP3 Hell */
    0x6F, 0x2C, 0x20, 0x57, 0x6F, 0x72, 0x6C, 0x64, 0x21, 0x20, 0x20, /* o, World!   */
    0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, /*             */
    0x00 };

    //u0_dbg_printf("Running vTestMusic\n");

    initDecoder(&xdcs, &xcs, &xreset, &dreq);

    while(1)
    {
        sendSDIData(&xdcs, &dreq, HelloMP3, sizeof(HelloMP3));
        vTaskDelay(1);
    }

}

//void callbackDREQRising(void)
//{
//    xSemaphoreGiveFromISR(xSemaDREQRise, NULL);
//}



void controlDecoder(uint16_t *change_volume, uint8_t *volume_counter)
{
//    u0_dbg_printf("\nCurrent Volume Counter = %i\n", *volume_counter);
    //Play Music
    if(decoder_state == playSong)
    {
        vTaskResume(xDecoderHandle);
        //u0_dbg_printf("Resume Playing Song\n");
    }
    if(decoder_state == pauseSong)
    {
        vTaskSuspend(xDecoderHandle);
        //u0_dbg_printf("Pause Playing Song\n");
    }

    //volumeUp
    if(decoder_state == volumeUp)
    {
        u0_dbg_printf("Increment Volume\n");
        xQueueSend(volume_queue, &change_volume[*volume_counter], 10);
        decoder_state = pauseSong;
        decoder_state = playSong;
    }
    //volumeDown
    if(decoder_state == volumeDown)
    {
        u0_dbg_printf("Decrement Volume\n");
        xQueueSend(volume_queue, &change_volume[*volume_counter], 10);
        decoder_state = pauseSong;
        decoder_state = playSong;

    }
}

void PlayPauseISR() {
    xSemaphoreGiveFromISR(PlayPauseSem, NULL);
}


void vControlMP3Player(void *vParam)
{
    /**
     * Active Low GPIO Buttons
     */
    /**
     * Initialize Pins
     */
    GPIO SW_VOL_UP(P2_4), SW_VOL_DOWN(P2_5);
    eint3_enable_port0(0, eint_falling_edge, PlayPauseISR);


    /**
     * Initialize Constants
     */
    uint8_t volume_counter = 1;
    int8_t toggle = 1;
    uint16_t adjust_volume[] = {VOLUME_1, VOLUME_1, VOLUME_2, VOLUME_3, VOLUME_4, VOLUME_5, VOLUME_6, VOLUME_7, VOLUME_8};

    /**
     * Initialize Queues/Semaphores
     */
    volume_queue = xQueueCreate(1, sizeof(uint16_t));

    /**
     * Initialize MP3 Player Menu
     */
    while(1)
    {
        //Play or Pause MP3 Player
        if(xSemaphoreTake(PlayPauseSem, 10) == pdTRUE)
        {
            while(xSemaphoreTake(PlayPauseSem, 10) == pdTRUE);
            toggle *= -1;
            if(toggle == 1)
            {
                decoder_state = playSong;
            }
            else if(toggle == -1)
            {
                decoder_state = pauseSong;
            }
        }
        //Decrement Volume
            else if(!SW_VOL_DOWN.read())
            {
                while(!SW_VOL_DOWN.read())
                {
                    vTaskDelay(2);
                }
                decoder_state = volumeDown;
                volume_counter--;
                if(volume_counter >= 1)
                {
                     u0_dbg_printf("%i\n", volume_counter);
                     controlDecoder(adjust_volume, &volume_counter);
                     vTaskDelay(2);
                }
                else
                {
                    u0_dbg_printf("%i\n", volume_counter);
                     volume_counter = 1;
                     u0_dbg_printf("Lowest Volume\n");
                }

            }
            //Increment Volume
            else if(!SW_VOL_UP.read())
            {
                while(!SW_VOL_UP.read())
                {
                    vTaskDelay(2);
                }
                decoder_state = volumeUp;
                volume_counter++;
                if(volume_counter <= 9)
                {
                   u0_dbg_printf("%i\n", volume_counter);
                   controlDecoder(adjust_volume, &volume_counter);
                   vTaskDelay(2);
                }
                else
                {
                 u0_dbg_printf("%i\n", volume_counter);
                 volume_counter = 9;
                 u0_dbg_printf("Max Volume\n");
                }

            }
        }
     }

//NextISR would be activated and then pass semaphore to increment
//tracknumber counter
void NextISR() {
    xSemaphoreGiveFromISR(NextSem, NULL);
}
//PrevISR would be activated and then pass semaphore to decrement
//tracknumber counter
void PrevISR() {
    xSemaphoreGiveFromISR(PrevSem, NULL);
}
void MenuISR() {
    xSemaphoreGiveFromISR(MenuSem, NULL);
}

void vMenuSelect(void *param) {
//    /**
//     * Active Low GPIO Buttons
//     */
//    /**
//     * Initialize Pins
//     */
//

    u0_dbg_printf("lcd main code\n");
    eint3_enable_port0(1, eint_falling_edge, NextISR); //Next
    eint3_enable_port0(26, eint_falling_edge, PrevISR); //Next
    eint3_enable_port2(6, eint_falling_edge, MenuISR); //Next

//
//    /*
//     * LCD Display Peripherals
//     */
    PWM lcd_backlight(PWM::pwm4,1000);
    GPIO sce(P2_0), reset(P2_1), dc(P2_2);
    LCDDisplayNokia5110 lcd_display(&sce, &dc, &reset, &lcd_backlight);
    lcd_display.initDisplay();
    lcd_display.setContrast(40);

    if(spi1_bus_lock != NULL) {
        u0_dbg_printf("\nLCD Takes Semaphore\n");
        if(xSemaphoreTake(spi1_bus_lock, 200)) {
            u0_dbg_printf("Going to clear lcd\n");
            lcd_display.clearDisplay(lcd_display.BLACK);
            xSemaphoreGive(spi1_bus_lock);

        }
    }


    int count = 0;
    const char *filestart[6] = {"track001.mp3", "track002.mp3", "track003.mp3", "track004.mp3", "track005.mp3", "track006.mp3"};

    if(spi1_bus_lock != NULL) {
        u0_dbg_printf("\nLCD Takes Semaphore\n");
        if(xSemaphoreTake(spi1_bus_lock, 50)) {
            lcd_display.printString(0,  0, filestart[0], lcd_display.WHITE);
            lcd_display.printString(0,  8, filestart[1], lcd_display.BLACK);
            lcd_display.printString(0, 16, filestart[2], lcd_display.BLACK);
            lcd_display.printString(0, 24, filestart[3], lcd_display.BLACK);
            lcd_display.printString(0, 32, filestart[4], lcd_display.BLACK);
            lcd_display.printString(0, 40, filestart[5], lcd_display.BLACK);
            xSemaphoreGive(spi1_bus_lock);

        }
    }
    while(1) {
        //Go to Next Song in Menu
        if(xSemaphoreTake(NextSem, 50)) {
            while(xSemaphoreTake(NextSem, 50)) {
                vTaskDelay(2);
            }

            count++;
            u0_dbg_printf("count = %i\n", count);
            if(count >= 5) {
                count = 0;
            }
            if(spi1_bus_lock != NULL) {
                if(xSemaphoreTake(spi1_bus_lock, 50)) {
                    if(count == 0) {
                        lcd_display.printString(0,  0, filestart[0], lcd_display.WHITE);
                        lcd_display.printString(0,  8, filestart[1], lcd_display.BLACK);
                        lcd_display.printString(0, 16, filestart[2], lcd_display.BLACK);
                        lcd_display.printString(0, 24, filestart[3], lcd_display.BLACK);
                        lcd_display.printString(0, 32, filestart[4], lcd_display.BLACK);
                        lcd_display.printString(0, 40, filestart[5], lcd_display.BLACK);
                    }
                    else if(count == 1) {
                        lcd_display.printString(0,  0, filestart[0], lcd_display.BLACK);
                        lcd_display.printString(0,  8, filestart[1], lcd_display.WHITE);
                        lcd_display.printString(0, 16, filestart[2], lcd_display.BLACK);
                        lcd_display.printString(0, 24, filestart[3], lcd_display.BLACK);
                        lcd_display.printString(0, 32, filestart[4], lcd_display.BLACK);
                        lcd_display.printString(0, 40, filestart[5], lcd_display.BLACK);
                    }
                    else if(count == 2) {
                        lcd_display.printString(0,  0, filestart[0], lcd_display.BLACK);
                        lcd_display.printString(0,  8, filestart[1], lcd_display.BLACK);
                        lcd_display.printString(0, 16, filestart[2], lcd_display.WHITE);
                        lcd_display.printString(0, 24, filestart[3], lcd_display.BLACK);
                        lcd_display.printString(0, 32, filestart[4], lcd_display.BLACK);
                        lcd_display.printString(0, 40, filestart[5], lcd_display.BLACK);
                    }
                    else if(count == 3) {
                        lcd_display.printString(0,  0, filestart[0], lcd_display.BLACK);
                        lcd_display.printString(0,  8, filestart[1], lcd_display.BLACK);
                        lcd_display.printString(0, 16, filestart[2], lcd_display.BLACK);
                        lcd_display.printString(0, 24, filestart[3], lcd_display.WHITE);
                        lcd_display.printString(0, 32, filestart[4], lcd_display.BLACK);
                        lcd_display.printString(0, 40, filestart[5], lcd_display.BLACK);
                    }
                    else if(count == 4) {
                        lcd_display.printString(0,  0, filestart[0], lcd_display.BLACK);
                        lcd_display.printString(0,  8, filestart[1], lcd_display.BLACK);
                        lcd_display.printString(0, 16, filestart[2], lcd_display.BLACK);
                        lcd_display.printString(0, 24, filestart[3], lcd_display.BLACK);
                        lcd_display.printString(0, 32, filestart[4], lcd_display.WHITE);
                        lcd_display.printString(0, 40, filestart[5], lcd_display.BLACK);
                    }
                    else if(count == 5) {
                        lcd_display.printString(0,  0, filestart[0], lcd_display.BLACK);
                        lcd_display.printString(0,  8, filestart[1], lcd_display.BLACK);
                        lcd_display.printString(0, 16, filestart[2], lcd_display.BLACK);
                        lcd_display.printString(0, 24, filestart[3], lcd_display.BLACK);
                        lcd_display.printString(0, 32, filestart[4], lcd_display.BLACK);
                        lcd_display.printString(0, 40, filestart[5], lcd_display.WHITE);
                    }

        }

        //Go to Prev Song in Menu
        else if(xSemaphoreTake(PrevSem, 50)) {
            while(xSemaphoreTake(PrevSem, 50)) {
                vTaskDelay(2);
            }
            count--;
            u0_dbg_printf("count = %i\n", count);
            if(count <= 0 ) {
                count = 5;
            }
                    if(count == 0) {
                        lcd_display.printString(0,  0, filestart[0], lcd_display.WHITE);
                        lcd_display.printString(0,  8, filestart[1], lcd_display.BLACK);
                        lcd_display.printString(0, 16, filestart[2], lcd_display.BLACK);
                        lcd_display.printString(0, 24, filestart[3], lcd_display.BLACK);
                        lcd_display.printString(0, 32, filestart[4], lcd_display.BLACK);
                        lcd_display.printString(0, 40, filestart[5], lcd_display.BLACK);
                    }
                    else if(count == 1) {
                        lcd_display.printString(0,  0, filestart[0], lcd_display.BLACK);
                        lcd_display.printString(0,  8, filestart[1], lcd_display.WHITE);
                        lcd_display.printString(0, 16, filestart[2], lcd_display.BLACK);
                        lcd_display.printString(0, 24, filestart[3], lcd_display.BLACK);
                        lcd_display.printString(0, 32, filestart[4], lcd_display.BLACK);
                        lcd_display.printString(0, 40, filestart[5], lcd_display.BLACK);
                    }
                    else if(count == 2) {
                        lcd_display.printString(0,  0, filestart[0], lcd_display.BLACK);
                        lcd_display.printString(0,  8, filestart[1], lcd_display.BLACK);
                        lcd_display.printString(0, 16, filestart[2], lcd_display.WHITE);
                        lcd_display.printString(0, 24, filestart[3], lcd_display.BLACK);
                        lcd_display.printString(0, 32, filestart[4], lcd_display.BLACK);
                        lcd_display.printString(0, 40, filestart[5], lcd_display.BLACK);
                    }
                    else if(count == 3) {
                        lcd_display.printString(0,  0, filestart[0], lcd_display.BLACK);
                        lcd_display.printString(0,  8, filestart[1], lcd_display.BLACK);
                        lcd_display.printString(0, 16, filestart[2], lcd_display.BLACK);
                        lcd_display.printString(0, 24, filestart[3], lcd_display.WHITE);
                        lcd_display.printString(0, 32, filestart[4], lcd_display.BLACK);
                        lcd_display.printString(0, 40, filestart[5], lcd_display.BLACK);
                    }
                    else if(count == 4) {
                        lcd_display.printString(0,  0, filestart[0], lcd_display.BLACK);
                        lcd_display.printString(0,  8, filestart[1], lcd_display.BLACK);
                        lcd_display.printString(0, 16, filestart[2], lcd_display.BLACK);
                        lcd_display.printString(0, 24, filestart[3], lcd_display.BLACK);
                        lcd_display.printString(0, 32, filestart[4], lcd_display.WHITE);
                        lcd_display.printString(0, 40, filestart[5], lcd_display.BLACK);
                    }
                    else if(count == 5) {
                        lcd_display.printString(0,  0, filestart[0], lcd_display.BLACK);
                        lcd_display.printString(0,  8, filestart[1], lcd_display.BLACK);
                        lcd_display.printString(0, 16, filestart[2], lcd_display.BLACK);
                        lcd_display.printString(0, 24, filestart[3], lcd_display.BLACK);
                        lcd_display.printString(0, 32, filestart[4], lcd_display.BLACK);
                        lcd_display.printString(0, 40, filestart[5], lcd_display.WHITE);
                    }
                    xSemaphoreGive(spi1_bus_lock);
                }
            }
        }

        else if(xSemaphoreTake(MenuSem, 50)) {
            while(xSemaphoreTake(MenuSem, 50)) {
                vTaskDelay(2);
            }
            if(spi1_bus_lock != NULL) {
                if(xSemaphoreTake(spi1_bus_lock, 50)) {
                    lcd_display.clearDisplay(lcd_display.WHITE);
                    switch(count)
                    {
                        case 0:
                            lcd_display.printString(15,  0, track001.song_title, lcd_display.BLACK);
                            lcd_display.printString(15,  8, track001.artist, lcd_display.BLACK);
                            lcd_display.printString(15, 16, track001.album, lcd_display.BLACK);
                            break;
                        case 1:
                            lcd_display.printString(15,  0, track002.song_title, lcd_display.BLACK);
                            lcd_display.printString(15,  8, track002.artist, lcd_display.BLACK);
                            lcd_display.printString(15, 16, track002.album, lcd_display.BLACK);
                            break;
                        case 2:
                            lcd_display.printString(15,  0, track003.song_title, lcd_display.BLACK);
                            lcd_display.printString(15,  8, track003.artist, lcd_display.BLACK);
                            lcd_display.printString(15, 16, track003.album, lcd_display.BLACK);
                            break;
                        case 3:
                            lcd_display.printString(15,  0, track004.song_title, lcd_display.BLACK);
                            lcd_display.printString(15,  8, track004.artist, lcd_display.BLACK);
                            lcd_display.printString(15, 16, track004.album, lcd_display.BLACK);
                            break;
                        case 4:
                            lcd_display.printString(15,  0, track005.song_title, lcd_display.BLACK);
                            lcd_display.printString(15,  8, track005.artist, lcd_display.BLACK);
                            lcd_display.printString(15, 16, track005.album, lcd_display.BLACK);
                            break;
                        case 5:
                            lcd_display.printString(15,  0, track006.song_title, lcd_display.BLACK);
                            lcd_display.printString(15,  8, track006.artist, lcd_display.BLACK);
                            lcd_display.printString(15, 16, track006.album, lcd_display.BLACK);
                            break;
                        default:
                            u0_dbg_printf("\nSong Metadata\n");
                            break;
                    }
                    xSemaphoreGive(spi1_bus_lock);
                }
            }

        }
    vTaskDelay(50);
    }
}

void runSDCard()
{

    audio_queue = xQueueCreate(1, sizeof(mp3_file_data));
    //we utilize terminal task to execute our suspend and resume producer(getLightData) and consumer(pullLightData) tasks
    //scheduler_add_task(new terminalTask(PRIORITY_HIGH));
    xTaskCreate(testReadSDCard, "testSDCard", 2048, NULL, 3, NULL);

    //scheduler_start();
}

void runController()
{
    xTaskCreate(vControlMP3Player, "MP3Controller", 2048, NULL, 4, NULL);
}

void runLCDDisplay(){
    u0_dbg_printf("lcd display\n");
    xTaskCreate(vMenuSelect, "LCDDisplay", 2048, NULL, 1, NULL);
}

void runMP3Decoder()
{
    //u0_dbg_printf("inside runMP3Decoder\n");

    //xSemaDREQRise = xSemaphoreCreateBinary();

    //Enable Rising Edge Interrupts on DREQ P0.1, no need to configure it using GPIO class
    //eint3_enable_port0(1, eint_rising_edge, callbackDREQRising);

    //xTaskCreate(vTestMP3Decoder, "sineTest", 1024, NULL, 1, NULL);
    xTaskCreate(vPlayMP3Music, "playMP3Music", 2048, NULL, 1, &xDecoderHandle);
    //xTaskCreate(vPlayMP3Sample, "playMP3Sample", 1024, NULL, 1, NULL);
}
