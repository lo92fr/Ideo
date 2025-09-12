#ifndef cc1100_H
#define cc1100_H

#include <stdint.h>


/*----------------------------------[standard]--------------------------------*/
#define TRUE  (1==1)
#define FALSE (!TRUE)


//**************************** pins ******************************************//
#define SS_PIN   10
#define SCK_PIN   14
#define GDO2     6
#define GDO0     5
#define MOSI_PIN     12
#define MISO_PIN     13

#define SPI_CHIP_SELECT   0

/*----------------------[CC1100 - misc]---------------------------------------*/
#define CRYSTAL_FREQUENCY         26000000
#define CFG_REGISTER              0x2F  //47 registers
#define FIFOBUFFER                0x42  //size of Fifo Buffer +2 for rssi and lqi
#define RSSI_OFFSET_868MHZ        0x4E  //dec = 74
#define TX_RETRIES_MAX            0x05  //tx_retries_max
#define ACK_TIMEOUT                250  //ACK timeout in ms
#define CC1100_COMPARE_REGISTER   0x00  //register compare 0=no compare 1=compare
#define BROADCAST_ADDRESS         0x00  //broadcast address
#define CC1100_FREQ_315MHZ        0x01
#define CC1100_FREQ_434MHZ        0x02
#define CC1100_FREQ_868MHZ        0x03
#define CC1100_FREQ_915MHZ        0x04
//#define CC1100_FREQ_2430MHZ       0x05
#define CC1100_TEMP_ADC_MV        3.225 //3.3V/1023 . mV pro digit
#define CC1100_TEMP_CELS_CO       2.47  //Temperature coefficient 2.47mV per Grad Celsius

/*---------------------------[CC1100 - R/W offsets]---------------------------*/
#define WRITE_SINGLE_BYTE   0x00
#define WRITE_BURST         0x40
#define READ_SINGLE_BYTE    0x80
#define READ_BURST          0xC0
/*---------------------------[END R/W offsets]--------------------------------*/

/*------------------------[CC1100 - FIFO commands]----------------------------*/
#define TXFIFO_BURST        0x7F    //write burst only
#define TXFIFO_SINGLE_BYTE  0x3F    //write single only
#define RXFIFO_BURST        0xFF    //read burst only
#define RXFIFO_SINGLE_BYTE  0xBF    //read single only
#define PATABLE_BURST       0x7E    //power control read/write
#define PATABLE_SINGLE_BYTE 0xFE    //power control read/write
/*---------------------------[END FIFO commands]------------------------------*/

 typedef struct
    {
        uint8_t iocfg2;   // GDO2 Output Pin Configuration
        uint8_t iocfg1;   // GDO1 Output Pin Configuration
        uint8_t iocfg0;   // GDO0 Output Pin Configuration
        uint8_t fifothr;  // RX FIFO and TX FIFO Thresholds
        uint8_t sync1;    // Sync Word, High Byte
        uint8_t sync0;    // Sync Word, Low Byte
        uint8_t pktlen;   // Packet Length
        uint8_t pktctrl1; // Packet Automation Control
        uint8_t pktctrl0; // Packet Automation Control
        uint8_t addr;     // Device Address
        uint8_t channr;   // Channel Number
        uint8_t fsctrl1;  // Frequency Synthesizer Control
        uint8_t fsctrl0;  // Frequency Synthesizer Control
        uint8_t freq2;    // Frequency Control Word, High Byte
        uint8_t freq1;    // Frequency Control Word, Middle Byte
        uint8_t freq0;    // Frequency Control Word, Low Byte
        uint8_t mdmcfg4;  // Modem Configuration
        uint8_t mdmcfg3;  // Modem Configuration
        uint8_t mdmcfg2;  // Modem Configuration
        uint8_t mdmcfg1;  // Modem Configuration
        uint8_t mdmcfg0;  // Modem Configuration
        uint8_t deviatn;  // Modem Deviation Setting
        uint8_t mcsm2;    // Main Radio Control State Machine Configuration
        uint8_t mcsm1;    // Main Radio Control State Machine Configuration
        uint8_t mcsm0;    // Main Radio Control State Machine Configuration
        uint8_t foccfg;   // Frequency Offset Compensation Configuration
        uint8_t bscfg;    // Bit Synchronization Configuration
        uint8_t agcctrl2; // AGC Control
        uint8_t agcctrl1; // AGC Control
        uint8_t agcctrl0; // AGC Control
        uint8_t worevt1;  // High Byte Event0 Timeout
        uint8_t worevt0;  // Low Byte Event0 Timeout
        uint8_t worctrl;  // Wake On Radio Control
        uint8_t frend1;   // Front End RX Configuration
        uint8_t frend0;   // Front End TX Configuration
        uint8_t fscal3;   // Frequency Synthesizer Calibration
        uint8_t fscal2;   // Frequency Synthesizer Calibration
        uint8_t fscal1;   // Frequency Synthesizer Calibration
        uint8_t fscal0;   // Frequency Synthesizer Calibration
        uint8_t rcctrl1;  // RC Oscillator Configuration
        uint8_t rcctrl0;  // RC Oscillator Configuration
        uint8_t fstest;   // Frequency Synthesizer Calibration Control
        uint8_t ptest;    // Production Test
        uint8_t agctest;  // AGC Test
        uint8_t test2;    // Various Test Settings
        uint8_t test1;    // Various Test Settings
        uint8_t test0;    // Various Test Settings
    } Configuration;
	
 enum class Register : uint8_t
{
	IOCFG2 = 0x00,   // GDO2 output pin configuration
	IOCFG1 = 0x01,   // GDO1 output pin configuration
	IOCFG0 = 0x02,   // GDO0 output pin configuration
	FIFOTHR = 0x03,  // RX FIFO and TX FIFO thresholds
	SYNC1 = 0x04,    // Sync word, high byte
	SYNC0 = 0x05,    // Sync word, low byte
	PKTLEN = 0x06,   // Packet length
	PKTCTRL1 = 0x07, // Packet automation control
	PKTCTRL0 = 0x08, // Packet automation control
	ADDR = 0x09,     // Device address
	CHANNR = 0x0A,   // Channel number
	FSCTRL1 = 0x0B,  // Frequency synthesizer control
	FSCTRL0 = 0x0C,  // Frequency synthesizer control
	FREQ2 = 0x0D,    // Frequency control word, high byte
	FREQ1 = 0x0E,    // Frequency control word, middle byte
	FREQ0 = 0x0F,    // Frequency control word, low byte
	MDMCFG4 = 0x10,  // Modem configuration
	MDMCFG3 = 0x11,  // Modem configuration
	MDMCFG2 = 0x12,  // Modem configuration
	MDMCFG1 = 0x13,  // Modem configuration
	MDMCFG0 = 0x14,  // Modem configuration
	DEVIATN = 0x15,  // Modem deviation setting
	MCSM2 = 0x16,    // Main Radio Cntrl State Machine config
	MCSM1 = 0x17,    // Main Radio Cntrl State Machine config
	MCSM0 = 0x18,    // Main Radio Cntrl State Machine config
	FOCCFG = 0x19,   // Frequency Offset Compensation config
	BSCFG = 0x1A,    // Bit Synchronization configuration
	AGCCTRL2 = 0x1B, // AGC control
	AGCCTRL1 = 0x1C, // AGC control
	AGCCTRL0 = 0x1D, // AGC control
	WOREVT1 = 0x1E,  // High byte Event 0 timeout
	WOREVT0 = 0x1F,  // Low byte Event 0 timeout
	WORCTRL = 0x20,  // Wake On Radio control
	FREND1 = 0x21,   // Front end RX configuration
	FREND0 = 0x22,   // Front end TX configuration
	FSCAL3 = 0x23,   // Frequency synthesizer calibration
	FSCAL2 = 0x24,   // Frequency synthesizer calibration
	FSCAL1 = 0x25,   // Frequency synthesizer calibration
	FSCAL0 = 0x26,   // Frequency synthesizer calibration
	RCCTRL1 = 0x27,  // RC oscillator configuration
	RCCTRL0 = 0x28,  // RC oscillator configuration
	FSTEST = 0x29,   // Frequency synthesizer cal control
	PTEST = 0x2A,    // Production test
	AGCTEST = 0x2B,  // AGC test
	TEST2 = 0x2C,    // Various test settings
	TEST1 = 0x2D,    // Various test settings
	TEST0 = 0x2E,    // Various test settings
};



 enum class ControlState : uint8_t
{
	SLEEP = 0x00,
	IDLE = 0x01,
	XOFF = 0x02,
	VCOON_MC = 0x03,
	REGON_MC = 0x04,
	MANCAL = 0x05,
	VCOON = 0x06,
	REGON = 0x07,
	STARTCAL = 0x08,
	BWBOOST = 0x09,
	FS_LOCK = 0x0A,
	IFADCON = 0x0B,
	ENDCAL = 0x0C,
	RX = 0x0D,
	RX_END = 0x0E,
	RX_RST = 0x0F,
	TXRX_SWITCH = 0x10,
	RXFIFO_OVERFLOW = 0x11,
	FSTXON = 0x12,
	TX = 0x13,
	TX_ON = 0x14,
	RXTX_SWITCH = 0x15,
	TXFIFO_UNDERFLOW = 0x16
};



enum class StrobeCommand : uint8_t
{
	SRES = 0x30,    // Reset chip
	SFSTXON = 0x31, // Enable/calibrate freq synthesizer
	SXOFF = 0x32,   // Turn off crystal oscillator.
	SCAL = 0x33,    // Calibrate freq synthesizer & disable
	SRX = 0x34,     // Enable RX.
	STX = 0x35,     // Enable TX.
	SIDLE = 0x36,   // Exit RX / TX
	SAFC = 0x37,    // AFC adjustment of freq synthesizer
	SWOR = 0x38,    // Start automatic RX polling sequence
	SPWD = 0x39,    // Enter pwr down mode when CSn goes hi
	SFRX = 0x3A,    // Flush the RX FIFO buffer.
	SFTX = 0x3B,    // Flush the TX FIFO buffer.
	SWORRST = 0x3C, // Reset real time clock.
	SNOP = 0x3D,    // No operation.
};

enum class StatusRegister : uint8_t
{
	PARTNUM = 0xF0,        // Part number
	VERSION = 0xF1,        // Current version number
	FREQEST = 0xF2,        // Frequency offset estimate
	LQI = 0xF3,            // Demodulator estimate for link quality
	RSSI = 0xF4,           // Received signal strength indication
	MARCSTATE = 0xF5,      // Control state machine state
	WORTIME1 = 0xF6,       // High byte of WOR timer
	WORTIME0 = 0xF7,       // Low byte of WOR timer
	PKTSTATUS = 0xF8,      // Current GDOx status and packet status
	VCO_VC_DAC = 0xF9,     // Current setting from PLL cal module
	TXBYTES = 0xFA,        // Underflow and # of bytes in TXFIFO
	RXBYTES = 0xFB,        // Overflow and # of bytes in RXFIFO
	RCCTRL1_STATUS = 0xFC, // Last RC Oscillator Calibration Result
	RCCTRL0_STATUS = 0xFD, // Last RC Oscillator Calibration Result
};

struct TxPacketData
  {
    char device;
    uint8_t command;
    char params[8];
  };

struct RawPacket
{
  uint16_t header;
  char device;
  uint8_t command;
  char params[8];
  uint16_t checksum;
  uint16_t footer;
};


struct RawRxPacket
{
  uint16_t header;
  char device;
  uint8_t command;
  char params[8];
  uint16_t checksum;
  uint16_t footer;
  int8_t rssi;
  uint8_t lqi;
};

 struct RxPacketData
  {
    char device;
    uint8_t command;
    char params[8];
    int8_t rssi;
    uint8_t lqi;
  };


static const char nibbleLut[] = "0123456789ABCDEF";


class CC1100
{
    private:

        void spi_begin(void);
        void spi_end(void);
        uint8_t spi_putc(uint8_t data);

    public:
      uint8_t debug_level;

      uint8_t set_debug_level(uint8_t set_debug_level);
      uint8_t get_debug_level(void);

      uint8_t begin(volatile uint8_t &My_addr);
      void end(void);
		
		  void goIdle();
		  void goReceive();
		  void goTransmit();

      void spi_write_strobe(StrobeCommand spi_instr);
      void spi_write_register(Register spi_instr, uint8_t value);
      void spi_write_burst(uint8_t spi_instr, uint8_t *pArr, uint8_t length);
      void spi_read_burst(uint8_t spi_instr, uint8_t *pArr, uint8_t length);
        
		  uint8_t spi_read_register(Register spi_instr);
      uint8_t spi_read_status(StatusRegister spi_instr);
		
		  uint8_t spi_read_register_i(uint8_t spi_instr);
		
		  void writeConfiguration(Configuration *config);

      void reset(void);
      void wakeup(void);
      void powerdown(void);

      void wor_enable(void);
      void wor_disable(void);
      void wor_reset(void);

		
		  ControlState getState();

      void show_register_settings(void);
      void show_main_settings(void);

      uint8_t packet_available();
      uint8_t wait_for_packet(uint16_t milliseconds);

      uint8_t get_payload(uint8_t rxbuffer[], uint8_t &pktlen_rx,uint8_t &my_addr,
                                      uint8_t &sender, int8_t &rssi_dbm, uint8_t &lqi);

      uint8_t tx_payload_burst(uint8_t my_addr, uint8_t rx_addr, uint8_t *txbuffer, uint8_t length);
      uint8_t tx_payload_burst2(uint8_t my_addr, uint8_t rx_addr, uint8_t *txbuffer, uint8_t length);
      uint8_t rx_payload_burst(uint8_t rxbuffer[], uint8_t &pktlen);

      void rx_fifo_erase(uint8_t *rxbuffer);
      void tx_fifo_erase(uint8_t *txbuffer);

      uint8_t sent_packet(uint8_t my_addr, uint8_t rx_addr, uint8_t *txbuffer, uint8_t pktlen, uint8_t tx_retries);
      void sent_acknowledge(uint8_t my_addr, uint8_t tx_addr);

      uint8_t check_acknowledge(uint8_t *rxbuffer, uint8_t pktlen, uint8_t sender, uint8_t my_addr);

      int8_t rssi_convert(uint8_t Rssi);
      uint8_t check_crc(uint8_t lqi);
      uint8_t lqi_convert(uint8_t lqi);
      uint8_t get_temp(uint8_t *ptemp_Arr);

      uint8_t getNumTxBytes();
      bool isRxOverflow();
      bool isTxUnderflow();

      int8_t rssiToDbm(uint8_t rawRssi);

      void set_myaddr(uint8_t addr);
      void set_channel(uint8_t channel);
      void set_ISM(uint8_t ism_freq);
      void set_mode(uint8_t mode);
      void set_output_power_level(int8_t dbm);
      void set_patable(uint8_t *patable_arr);
      void set_fec(uint8_t cfg);
      void set_data_whitening(uint8_t cfg);
      void set_modulation_type(uint8_t cfg);
      void set_preamble_len(uint8_t cfg);
      void set_manchester_encoding(uint8_t cfg);
      void set_sync_mode(uint8_t cfg);
      void set_datarate(uint8_t mdmcfg4, uint8_t mdmcfg3, uint8_t deviant);
		
		  void printPacket(const RawRxPacket *packet);
      void printTxPacket(const RawPacket *packet);
		  void printParams(const char *params);
		  uint8_t parseNibble(char param);
		  uint16_t parseUint16(const char *param);
		  uint16_t computeChecksum(const RawPacket *pkt);

      void sendPacket(TxPacketData *packet);
      void writeTxFifo(uint8_t *data, uint8_t length);
};



//=======================[CC1100 special functions]=============================


#endif // CC1100_H
