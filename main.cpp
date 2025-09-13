#include "wiringPi.h"
#include "cc1100_raspi.h"
#include <mosquitto.h>
#include <iostream>
#include <ctime>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <cstdio>
#include <string.h>
#include <wiringPiSPI.h>

#define PACKAGE    "CC1100 SW"
#define VERSION_SW "0.9.1"



//#define IOBL_SS_PIN 3
#define IOBL_INT_PIN 6
uint8_t My_addr = 0, Tx_addr, Rx_addr, Pktlen, pktlen, Lqi, Rssi;
uint8_t rx_addr,sender,lqi;
int8_t rssi_dbm;


// Met le terminal en mode non-bloquant et non-canonique
void initKeyboard()
{
    termios term;
    tcgetattr(STDIN_FILENO, &term);
    term.c_lflag &= ~(ICANON | ECHO); // mode non-canonique, pas d'echo
    tcsetattr(STDIN_FILENO, TCSANOW, &term);

    // set non-blocking
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
}
void resetKeyboard()
{
    termios term;
    tcgetattr(STDIN_FILENO, &term);
    term.c_lflag |= (ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &term);

    // remet bloquant
    fcntl(STDIN_FILENO, F_SETFL, 0);
}

// Vérifie si une touche a été pressée
int kbhit()
{
    unsigned char ch;
    if (read(STDIN_FILENO, &ch, 1) > 0)
        return ch; // retourne le caractère lu
    return 0;
}


static Configuration cc1100_ideo = {
                    0x01,  // IOCFG2        GDO2 Output Pin Configuration
                    0x2E,  // IOCFG1        GDO1 Output Pin Configuration
                    0x80,  // IOCFG0        GDO0 Output Pin Configuration
                    0x07,  // FIFOTHR       RX FIFO and TX FIFO Thresholds
					
                    0x2D,  // SYNC1         Sync Word, High Byte
                    0x00,  // SYNC0         Sync Word, Low Byte
                    0x10,  // PKTLEN        Packet Length
                    0x04,  // PKTCTRL1      Packet Automation Control
                    0x00,  // PKTCTRL0      Packet Automation Control
                    0x00,  // ADDR          Device Address
                    0x00,  // CHANNR        Channel Number
                    0x06,  // FSCTRL1       Frequency Synthesizer Control
                    0x00,  // FSCTRL0       Frequency Synthesizer Control
					
                    0x21,  // FREQ2         Frequency Control Word, High Byte
                    0x65,  // FREQ1         Frequency Control Word, Middle Byte
                    0xC3,  // FREQ0         Frequency Control Word, Low Byte
					
                    0x48,  // MDMCFG4       Modem Configuration
                    0x83,  // MDMCFG3       Modem Configuration
					
                    0x01,  // MDMCFG2       Modem Configuration
                    0x02,  // MDMCFG1       Modem Configuration
                    0xF8,  // MDMCFG0       Modem Configuration
					
                    0x54,  // DEVIATN       Modem Deviation Setting
                    0x07,  // MCSM2         Main Radio Control State Machine Configuration
                    0x0F,  // MCSM1         Main Radio Control State Machine Configuration
                    0x18,  // MCSM0         Main Radio Control State Machine Configuration
                    0x16,  // FOCCFG        Frequency Offset Compensation Configuration
                    0x6C,  // BSCFG         Bit Synchronization Configuration
                    0x07,  // AGCCTRL2      AGC Control
                    0x40,  // AGCCTRL1      AGC Control
                    0x91,  // AGCCTRL0      AGC Control
                    0x87,  // WOREVT1       High Byte Event0 Timeout
                    0x6B,  // WOREVT0       Low Byte Event0 Timeout
                    0x09,  // WORCTRL       Wake On Radio Control
                    0x56,  // FREND1        Front End RX Configuration
                    0x10,  // FREND0        Front End TX Configuration
                    0xE9,  // FSCAL3        Frequency Synthesizer Calibration
                    0x2A,  // FSCAL2        Frequency Synthesizer Calibration
                    0x00,  // FSCAL1        Frequency Synthesizer Calibration
                    0x1F,  // FSCAL0        Frequency Synthesizer Calibration
                    0x41,  // RCCTRL1       RC Oscillator Configuration
                    0x00,  // RCCTRL0       RC Oscillator Configuration
                    0x59,  // FSTEST        Frequency Synthesizer Calibration Control
                    0x7F,  // PTEST         Production Test
                    0x3F,  // AGCTEST       AGC Test
                    0x81,  // TEST2         Various Test Settings
                    0x35,  // TEST1         Various Test Settings
                    0x09,  // TEST0         Various Test Settings
               };			   




CC1100 cc1100;
mosquitto *mosq = nullptr;
int cc1100_freq_select, cc1100_mode_select, cc1100_channel_select;

bool isPacketAvailable;
RxPacketData lastRxPacket;

std::string pad2(int n) {
    if (n < 10)
        return "0" + std::to_string(n);
    else
        return std::to_string(n);
}

void packetHandler(RawRxPacket *packet) {
  uint16_t param_1 = cc1100.parseUint16(packet->params);
  uint16_t param_2 = cc1100.parseUint16(&packet->params[4]);
  uint8_t bypassmode = 0;
  uint8_t onOffState = 0;
  uint8_t fanmode = 0;
  std::string str;
  const char *message;
  

  switch (packet->command)
  {
    case 0x31:
        if (param_1==0 && param_2==0) {
            return;
        }
		message = std::to_string( param_1/10.00).c_str();
    	mosquitto_publish(mosq, nullptr, "vmc/state/temperatureSondeAirVicie", strlen(message), message, 0, false);

		message = std::to_string( param_2/10.00).c_str();
    	mosquitto_publish(mosq, nullptr, "vmc/state/temperatureSondeEntreeAirNeuf", strlen(message), message, 0, false);

        break;

    case 0x32:
        if (param_1==0 && param_2==0) {
            return;
        }
	    message = std::to_string( param_1/10.00).c_str();
    	mosquitto_publish(mosq, nullptr, "vmc/state/temperatureSondeEntreeAirExterieure", strlen(message), message, 0, false);
        break;

    case 0x33:
        if (param_1==0 && param_2==0) {
            return;
        }
        bypassmode = ((param_2 >> 12) & 0xf);
        onOffState = (param_2 & 0xFF)==255;
        fanmode = param_1;

	    message = std::to_string(bypassmode).c_str();
    	mosquitto_publish(mosq, nullptr, "vmc/state/bypassmode", strlen(message), message, 0, false);

	    message = std::to_string(fanmode).c_str();
    	mosquitto_publish(mosq, nullptr, "vmc/state/fanMode", strlen(message), message, 0, false);

        message = "OFF";
        if (onOffState==1) {
            message = "ON";
        }
	    mosquitto_publish(mosq, nullptr, "vmc/state/onOffState", strlen(message), message, 0, false);
        break;
    case 0x3A:
        if ((param_2 & 0xFF) != 0xFF)
        {
            uint16_t day = (param_2 & 0xff);
            uint16_t hour = (param_2 >> 8);
            uint16_t minute = (param_1 & 0xff);

            if (day>7) {
                return;
            }

            printf("day: %d\n", day);
            printf("hour: %d\n", hour);
            printf("minute: %d\n", minute);
            printf("param_1: %d\n", param_1);
            printf("param_2: %d\n", param_2);

            std::time_t t = std::time(nullptr);
            std::tm* now = std::localtime(&t);

            int month = now->tm_mon + 1;
            int year = now->tm_year + 1900;


            str = std::to_string(year) + "-" + pad2(month) + "-" + pad2(day) + " " + std::to_string(hour) + ":" + std::to_string(minute);
	        message = str.c_str();
            printf("date: %s\n", message);
    	    mosquitto_publish(mosq, nullptr, "vmc/state/date", strlen(message), message, 0, false);
        }
    case 0x3B:
        if (param_2 == 0)
        {
	        message = std::to_string(param_1).c_str();
    	    mosquitto_publish(mosq, nullptr, "vmc/state/schedule", strlen(message), message, 0, false);
        }
        break;
 
    case 0x3C:
        if (param_2 == 0)
        {
	        message = std::to_string(param_1).c_str();
    	    mosquitto_publish(mosq, nullptr, "vmc/state/lowFanSpeed", strlen(message), message, 0, false);
        }
        break;

    case 0x3D:
        if (param_2 == 0)
        {
	        message = std::to_string(param_1).c_str();
    	    mosquitto_publish(mosq, nullptr, "vmc/state/highFanSpeed", strlen(message), message, 0, false);
        }
        break;
    case 0x3E:
        if (param_2 == 0)
        {
        }
        break;
    case 0x3F:
        if (param_2 == 0)
        {
        }
        break;
    case 0x41:
        if (param_2 == 0)
        {
            message = "OFF";
            if (param_1==1) {
                message = "ON";
            }
    	    
            mosquitto_publish(mosq, nullptr, "vmc/state/holidayMode", strlen(message), message, 0, false);
        }
        break;
    case 0x58:
        // Set Contact Polarity
        break;
    case 0x59:
        // Dirty filter
        break;
    case 0x5A:
        // 
        break;
    case 0x5B:
        // 
        break;
    case 0x5C:
        // 
        break;
    case 0x5D:
        //         
        break;
    case 0x80:
        //         
        break;
    default:
        printf("Unknown command: 0x%02X\n", packet->command);
        break;
  }

}	

void rfCallback()
{
  	RawRxPacket rxPacket;

  	cc1100.spi_read_burst(RXFIFO_BURST, (uint8_t *)&rxPacket, sizeof(RawRxPacket));


  	if (rxPacket.header == 0x3001 &&
      rxPacket.footer == 0x0003 &&
      rxPacket.checksum == cc1100.computeChecksum((RawPacket *)&rxPacket))  {
    
		isPacketAvailable = true;
		lastRxPacket.device = rxPacket.device;
		lastRxPacket.command = rxPacket.command;
		for (int i=0;i<8;i++) {
			lastRxPacket.params[i] = rxPacket.params[i];
		}
    	lastRxPacket.rssi = cc1100.rssiToDbm(rxPacket.rssi);
    	lastRxPacket.lqi = rxPacket.lqi & 0x7F;

		cc1100.printPacket(&rxPacket);	
		packetHandler(&rxPacket);
	}

  	if (cc1100.isRxOverflow()) {
		printf("RX Overflow\n");
	    cc1100.spi_write_strobe(StrobeCommand::SFRX);
	}

	cc1100.goReceive();
}


static const char *delims = ",";




bool parseMessage(const char *message,TxPacketData *tx) {
	uint8_t ntok = 0;

  char* data = strdup(message);
  char *token = strtok(data, delims);

  while (token != NULL)
    {
        switch (ntok++)
        {
        case 0:
            if (strlen(token) != 1)
            {
                printf("Device ID must be a 1-byte token. Got: ");
                printf("%s\n", token);
                return false;
            }
            tx->device = token[0];
            break;

        case 1:
            if (strlen(token) != 2)
            {
                printf("Command must be a 2-byte token. Got: ");
                printf("%s", token);
                return false;
            }
            tx->command = (cc1100.parseNibble(token[0]) << 4) | cc1100.parseNibble(token[1]);
            break;

        case 2:
            if (strlen(token) != 8)
            {
                printf("Command parameters must be a 8-byte token. Got:");
                printf("%s", token);
                return false;
            }
			printf("====================set params:%s\n", token);
            memcpy(tx->params, token, 8);
            break;
        }
        token = strtok(NULL, delims);
    }
    if (ntok >= 3)
    {
        return true;
    }
    else
    {
        printf("Not enough data, got tokens: ");
        printf("%d\n", ntok);
		return false;
    }
}

static uint8_t patable_power_868[8] = {0x03,0x17,0x1D,0x26,0x50,0x86,0xCD,0xC0};

void sendCommand(const char *message) {
	printf("send\n");
	TxPacketData tx;
	parseMessage(message, &tx);

	uint32_t packetSendTime = millis();
	cc1100.sendPacket(&tx);
	printf("wait\n");
	while (!cc1100.packet_available() && millis() - packetSendTime < 250) {
		delay(10);
	}
	printf("endwait\n");
}

void on_message(struct mosquitto *, void *, const struct mosquitto_message *msg) {
    if (!strstr(msg->topic, "vmc/cmd/")) {
        std::cout << "Message reçu sur un topic inconnu: " << msg->topic << std::endl;
        return;
    }

    if (strstr(msg->topic, "bypassmode")) {
        if (strstr((char*)msg->payload, "0")) {
            sendCommand("1,40,00000000");
        }
        else if (strstr((char*)msg->payload, "9")) {
            sendCommand("1,40,00010022");
        }
    }
    else if (strstr(msg->topic, "holidayMode")) {
        if (strstr((char*)msg->payload, "OFF")) {
            sendCommand("0,41,00000000");
        }
        else if (strstr((char*)msg->payload, "ON")) {
            sendCommand("0,41,00010000");
        }
    }
    else if (strstr(msg->topic, "onOffState")) {
        if (strstr((char*)msg->payload, "OFF")) {
            printf("OFF\n");
            sendCommand("0,5c,00010000");
        }
        else if (strstr((char*)msg->payload, "ON")) {
            printf("ON\n");
            sendCommand("0,5c,00000000");
        }
    }
    else if (strstr(msg->topic, "schedule")) {
        if (strstr((char*)msg->payload, "2")) {
            sendCommand("0,3B,00002000");
        }
        else if (strstr((char*)msg->payload, "3")) {
            sendCommand("0,3B,00003000");
        }
        else if (strstr((char*)msg->payload, "4")) {
            sendCommand("0,3B,00004000");
        }
    }
    else if (strstr(msg->topic, "lowFanSpeed")) {
        int speed = atoi((char*)msg->payload);
        if (speed >= 90 && speed <= 325) {
            char command[20];
            sprintf(command, "0,3C,%08X", speed << 16);
            sendCommand(command);
        }
    }
    else if (strstr(msg->topic, "highFanSpeed")) {
        int speed = atoi((char*)msg->payload);
        if (speed >= 90 && speed <= 325) {
            char command[20];
            sprintf(command, "0,3D,%08X", speed << 16);
            sendCommand(command);
        }
    }
    else if (strstr(msg->topic, "boost")) {
        sendCommand("1,94,00000000");
    }
    else if (strstr(msg->topic, "temperatureSondeAirVicie") ||
             strstr(msg->topic, "temperatureSondeEntreeAirNeuf")) {
        // Lecture
        sendCommand("0,31,00000000");
    }
    else if (strstr(msg->topic, "temperatureSondeEntreeAirExterieure")) {
        // Lecture
        sendCommand("0,32,00000000");
    }
    else {
        std::cout << "Commande inconnue sur le topic: " << msg->topic << std::endl;
        return;
    }
    
    
    std::cout << "Message reçu sur " << msg->topic << ": "
              << (char*) msg->payload << std::endl;

    sendCommand("0,33,00000000");
    sendCommand("0,31,00000000");
    sendCommand("0,32,00000000");
}


void startMosquitto() {
	 mosquitto_lib_init();

    // Créer un client MQTT
    mosq = mosquitto_new("client-pub", true, nullptr);
    if(!mosq) {
        std::cerr << "Erreur création client mosquitto\n";
        return NULL;
    }

	 mosquitto_message_callback_set(mosq, on_message);

    // Connexion au broker local
    if(mosquitto_connect(mosq, "localhost", 1883, 60) != MOSQ_ERR_SUCCESS) {
        std::cerr << "Impossible de se connecter au broker.\n";
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        return NULL;
    }

    const char *message = "Ideo broker starting";
    mosquitto_publish(mosq, nullptr, "vmc/state/global", strlen(message), message, 0, false);

	mosquitto_subscribe(mosq, nullptr, "vmc/cmd/#", 0);

    mosquitto_loop_start(mosq); 

}

void stopMosquitto(mosquitto *mosq) {
	// Arrêter la boucle de traitement des messages
	mosquitto_loop_stop(mosq, true);

    // Déconnexion et nettoyage
    mosquitto_disconnect(mosq);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();

}




void go2() {
	static char serial_buffer[32];
	wiringPiSetup();			//setup wiringPi library


	cc1100.begin(My_addr);			//setup cc1000 RF IC

	cc1100.set_ISM(3);
//	uint8_t patable_arr[8] = {0xC0, 0, 0, 0, 0, 0, 0, 0};
	cc1100.set_patable(patable_power_868);
	cc1100.set_output_power_level(10);

	cc1100.spi_write_strobe(StrobeCommand::SRES);
	while (cc1100.getState() != ControlState::IDLE);
	cc1100.writeConfiguration(&cc1100_ideo);
	// Communication channel (as set on the devices) is the low sync byte
	cc1100.spi_write_register(Register::SYNC0, 0);
	
	pinMode(IOBL_INT_PIN, INPUT);
	wiringPiISR(IOBL_INT_PIN, INT_EDGE_RISING, &rfCallback);


	cc1100.show_register_settings();
	cc1100.show_main_settings();             //shows setting debug messages to UART
	
	// Put radio in receive mode
	cc1100.goReceive();

	pktlen = sizeof(RawRxPacket);
	int count = 0;

	

	char ch;
	initKeyboard();

	startMosquitto();

	printf("goloop\n");
	printf("\n");
					  
	//------------------------- Main Loop ------------------------
	for (;;) {
		delay(10);                            //delay to reduce system load

		int c = kbhit();
        if (c=='r')
        {
			// Outlet? Temperature
			sendCommand("0,32,00000000");
		}
		else if (c=='t')
        {
			// Outside Temperature
			sendCommand("0,31,00000000");
		}
		else if (c=='f')
        { 
			// Boost
			sendCommand("1,94,00000000");
		}
		else if (c=='s')
        {
			// Status
			sendCommand("0,33,00000000");
		}
		else if (c=='2')
        {
			// Schedule
			sendCommand("0,3B,00002000");
		}
		else if (c=='3')
        {
			// Schedule
			sendCommand("0,3B,00003000");
		}
		else if (c=='4')
        {
			// Schedule
			sendCommand("0,3B,00004000");
		}
		else if (c=='l')
        {
			// Low Fan Speed : 5A ==> 90
			sendCommand("0,3C,006E0000");
		}
		else if (c=='L')
        {
			// High Fan Speed : 110 ==> 272
			sendCommand("0,3C,01090000");
		}
		else if (c=='h')
        {
			// High Fan Speed : 70 ==> 112
			sendCommand("0,3D,00700000");
		}
		else if (c=='H')
        {
			// High Fan Speed : 120 ==> 288
			sendCommand("0,3D,01200000");
		}
		else if (c=='v')
        {
			// Holiday Mode On
			sendCommand("0,41,00000000");
		}
		else if (c=='V')
        {
			// Holiday Mode Off
			sendCommand("0,41,00010000");
		}
		else if (c=='b')
        {
			// Bypass
			sendCommand("1,40,00000000");
		}
		else if (c=='B')
        {
			// Bypass
			sendCommand("1,40,00010022");
		}
		else if (c=='d') {
			// Date/Time
			sendCommand("0,3A,00131402");

		}
		else if (c=='o') {
			// Off
			sendCommand("0,5c,00010000");

		}
		else if (c=='O') {
			// On
			sendCommand("0,5c,00000000");

		}
		else if (c==' ') {
			// Test
			sendCommand("0,80,00000000");

		}
		else if (c) {
			printf("==========================\n");
		}

		count ++;
	}


	stopMosquitto(mosq);

		
}	




int main() 
{

	go2();
	return 0;
}

/*
				0x21		?
Command: 49		0x31		Get Outlet? Temperature:
Command: 50		0x32		Get Outside Temperature
Command: 51		0x33		Get Status
Command: 58		0x3A		Set Date time
Command: 59		0x3B		Set Schedule
Command: 60		0x3C		Set Low Fan Speed
Command: 61		0x3D		Set High Fan Speed
Command: 62	    0x3E		Set Schedule Manual Period
Command: 63	    0x3E		Set Schedule Manual Period
Command: 64     0x3F        Set Schedule Manual Period   


Command: 64		0x40		Set Force Bypass
Command: 65		0x41		Set Holiday Mode
Command: 66		0x42		Set Bypass
Command: 88		0x58		Set Contact Polarité
Command: 89		0x59		Dirty filter alarm threshold
Command: 90		0x5A		?
Command: 91		0x5B		?
Command: 92		0x5C		?
Command: 93		0x5D		?
Command: 94		0x5D		Boost
Command: 128	0x80		?

fanMode: 0/1/2 ?
class AirflowState(Enum):
    Low = 1
    High = 0
    Away = 2

class BypassState(Enum):
    Off = 0
    On = 8
    Forced = 9

class Schedule(Enum):
    Auto1 = 2
    Auto2 = 3
    Manual = 4


Mode Dégivrage: ?



	

	 Température extérieure en degré Celsius (sur la prise d’air neuf)	: TOut	: 17,0°		: Get Outside Temperature / p1
	 Température intérieure en degré Celsius (sur la télécommande)     	: TIn	: 26,5°		: ??
	 sondes de l’extraction air vicié  									: TIn 	: 21,5°		: Get Outlet? Temperature / p1
 	 entrée air neuf 													: Tout 	: 19°		: Get Outlet? Temperature / p2

 Get Outside Temperature: 17.000000 19.000000
 Get Outlet? Temperature: 21.500000 19.000000

 

61635

 170 ?
*/


