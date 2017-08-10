#ifndef FLASH_H__
#define FLASH_H__

//#include "FreeRTOS.h"
//#include "timers.h"
//float AVRMSCalib;

//float AIRMSCalib;


 
#define AWATTHR   0x01
#define BWATTHR   0x02
#define CWATTHR   0x03
#define AVARHR    0x04
#define BVARHR    0x05
#define CVARHR    0x06
#define AVAHR     0x07
#define BVAHR     0x08
#define CVAHR     0x09
#define AIRMS     0x0A
#define BIRMS     0x0B
#define CIRMS     0x0C
#define AVRMS     0x0D
#define BVRMS     0x0E
#define CVRMS     0x0F
#define FREQ      0x10
#define TEMP      0x11
#define WFORM     0x12
#define OPMODE    0x13
#define MMODE     0x14
#define WAVMODE   0x15
#define COMPMODE  0x16
#define LCYCMODE  0x17
#define MASK      0x18
#define STATUS    0x19
#define RSTATUS   0x1A
#define ZXTOUT    0x1B
#define LINECYC   0x1C
#define SAGCYC    0x1D
#define SAGLVL    0x1E
#define VPINTLVL  0x1F
#define IPINTLVL  0x20
#define VPEAK     0x21
#define IPEAK     0x22
#define GAIN      0x23
#define AVRMSGAIN 0x24
#define BVRMSGAIN 0x25
#define CVRMSGAIN 0x26
#define AIGAIN    0x27
#define BIGAIN    0x28
#define CIGAIN    0x29
#define AWG       0x2A
#define BWG       0x2B
#define CWG       0x2C
#define AVARG     0x2D
#define BVARG     0x2E
#define CVARG     0x2F
#define AVAG      0x30
#define BVAG      0x31
#define CVAG      0x32
#define AVRMSOS   0x33
#define BVRMSOS   0x34
#define CVRMSOS   0x35
#define AIRMSOS   0x36
#define BIRMSOS   0x37
#define CIRMSOS   0x38
#define AWAITOS   0x39
#define BWAITOS   0x3A
#define CWAITOS   0x3B
#define AVAROS    0x3C
#define BVAROS    0x3D
#define CVAROS    0x3E
#define APHCAL    0x3F
#define BPHCAL    0x40
#define CPHCAL    0x41
#define WDIV      0x42
#define VADIV     0x44
#define VARDIV    0x43
#define APCFNUM   0x45
#define APCFDEN   0x46
#define VARCFNUM  0x47
#define VARCFDEN  0x48
#define CHKSUM    0x7E
#define VERSION   0x7F
 
#define REG_READ(reg) reg
#define REG_WRITE(reg)  (unsigned char)((reg) | 0x80)
 
//PHASE_SEL
#define PHASE_A     0
#define PHASE_B     1
#define PHASE_C     2
 
//WAV_SEL
#define CURRENT     0
#define VOLTAGE     1
#define ACT_PWR     2
#define REACT_PWR   3
#define APP_PWR     4
 
//interrupt mask/status bit
#define AEHF            0
#define REHF            1
#define VAEHF           2
#define SAGA            3
#define SAGB            4
#define SAGC            5
#define ZXTOA           6
#define ZXTOB           7
#define ZXTOC           8
#define ZXA             9
#define ZXB             10
#define ZXC             11
#define LENERGY         12
#define RESET           13
#define PKV             14
#define PKI             15
#define WFSM            16
#define REVPAP          17
#define REVPRP          18
#define SEQERR          19
 
#define WAVMODE_VALUE(phase, wave)   (((wave)<<2)|(phase))
 
#define NUMSAMPLES      20
 
#define ITEST           0.6
#define VNOM            220
#define WHREXPECTED     1.2
#define VAHREXPECTED    1.2

#define PHASEA             1
#define PHASEB             2
#define PHASEC             3

typedef struct
{
	float f_VRMS_DEV1;
    float f_VRMS_DEV2;
    float f_VRMS_DEV3;
    float f_VRMS_DEV4;
    float f_VRMS_DEV5;
    float f_VRMS_DEV6;
    float f_VRMS_DEV7;
    float f_VRMS_DEV8;
    float f_VRMS_DEV9;
    float f_SUM_VRMS;
    float f_AVG_VRMS;

}VRMS_ADE;

typedef struct
{
	float f_IRMS_DEV1;
    float f_IRMS_DEV2;
    float f_IRMS_DEV3;
    float f_IRMS_DEV4;
    float f_IRMS_DEV5;
    float f_IRMS_DEV6;
    float f_IRMS_DEV7;
    float f_IRMS_DEV8;
    float f_IRMS_DEV9;
    float f_SUM_IRMS;

}IRMS_ADE;

typedef struct
{
	float f_PWR_DEV1;
    float f_PWR_DEV2;
    float f_PWR_DEV3;
    float f_PWR_DEV4;
    float f_PWR_DEV5;
    float f_PWR_DEV6;
    float f_PWR_DEV7;
    float f_PWR_DEV8;
    float f_PWR_DEV9;
    float f_SUM_PWR;

}PWR_ADE;

typedef struct
{
    uint8_t flag_chanel123;
    uint8_t flag_chanel456;
    uint8_t flag_chanel789;
}flag_ade_connect;

//typedef enum {
//	ADE_STARTUP = 0,
//	ADE_123_CHECK_CONNECT,
//    ADE_456_CHECK_CONNECT,
//    ADE_789_CHECK_CONNECT,
//	ADE_123_CONNECT,
//    ADE_456_CONNECT,
//    ADE_789_CONNECT,
//	ADE_123_ON_DEVICE_NO_VOLT,
//    ADE_456_ON_DEVICE_NO_VOLT,
//    ADE_789_ON_DEVICE_NO_VOLT,
//	ADE_123_ON_DEVICE_NO_AMP,
//    ADE_456_ON_DEVICE_NO_AMP,
//    ADE_789_ON_DEVICE_NO_AMP,
//	ADE_123_ON_DEVICE_ON,
//    ADE_456_ON_DEVICE_ON,
//    ADE_789_ON_DEVICE_ON,
//	ADE_123_OFF,
//    ADE_456_OFF,
//    ADE_789_OFF,
//	REVERSE,
//}ADE_state;

#define ADE7758_DEV1                    1
#define ADE7758_DEV2                    2
#define ADE7758_DEV3                    3

void ADE7758_init(void);
void ADE7758_on(char ade);
void ADE7758_off(char ade);
int ADE7758_ReadWrite(uint8_t chanel, unsigned char writedat);
void ADE7758Write8Op(uint8_t chanel, unsigned char address, unsigned char data);
void ADE7758Write16Op(uint8_t chanel, unsigned char address, uint16_t data);
void ADE7758Write24Op(uint8_t chanel, unsigned char address, uint32_t data);
uint8_t ADE7758Read8Op(uint8_t chanel, unsigned char address);
uint16_t ADE7758Read16Op(uint8_t chanel, unsigned char address);
uint32_t ADE7758Read24Op(uint8_t chanel, unsigned char address);
uint32_t ADE7758_test(char numSamples);
uint32_t ADE7758getVRMS(char phase);
void ADE7758_setpara(uint8_t chanel);
float ADE7758_readVRMS(uint8_t chanel, uint8_t phase);
float ADE7758_readIRMS(uint8_t chanel, uint8_t phase);
void ADE7758_readLENGRY(void);
uint8_t ADE7758_Check(uint8_t chanel);

uint16_t ADE7758_LineFreq(char phase);
void ADE7758_GetEnery(uint8_t phase, uint16_t *AWattHr, uint16_t *BWattHr, uint16_t *CWattHr);
float ADE7758_GetTime(void);
void ADE7758_GetEnery1(uint8_t phase, uint8_t samplingPeriod, uint16_t *AWattHr, uint16_t *BWattHr, uint16_t *CWattHr);   
void ADE7758_delay(void);

void ADUM5401_on(char adum);
void ADUM5401_off(char adum);

void vADE7758_timer_cb(void);
void ADE7758_timer_init(void);
void ADE7758_data_init(void);
//void ADE7758_test(void);
//uint8_t ADE7758(uint8_t data);
//uint8_t SPI_ADE7758_SendByte(uint8_t byte);
//unsigned char DF_Read_status_Register(void);
#endif
