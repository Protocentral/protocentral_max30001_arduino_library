// ______          _        _____            _             _
// | ___ \        | |      /  __ \          | |           | |
// | |_/ / __ ___ | |_ ___ | /  \/ ___ _ __ | |_ _ __ __ _| |
// |  __/ '__/ _ \| __/ _ \| |    / _ \ '_ \| __| '__/ _` | |
// | |  | | | (_) | || (_) | \__/\  __/ | | | |_| | | (_| | |
// \_|  |_|  \___/ \__\___/ \____/\___|_| |_|\__|_|  \__,_|_|

//////////////////////////////////////////////////////////////////////////////////////////
//
//  Demo code for the MAX30001 breakout board
//
//  Copyright (c) 2020 ProtoCentral
//
//  This software is licensed under the MIT License(http://opensource.org/licenses/MIT).
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
//  NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
//  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
//  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//  For information on how to use, visit https://github.com/Protocentral/protocentral-max30001-arduino
//
/////////////////////////////////////////////////////////////////////////////////////////

#ifndef protocentral_max30001_h
#define protocentral_max30001_h

#include <Arduino.h>

#define WREG 0x00
#define RREG 0x01

#define STATUS 0x01
#define EN_INT 0x02
#define EN_INT2 0x03
#define MNGR_INT 0x04
#define MNGR_DYN 0x05
#define SW_RST 0x08
#define SYNCH 0x09
#define FIFO_RST 0x0A
#define INFO 0x0F
#define CNFG_GEN 0x10
#define CNFG_CAL 0x12
#define CNFG_EMUX 0x14
#define CNFG_ECG 0x15

#define CNFG_BIOZ_LC 0x1A

#define CNFG_BMUX 0x17
#define CNFG_BIOZ 0x18

#define CNFG_RTOR1 0x1D
#define CNFG_RTOR2 0x1E

#define ECG_FIFO_BURST 0x20
#define ECG_FIFO 0x21

#define BIOZ_FIFO_BURST 0x22
#define BIOZ_FIFO 0x23

#define RTOR 0x25
#define NO_OP 0x7F

#define CLK_PIN 6
#define RTOR_INTR_MASK 0x04

enum EN_INT_bits
{
  EN_EINT = 0x800000,
  EN_ECG_FIFO_OVF = 0x400000,
  EN_ECG_FAST_REC = 0x200000,
  EN_DCLOFFINT = 0x100000,
  EN_BIOZ_FIFO_INT = 0x80000,
  EN_BIOZ_FIFO_OVF = 0x40000,
  EN_BIOZ_OVER_RANGE = 0x20000,
  EN_BIOZ_UNDER_RANGE = 0x10000,
  EN_BIOZ_CG_MON = 0x8000,

  EN_LONINT = 0x800,
  EN_RRINT = 0x400,
  EN_SAMP = 0x200,
  EN_PLLINT = 0x100,

  EN_BCGMP = 0x20,
  EN_BCGMN = 0x10,
  EN_LDOFF_PH = 0x8,
  EN_LDOFF_PL = 0x4,
  EN_LDOFF_NH = 0x2,
  EN_LDOFF_NL = 0x1
};

typedef union max30001_status_reg
{
  uint32_t all;

  struct
  {
    uint32_t loff_nl : 1;
    uint32_t loff_nh : 1;
    uint32_t loff_pl : 1;
    uint32_t loff_ph : 1;

    uint32_t bcgmn : 1;
    uint32_t bcgmp : 1;
    uint32_t reserved1 : 1;
    uint32_t reserved2 : 1;

    uint32_t pllint : 1;
    uint32_t samp : 1;
    uint32_t rrint : 1;
    uint32_t lonint : 1;

    uint32_t pedge : 1;
    uint32_t povf : 1;
    uint32_t pint : 1;
    uint32_t bcgmon : 1;

    uint32_t bundr : 1;
    uint32_t bover : 1;
    uint32_t bovf : 1;
    uint32_t bint : 1;

    uint32_t dcloffint : 1;
    uint32_t fstint : 1;
    uint32_t eovf : 1;
    uint32_t eint : 1;

    uint32_t reserved : 8;

  } bit;

} max30001_status_t;

/**
 * @brief MNGR_INT (0x04)
 */
typedef union max30001_mngr_int_reg
{
  uint32_t all;

  struct
  {
    uint32_t samp_it : 2;
    uint32_t clr_samp : 1;
    uint32_t clr_pedge : 1;
    uint32_t clr_rrint : 2;
    uint32_t clr_fast : 1;
    uint32_t reserved1 : 1;
    uint32_t reserved2 : 4;
    uint32_t reserved3 : 4;

    uint32_t b_fit : 3;
    uint32_t e_fit : 5;

    uint32_t reserved : 8;

  } bit;

} max30001_mngr_int_t;

typedef enum
{
  SAMPLINGRATE_128 = 128,
  SAMPLINGRATE_256 = 256,
  SAMPLINGRATE_512 = 512
} sampRate;

class MAX30001
{
public:
  MAX30001(int cs_pin);
  unsigned int heartRate;
  unsigned int RRinterval;
  signed long ecg_data;
  signed long bioz_data;

  volatile int ecgSamplesAvailable;
  signed long s32ECGData[128];

  void BeginECGOnly();
  void BeginECGBioZ();
  void BeginRtoRMode();

  signed long getECGSamples(void);
  signed long getBioZSamples(void);
  void getHRandRR(void);

  bool max30001ReadInfo(void);
  void max30001SetsamplingRate(uint16_t samplingRate);

  void max30001SetInterrupts(uint32_t interrupts);
  void max30001ServiceAllInterrupts();

  void readStatus(void);

private:

  void _max30001ReadECGFIFO(int num_bytes);
  void _max30001ReadBIOZFIFO(int num_bytes);
  
  void _max30001Synch(void);
  void _max30001RegWrite(unsigned char WRITE_ADDRESS, unsigned long data);
  void _max30001RegRead(uint8_t Reg_address, uint8_t *buff);
  void _max30001RegRead24(uint8_t Reg_address, uint32_t *read_data);

  void _max30001SwReset(void);
  void _max30001FIFOReset(void);

  max30001_status_t global_status;
  int _cs_pin;
  volatile unsigned char _readBufferECG[128]; // 4*32 samples
  volatile unsigned char _readBufferBIOZ[128]; // 4*32 samples
};

#endif
