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

  void BeginECGOnly();
  void BeginECGBioZ();
  void BeginRtoRMode();
  
  signed long getECGSamples(void);
  signed long getBioZSamples(void);
  void getHRandRR(void);

  bool max30001ReadInfo(void);
  void max30001SetsamplingRate(uint16_t samplingRate);
 
private:
  void _max30001ReadData(int num_samples, uint8_t *readBuffer);
  void _max30001Synch(void);
  void _max30001RegWrite(unsigned char WRITE_ADDRESS, unsigned long data);
  void _max30001RegRead(uint8_t Reg_address, uint8_t *buff);
  void _max30001SwReset(void);

  int _cs_pin;
};

#define WREG 0x00
#define RREG 0x01

#define NO_OP 0x00
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

#endif
