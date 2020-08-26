#include <unistd.h>
#include <termios.h>
#include "tiago_tactile_ta11/u6.h"


static struct termios termNew, termOrig;
static int peek = -1;

int configIO(HANDLE hDevice);
int feedback_loop(HANDLE hDevice, u6CalibrationInfo *caliInfo);

void setTerm();
int kbhit();
void unsetTerm();

int main(int argc, char **argv)
{
  HANDLE hDevice;
  u6CalibrationInfo caliInfo;

  //setting terminal settings
  setTerm();

  //Opening first found U6 over USB
  if( (hDevice = openUSBConnection(-1)) == NULL )
    goto done;

  //Getting calibration information from U6
  if( getCalibrationInfo(hDevice, &caliInfo) < 0 )
    goto close;

  if( configIO(hDevice) != 0 )
    goto close;

  if( feedback_loop(hDevice, &caliInfo) != 0 )
    goto close;

  configIO(hDevice);

  close:
    closeUSBConnection(hDevice);
  done:
    printf("\nDone\n");

  //Setting terminal settings to previous settings
  unsetTerm();
  return 0;
}

//Sends a ConfigIO low-level command that configures the FIOs, DAC, Timers and
//Counters for this example
int configIO(HANDLE hDevice)
{
  uint8 sendBuff[16], recBuff[16];
  uint16 checksumTotal;
  int sendChars, recChars, i;

  sendBuff[1] = (uint8)(0xF8);  //Command byte
  sendBuff[2] = (uint8)(0x05);  //Number of data words
  sendBuff[3] = (uint8)(0x0B);  //Extended command number

  sendBuff[6] = 1;  //Writemask : Setting writemask for timerCounterConfig (bit 0)

  sendBuff[7] = 0;      //NumberTimersEnabled
  sendBuff[8] = 0;  //CounterEnable: Bit 0 is Counter 0, Bit 1 is Counter 1
  sendBuff[9] = 1;  //TimerCounterPinOffset:  Setting to 1 so Timer/Counters start on FIO1

  for( i = 10; i < 16; i++ )
    sendBuff[i] = 0;  //Reserved
  extendedChecksum(sendBuff, 16);

  //Sending command to U6
  if( (sendChars = LJUSB_Write(hDevice, sendBuff, 16)) < 16 )
  {
    if(sendChars == 0)
      printf("ConfigIO error : write failed\n");
    else
      printf("ConfigIO error : did not write all of the buffer\n");
    return -1;
  }

  //Reading response from U6
  if( (recChars = LJUSB_Read(hDevice, recBuff, 16)) < 16 )
  {
    if(recChars == 0)
      printf("ConfigIO error : read failed\n");
    else
      printf("ConfigIO error : did not read all of the buffer\n");
    return -1;
  }

  checksumTotal = extendedChecksum16(recBuff, 16);
  if( (uint8)((checksumTotal / 256 ) & 0xff) != recBuff[5] )
  {
    printf("ConfigIO error : read buffer has bad checksum16(MSB)\n");
    return -1;
  }

  if( (uint8)(checksumTotal & 0xff) != recBuff[4] )
  {
    printf("ConfigIO error : read buffer has bad checksum16(LBS)\n");
    return -1;
  }

  if( extendedChecksum8(recBuff) != recBuff[0] )
  {
    printf("ConfigIO error : read buffer has bad checksum8\n");
    return -1;
  }

  if( recBuff[1] != (uint8)(0xF8) || recBuff[2] != (uint8)(0x05) || recBuff[3] != (uint8)(0x0B) )
  {
    printf("ConfigIO error : read buffer has wrong command bytes\n");
    return -1;
  }

  if( recBuff[6] != 0 )
  {
    printf("ConfigIO error : read buffer received errorcode %d\n", recBuff[6]);
    return -1;
  }

  return 0;
}

//Calls a Feedback low-level call to read AIN0
int feedback_loop(HANDLE hDevice, u6CalibrationInfo *caliInfo)
{
  int slen = 16;
  int rlen = 16;

  long count;
  uint8 sendBuff[slen], recBuff[rlen];
  int sendChars, recChars;
  uint16 checksumTotal;
  double v1, v2;

  sendBuff[1] = (uint8)(0xF8);  //Command byte
  sendBuff[2] = 5;             //Number of data words (.5 word for echo + 4.5)
  sendBuff[3] = (uint8)(0x00);  //Extended command number

  sendBuff[6] = 0;  //Echo

  sendBuff[7] = 2;          //IOType is AIN24
  sendBuff[8] = 0;          //Positive channel
  sendBuff[9] = 8 + 0*16;   //ResolutionIndex(Bits 0-3) = 8, GainIndex(Bits 4-7) = 0 (+-10V)
  sendBuff[10] = 0 + 1*128;  //SettlingFactor(Bits 0-2) = 0 (5 microseconds), Differential(Bit 7) = 1

  sendBuff[11] = 2;          //IOType is AIN24
  sendBuff[12] = 2;          //Positive channel
  sendBuff[13] = 8 + 0*16;   //ResolutionIndex(Bits 0-3) = 8, GainIndex(Bits 4-7) = 0 (+-10V)
  sendBuff[14] = 0 + 1*128;  //SettlingFactor(Bits 0-2) = 0 (5 microseconds), Differential(Bit 7) = 1

  sendBuff[15] = 0;    //Padding byte

  extendedChecksum(sendBuff, slen);

  printf("Running Feedback calls in a loop\n");

  count = 0;
  while( !kbhit() )
  {
    count++;

    //Sending command to U6
    if( (sendChars = LJUSB_Write(hDevice, sendBuff, slen)) < slen )
    {
      if(sendChars == 0)
        printf("Feedback loop error : write failed\n");
      else
        printf("Feedback loop error : did not write all of the buffer\n");
      return -1;
    }

    //Reading response from U6
    if( (recChars = LJUSB_Read(hDevice, recBuff, rlen)) < rlen )
    {
      if( recChars == 0 )
      {
        printf("Feedback loop error : read failed (wrong rlen?)\n");
        return -1;
      }
      else
        printf("Feedback loop error : did not read all of the expected buffer\n");
    }

    if( recChars < rlen )
    {
      printf("Feedback loop error : response is not large enough. Received %d; Expected %d bytes (sendBuff[2] wrong?)\n", recChars, rlen);
      for (int i=0;i<recChars;i++)
        printf("recBuff[%d] = %d\n", i, recBuff[i]);
      return -1;
    }

    checksumTotal = extendedChecksum16(recBuff, recChars);
    if( (uint8)((checksumTotal / 256 ) & 0xff) != recBuff[5] )
    {
      printf("Feedback loop error : read buffer has bad checksum16(MSB)\n");
      return -1;
    }

    if( (uint8)(checksumTotal & 0xff) != recBuff[4] )
    {
      printf("Feedback loop error : read buffer has bad checksum16(LBS)\n");
      return -1;
    }

    if( extendedChecksum8(recBuff) != recBuff[0] )
    {
      printf("Feedback loop error : read buffer has bad checksum8\n");
      return -1;
    }

    if( recBuff[1] != (uint8)(0xF8) ||  recBuff[3] != (uint8)(0x00) )
    {
      printf("Feedback loop error : read buffer has wrong command bytes \n");
      return -1;
    }

    if( recBuff[6] != 0 )
    {
      printf("Feedback loop error : received errorcode %d for frame %d ", recBuff[6], recBuff[7]);
      return -1;
    }

    getAinVoltCalibrated(caliInfo, 8, 0, 1, recBuff[9] + recBuff[10]*256 + recBuff[11]*65536, &v1);
    printf("AIN0(Diff) : %.6f volts\n", v1);

    sleep(1);
  }
  return 0;
}


void setTerm()
{
  tcgetattr(0, &termOrig);
  termNew = termOrig;
  termNew.c_lflag &= ~ICANON;
  termNew.c_lflag &= ~ECHO;
  termNew.c_lflag &= ~ISIG;
  termNew.c_cc[VMIN] = 1;
  termNew.c_cc[VTIME] = 0;
  tcsetattr(0, TCSANOW, &termNew);
}

int kbhit()
{
  char ch;
  int nread;

  if( peek != -1 )
    return 1;

  termNew.c_cc[VMIN]=0;
  tcsetattr(0, TCSANOW, &termNew);
  nread = read(0,&ch,1);
  termNew.c_cc[VMIN]=1;
  tcsetattr(0, TCSANOW, &termNew);

  if( nread == 1 )
  {
    peek = ch;
    return 1;
  }

  return 0;
}

void unsetTerm()
{
  tcsetattr(0, TCSANOW, &termOrig);
}
