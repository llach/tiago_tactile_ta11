#include <unistd.h>
#include "tiago_tactile_ta11/ta11.h"


int configIO(HANDLE hDevice);
int feedback_loop(HANDLE hDevice, u6CalibrationInfo *caliInfo);

namespace tiago_tactile_ta11{

  TA11::TA11() {

    ROS_INFO("Opening U6 over USB");
    if( (_hDevice = openUSBConnection(-1)) == NULL ){
      throw std::runtime_error("LabJack U6 not found!");
    }

    ROS_INFO("Getting calibration information from U6");
    if( getCalibrationInfo(_hDevice, &_caliInfo) < 0 ){
      throw std::runtime_error("Could not acquire calibration info!");
    }

    ROS_INFO("Configuring U6 ...");
    if( configIO() != 0 ){
      throw std::runtime_error("Could not configure U6!");
    }

    _sendBuff[1] = (uint8)(0xF8);  //Command byte
    _sendBuff[2] = 5;             //Number of data words (.5 word for echo + 4.5)
    _sendBuff[3] = (uint8)(0x00);  //Extended command number

    _sendBuff[6] = 0;  //Echo

    _sendBuff[7] = 2;          //IOType is AIN24
    _sendBuff[8] = 0;          //Positive channel
    _sendBuff[9] = 8 + 0 * 16;   //ResolutionIndex(Bits 0-3) = 8, GainIndex(Bits 4-7) = 0 (+-10V)
    _sendBuff[10] = 0 + 1 * 128;  //SettlingFactor(Bits 0-2) = 0 (5 microseconds), Differential(Bit 7) = 1

    _sendBuff[11] = 2;          //IOType is AIN24
    _sendBuff[12] = 2;          //Positive channel
    _sendBuff[13] = 8 + 0 * 16;   //ResolutionIndex(Bits 0-3) = 8, GainIndex(Bits 4-7) = 0 (+-10V)
    _sendBuff[14] = 0 + 1 * 128;  //SettlingFactor(Bits 0-2) = 0 (5 microseconds), Differential(Bit 7) = 1

    _sendBuff[15] = 0;    //Padding byte

    extendedChecksum(_sendBuff, _slen);

    values = std::vector<float>(2);
  }

  TA11::~TA11() {
    configIO();
    closeUSBConnection(_hDevice);
  }

  //Sends a ConfigIO low-level command that configures the U6
  int TA11::configIO()
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
    if( (sendChars = LJUSB_Write(_hDevice, sendBuff, 16)) < 16 )
    {
      if(sendChars == 0)
        throw std::runtime_error("ConfigIO error : write failed\n");
      else
        throw std::runtime_error("ConfigIO error : did not write all of the buffer\n");
    }

    //Reading response from U6
    if( (recChars = LJUSB_Read(_hDevice, recBuff, 16)) < 16 )
    {
      if(recChars == 0)
        throw std::runtime_error("ConfigIO error : read failed\n");
      else
        throw std::runtime_error("ConfigIO error : did not read all of the buffer\n");
    }

    checksumTotal = extendedChecksum16(recBuff, 16);
    if( (uint8)((checksumTotal / 256 ) & 0xff) != recBuff[5] )
    {
      throw std::runtime_error("ConfigIO error : read buffer has bad checksum16(MSB)\n");
    }

    if( (uint8)(checksumTotal & 0xff) != recBuff[4] )
    {
      throw std::runtime_error("ConfigIO error : read buffer has bad checksum16(LBS)\n");
    }

    if( extendedChecksum8(recBuff) != recBuff[0] )
    {
      throw std::runtime_error("ConfigIO error : read buffer has bad checksum8\n");
    }

    if( recBuff[1] != (uint8)(0xF8) || recBuff[2] != (uint8)(0x05) || recBuff[3] != (uint8)(0x0B) )
    {
      throw std::runtime_error("ConfigIO error : read buffer has wrong command bytes\n");
    }

    if( recBuff[6] != 0 )
    {
      ROS_FATAL("ConfigIO error : read buffer received errorcode %d\n", recBuff[6]);
    }

    return 0;
  }

  void TA11::read_loop()
  {
    int sendChars, recChars;
    uint16 checksumTotal;
    double v1, v2;

    ROS_INFO("Reading sensor values ...");

    while( ros::ok() )
    {

      //Sending command to U6
      if((sendChars = LJUSB_Write(_hDevice, _sendBuff, _slen)) < _slen )
      {
        if(sendChars == 0)
          throw std::runtime_error("Feedback loop error : write failed");
        else
          throw std::runtime_error("Feedback loop error : did not write all of the buffer");
      }

      //Reading response from U6
      if((recChars = LJUSB_Read(_hDevice, _recBuff, _rlen)) < _rlen )
      {
        if( recChars == 0 )
        {
          throw std::runtime_error("Feedback loop error : read failed (wrong rlen?)");
        }
        else
          throw std::runtime_error("Feedback loop error : did not read all of the expected buffer");
      }

      if( recChars < _rlen )
      {
        ROS_FATAL("Feedback loop error : response is not large enough. Received %d; Expected %d bytes (_sendBuff[2] wrong?)", recChars, _rlen);
        for (int i=0;i<recChars;i++)
          ROS_FATAL("_recBuff[%d] = %d", i, _recBuff[i]);
        throw std::runtime_error("response not large enough");
      }

      checksumTotal = extendedChecksum16(_recBuff, recChars);
      if((uint8)((checksumTotal / 256 ) & 0xff) != _recBuff[5] )
      {
        throw std::runtime_error("Feedback loop error : read buffer has bad checksum16(MSB)");
      }

      if((uint8)(checksumTotal & 0xff) != _recBuff[4] )
      {
        throw std::runtime_error("Feedback loop error : read buffer has bad checksum16(LBS)");
      }

      if(extendedChecksum8(_recBuff) != _recBuff[0] )
      {
        throw std::runtime_error("Feedback loop error : read buffer has bad checksum8");
      }

      if(_recBuff[1] != (uint8)(0xF8) || _recBuff[3] != (uint8)(0x00) )
      {
        throw std::runtime_error("Feedback loop error : read buffer has wrong command bytes ");
      }

      if(_recBuff[6] != 0 )
      {
        ROS_FATAL("Feedback loop error : received errorcode %d for frame %d ", _recBuff[6], _recBuff[7]);
      }

      getAinVoltCalibrated(&_caliInfo, 8, 0, 1, _recBuff[9] + _recBuff[10] * 256 + _recBuff[11] * 65536, &v1);
      getAinVoltCalibrated(&_caliInfo, 8, 0, 1, _recBuff[12] + _recBuff[13] * 256 + _recBuff[14] * 65536, &v2);
      ROS_DEBUG_NAMED("forceValues", "AIN0 %.6f AIN1 %.6f\n", v1, v2);

      sleep(1); // todo don't
    } // while
  }
}