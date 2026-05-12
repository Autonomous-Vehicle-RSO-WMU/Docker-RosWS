#ifndef MY_ROBOT_DESCRIPTION_ARDUINO_COMMS_HPP
#define MY_ROBOT_DESCRIPTION_ARDUINO_COMMS_HPP

// #include <cstring>
#include <sstream>
// #include <cstdlib>
#include <libserial/SerialPort.h>
#include <iostream>
#include <unistd.h>
using namespace std;


LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
}

class ArduinoComms
{

public:

  ArduinoComms() = default;

  void connect(const string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
    // Wait for Arduino to reset after serial open, then flush stale data
    usleep(2000000);  // 2 seconds
    serial_conn_.FlushIOBuffers();
  }

  void disconnect()
  {
    serial_conn_.Close();
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }


  string send_msg(const string &msg_to_send, bool print_output = false)
  {
    serial_conn_.FlushIOBuffers(); // Just in case
    serial_conn_.Write(msg_to_send);

    string response = "";
    try
    {
      // Responses end with \r\n so we will read up to (and including) the \n.
      serial_conn_.ReadLine(response, '\n', timeout_ms_);
    }
    catch (const LibSerial::ReadTimeout&)
    {
        cerr << "The ReadByte() call has timed out." << endl ;
    }

    if (print_output)
    {
      cout << "Sent: " << msg_to_send << " Recv: " << response << endl;
    }

    return response;
  }


  void send_empty_msg()
  {
    string response = send_msg("\r");
  }

  void read_sensor_values(double &enc_l, double &enc_r)//, double*accel, double*gyro, double*mag
  {
    string response = send_msg("e\r");

    string delimiter = ",";
    char *buf=response.data();
    //delimiter.cstr is because stringsep is a C function and expects a char* not a string and so its just there so we can easily change the delimiter if we want to in the future. Also, we have to use a char* for stringsep because it modifies the string by inserting null terminators at the end of each token and advancing the pointer, so we can't use a const char* or a std::string directly.
    auto next_token = [&]() -> string {
      char *t = strsep(&buf, delimiter.c_str());
      return t ? string(t) : string("0");
    };

    string token_1  = next_token();
    string token_2  = next_token();
    // string token_3  = next_token();
    // string token_4  = next_token();
    // string token_5  = next_token();
    // string token_6  = next_token();
    // string token_7  = next_token();
    // string token_8  = next_token();
    // string token_9  = next_token();
    // string token_10 = next_token();
    // string token_11 = next_token();

    enc_l    = atoi(token_1.c_str());
    enc_r    = atoi(token_2.c_str());
    // accel[0] = stof(token_3);
    // accel[1] = stof(token_4);
    // accel[2] = stof(token_5);
    // gyro[0]  = stof(token_6);
    // gyro[1]  = stof(token_7);
    // gyro[2]  = stof(token_8);
    // mag[0]   = stof(token_9);
    // mag[1]   = stof(token_10);
    // mag[2]   = stof(token_11);

    
  }
  void set_motor_values(int val_1, int val_2)
  {
    stringstream ss;
    ss << "m " << val_1 << " " << val_2 << "\r";
    send_msg(ss.str());
  }



private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
};

#endif // MY_ROBOT_DESCRIPTION_ARDUINO_COMMS_HPP