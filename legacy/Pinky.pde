#define DEBUG

#include <debug.h>
#include <scheduling.h>

#include <Usb.h>
#include <AndroidAccessory.h>
#include <AFMotor.h>
#include <MshieldMotor.h>


PinkyMotor motor_l(PINKY_MOTOR_LEFT);
PinkyMotor motor_r(PINKY_MOTOR_RIGHT);


#define MEGABOT_ADK_MANUFACTURER "Google, Inc."
#define MEGABOT_ADK_MODEL "Megabot"
#define MEGABOT_ADK_DESCRIPTION "Super smart robust robot star"
#define MEGABOT_ADK_VERSION "0.1"
#define MEGABOT_ADK_URI "http://google.ch"
#define MEGABOT_ADK_SERIAL "0001"

class MegaBotAdk : public scheduling::Task {
  public:
    MegaBotAdk()
      : initialized_(false),
        adk_(MEGABOT_ADK_MANUFACTURER,
             MEGABOT_ADK_MODEL,
             MEGABOT_ADK_DESCRIPTION,
             MEGABOT_ADK_VERSION,
             MEGABOT_ADK_URI,
             MEGABOT_ADK_SERIAL) { }
    
    void Init() {
      DEBUG_PRINT("before init");
      adk_.powerOn();
      initialized_ = true;
      DEBUG_PRINT("after init");
    }
    
    virtual void Run() {
      if (!initialized_) {
        FATAL_ERROR("MegaBotAdk not initialized");
      }

      DEBUG_THROTTLE_START(1000);
      static int count = 0;
      DEBUG_PRINT2("run", count++);
      DEBUG_THROTTLE_END();
      
      if(!adk_.isConnected()) {
        motor_l.setSpeed(0);
        motor_r.setSpeed(0);
        return;
      }
      
      DEBUG_PRINT("adk connected");
      
      uint8_t msg[2] = { 0x00 };
      uint16_t len = adk_.read(msg, sizeof(msg));

      if(len == 2) {
        DEBUG_PRINT("valid command recieved");
        motor_l.setSpeed(msg[0]);
        motor_r.setSpeed(msg[1]);
      }
    }
    
  private:
    bool initialized_;
    AndroidAccessory adk_;
};


MegaBotAdk megabot_adk;
scheduling::Scheduler scheduler;


void setup() {
  Serial.begin(115200);
  DEBUG_PRINT("Start");
  
  scheduler.AddTask("MegaBotAdk", &megabot_adk);
  megabot_adk.Init();
  DEBUG_PRINT("Start<>");
}

void loop() {
  scheduler.Loop();  
}

