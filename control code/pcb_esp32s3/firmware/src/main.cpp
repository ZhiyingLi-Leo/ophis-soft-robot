#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <driver/rmt.h>
#include <esp_task_wdt.h>
#include <atomic>
#include "controller.h"
#include "pulse_pattern.h"

class BoardHardware : public Hardware {
 public:
  bool initialized=false;unsigned configured=0;
  std::array<int,4> running{};
  bool external() const override {return digitalRead(board::EXT_SELECTED)==HIGH;}
  unsigned faults() const override {unsigned bits=0;for(unsigned i=0;i<4;i++)if(digitalRead(board::FAULT_N[i])==LOW)bits|=1U<<i;return bits;}
  bool begin() {
    digitalWrite(board::WAKE,LOW);pinMode(board::WAKE,OUTPUT);
    pinMode(board::EXT_SELECTED,INPUT);pinMode(board::USB_PRESENT_N,INPUT);
    for(unsigned i=0;i<4;i++) {
      digitalWrite(board::ENABLE[i],LOW);pinMode(board::ENABLE[i],OUTPUT);
      digitalWrite(board::DIR[i],board::FORWARD_HIGH[i]);pinMode(board::DIR[i],OUTPUT);
      digitalWrite(board::STEP[i],LOW);pinMode(board::STEP[i],OUTPUT);
      pinMode(board::FAULT_N[i],INPUT);
      if(!ledcSetup(i,20000,10))return false;
      ledcAttachPin(board::CURRENT_PWM[i],i);current(i,board::DEFAULT_CURRENT_MA);
      rmt_config_t cfg=RMT_DEFAULT_CONFIG_TX(gpio_num_t(board::STEP[i]),rmt_channel_t(i));
      cfg.clk_div=40;cfg.mem_block_num=1;cfg.tx_config.loop_en=true;cfg.tx_config.loop_count=1023;
      cfg.tx_config.carrier_en=false;cfg.tx_config.idle_output_en=true;cfg.tx_config.idle_level=RMT_IDLE_LEVEL_LOW;
      if(rmt_config(&cfg)!=ESP_OK || rmt_driver_install(rmt_channel_t(i),0,0)!=ESP_OK)return false;
      configured=i+1;
      if(rmt_enable_tx_loop_autostop(rmt_channel_t(i),false)!=ESP_OK)return false;
    }
    initialized=true;return true;
  }
  void current(unsigned i,unsigned ma) override {ledcWrite(i,(ma*1023U+board::FULL_SCALE_MA/2)/board::FULL_SCALE_MA);}
  bool enable() override {
    if(!initialized||!external())return false;
    digitalWrite(board::WAKE,HIGH);delay(10); // nSLEEP wake max1.2ms + RC settling.
    if(!external()||faults()){digitalWrite(board::WAKE,LOW);return false;}
    for(unsigned i=0;i<4;i++)digitalWrite(board::ENABLE[i],HIGH);
    delayMicroseconds(10);return true;
  }
  void disable() override {
    for(unsigned i=0;i<4;i++)digitalWrite(board::ENABLE[i],LOW);
    digitalWrite(board::WAKE,LOW);
    for(unsigned i=0;i<configured;i++)rmt_tx_stop(rmt_channel_t(i));
    running.fill(0);delayMicroseconds(150);
  }
  bool rates(const std::array<int,4>& target) override {
    if(!initialized)return target==std::array<int,4>{};
    bool changed=false;
    for(unsigned i=0;i<4;i++)if(target[i]!=running[i]) {
      changed=true;
      if(rmt_tx_stop(rmt_channel_t(i))!=ESP_OK)return false;
      // Idle output is explicitly LOW, including continuous-TX erratum workaround.
      rmt_set_idle_level(rmt_channel_t(i),true,RMT_IDLE_LEVEL_LOW);
    }
    if(!changed)return true;
    delayMicroseconds(10); // DIR hold after the last STEP edge.
    for(unsigned i=0;i<4;i++)if(target[i]!=running[i] && target[i])
      digitalWrite(board::DIR[i],target[i]>0 ? board::FORWARD_HIGH[i] : !board::FORWARD_HIGH[i]);
    delayMicroseconds(10); // DIR setup before next rising STEP edge.
    for(unsigned i=0;i<4;i++)if(target[i]!=running[i] && target[i]) {
      PulseHalf halves[64];const unsigned n=pulsePattern(unsigned(abs(target[i])),halves);
      rmt_item32_t items[33]{}; // Zero-duration terminator after the complete period.
      for(unsigned j=0;j<n;j+=2){items[j/2].duration0=halves[j].ticks;items[j/2].level0=halves[j].high;items[j/2].duration1=halves[j+1].ticks;items[j/2].level1=halves[j+1].high;}
      // Directly replace stopped channel memory. rmt_write_items takes a semaphore
      // that can wait for a previous continuous TX, even when wait_tx_done=false.
      if(rmt_fill_tx_items(rmt_channel_t(i),items,n/2+1,0)!=ESP_OK ||
         rmt_tx_start(rmt_channel_t(i),true)!=ESP_OK)return false;
    }
    running=target;return true;
  }
} hardware;
Controller controller(hardware);
QueueHandle_t rxQueue;
struct Packet {uint8_t size;char bytes[64];};
struct BleOutput {char text[320];};
QueueHandle_t txQueue;
BleOutput pendingTx{};size_t txOffset=0;bool txActive=false;
std::atomic<bool> bleConnected{false},bleLost{false},rxOverflow{false};
BLECharacteristic* tx=nullptr;
class Callbacks : public BLEServerCallbacks {
 void onConnect(BLEServer*) override {bleConnected=true;}
 void onDisconnect(BLEServer*) override {bleConnected=false;bleLost=true;}
};
class RxCallbacks : public BLECharacteristicCallbacks {
 void onWrite(BLECharacteristic* ch) override {
   auto data=ch->getValue();
   if(data.size()>64){rxOverflow=true;return;}
   Packet p{};p.size=data.size();memcpy(p.bytes,data.data(),p.size);
   if(xQueueSend(rxQueue,&p,0)!=pdTRUE)rxOverflow=true;
 }
};
LineReader usbLines,bleLines;
void respond(Source source,const char* line) {
  if(source==Source::USB) {if(Serial){Serial.print(line);Serial.print('\n');}return;}
  if(!bleConnected||!tx)return;
  BleOutput message{};snprintf(message.text,sizeof(message.text),"%s\n",line);
  if(!txQueue || xQueueSend(txQueue,&message,0)!=pdTRUE)rxOverflow=true;
}
void pumpBle() {
  static uint32_t last=0;
  if(!bleConnected || !tx || uint32_t(millis()-last)<30)return;
  if(!txActive){if(!txQueue || xQueueReceive(txQueue,&pendingTx,0)!=pdTRUE)return;txOffset=0;txActive=true;}
  size_t n=std::min(size_t(20),strlen(pendingTx.text)-txOffset);
  tx->setValue(reinterpret_cast<uint8_t*>(pendingTx.text)+txOffset,n);tx->notify();
  txOffset+=n;last=millis();if(txOffset>=strlen(pendingTx.text))txActive=false;
}
void feed(LineReader& reader,Source source,char c) {
  int result=reader.feed(c);if(!result)return;
  char reply[300];
  if(result<0){controller.disconnected(source);respond(source,"ERR LINE");return;}
  controller.command(reader.data,source,millis(),reply,sizeof(reply));respond(source,reply);
}
void setup() {
  Serial.begin(115200);Serial.setTxTimeoutMs(10); // Do not let a stalled USB host block safety checks.
  if(!hardware.begin())controller.off("HARDWARE_INIT",true);
  rxQueue=xQueueCreate(12,sizeof(Packet));
  txQueue=xQueueCreate(8,sizeof(BleOutput));
  if(!rxQueue||!txQueue){controller.off("QUEUE_INIT",true);return;}
  BLEDevice::init(board::BLE_NAME);auto server=BLEDevice::createServer();server->setCallbacks(new Callbacks());
  auto service=server->createService(board::SERVICE_UUID);
  tx=service->createCharacteristic(board::TX_UUID,BLECharacteristic::PROPERTY_NOTIFY);tx->addDescriptor(new BLE2902());
  auto rx=service->createCharacteristic(board::RX_UUID,BLECharacteristic::PROPERTY_WRITE);rx->setCallbacks(new RxCallbacks());
  service->start();auto adv=BLEDevice::getAdvertising();adv->addServiceUUID(board::SERVICE_UUID);adv->setScanResponse(true);adv->start();
  esp_task_wdt_init(3,true);esp_task_wdt_add(nullptr);
}
void loop() {
  static bool wasUsb=false;
  bool usb=digitalRead(board::USB_PRESENT_N)==LOW && bool(Serial);
  if(wasUsb && !usb){controller.disconnected(Source::USB);usbLines=LineReader{};}wasUsb=usb;
  if(bleLost.exchange(false)){controller.disconnected(Source::BLE);bleLines=LineReader{};if(rxQueue)xQueueReset(rxQueue);if(txQueue)xQueueReset(txQueue);txActive=false;BLEDevice::startAdvertising();}
  if(rxOverflow.exchange(false)){controller.disconnected(Source::BLE);bleLines=LineReader{};if(rxQueue)xQueueReset(rxQueue);if(txQueue)xQueueReset(txQueue);txActive=false;}
  controller.tick(millis());
  for(unsigned i=0;i<64 && Serial.available();i++)feed(usbLines,Source::USB,char(Serial.read()));
  Packet p{};
  for(unsigned count=0;count<4 && rxQueue && xQueueReceive(rxQueue,&p,0)==pdTRUE;count++)
    for(unsigned i=0;i<p.size;i++)feed(bleLines,Source::BLE,p.bytes[i]);
  pumpBle();esp_task_wdt_reset();delay(1);
}
