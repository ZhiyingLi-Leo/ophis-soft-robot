// 推进器步进电机控制固件
// 串口协议（115200, 8N1）：
//   V<signed_steps_per_second>\n  设置速度和方向，例如 V8000、V-4000、V0
//   S\n                           立即停止
//   ?\n                           返回当前速度

// 四个驱动器共用一组脉冲差分端，因此四个电机始终同步同速。
const uint8_t PULSE_MINUS_PIN = 8;  // D8  -> 四个驱动器 P-
const uint8_t PULSE_PLUS_PIN = 9;   // D9  -> 四个驱动器 P+

// 方向负端共用，方向正端按上下两组分开。
const uint8_t DIRECTION_MINUS_PIN = 10;       // D10 -> 四个驱动器 D-
const uint8_t LOWER_DIRECTION_PLUS_PIN = 11; // D11 -> 下面两个驱动器 D+
const uint8_t UPPER_DIRECTION_PLUS_PIN = 12; // D12 -> 上面两个驱动器 D+

// 当前采用“负端固定 LOW，正端输出信号”的方式。
const uint8_t SIGNAL_REFERENCE_LEVEL = LOW;

// 正速定义为“导管前进”。上下滑轮面对面，所以两组电机需反向旋转。
// 如果真机上正速实际表现为回退，只需交换这两个电平。
const uint8_t UPPER_FORWARD_LEVEL = HIGH;
const uint8_t LOWER_FORWARD_LEVEL = LOW;

const long MAX_SPEED_STEPS_PER_SECOND = 16000;
const unsigned long DIRECTION_SETUP_MICROS = 10;

char commandBuffer[32];
uint8_t commandLength = 0;
long commandedSpeed = 0;
unsigned long halfPeriodMicros = 0;
unsigned long lastEdgeMicros = 0;
bool stepLevel = false;
int currentDirection = 1; // 1=导管前进，-1=导管回退
unsigned long directionChangedMicros = 0;

void setTravelDirection(int direction) {
  digitalWrite(DIRECTION_MINUS_PIN, SIGNAL_REFERENCE_LEVEL);
  if (direction > 0) {
    digitalWrite(UPPER_DIRECTION_PLUS_PIN, UPPER_FORWARD_LEVEL);
    digitalWrite(LOWER_DIRECTION_PLUS_PIN, LOWER_FORWARD_LEVEL);
  } else {
    digitalWrite(UPPER_DIRECTION_PLUS_PIN, !UPPER_FORWARD_LEVEL);
    digitalWrite(LOWER_DIRECTION_PLUS_PIN, !LOWER_FORWARD_LEVEL);
  }
  currentDirection = direction;
  directionChangedMicros = micros();
}

void stopMotor() {
  commandedSpeed = 0;
  halfPeriodMicros = 0;
  stepLevel = false;
  // P+=P- 时差分电压为 0，驱动器不计步。
  digitalWrite(PULSE_PLUS_PIN, SIGNAL_REFERENCE_LEVEL);
}

void setSpeed(long speedStepsPerSecond) {
  speedStepsPerSecond = constrain(
      speedStepsPerSecond,
      -MAX_SPEED_STEPS_PER_SECOND,
      MAX_SPEED_STEPS_PER_SECOND);

  if (speedStepsPerSecond == 0) {
    stopMotor();
    return;
  }

  const int requestedDirection = speedStepsPerSecond > 0 ? 1 : -1;
  if (requestedDirection != currentDirection) {
    // 先将脉冲端回到静止电平，再改方向，避免方向切换时误计步。
    stepLevel = false;
    digitalWrite(PULSE_PLUS_PIN, SIGNAL_REFERENCE_LEVEL);
    setTravelDirection(requestedDirection);
  }
  commandedSpeed = speedStepsPerSecond;
  const unsigned long magnitude = labs(speedStepsPerSecond);
  halfPeriodMicros = 500000UL / magnitude;
  // 保证 STEP 高低电平都不短于 2 us；真实下限以驱动器手册为准。
  if (halfPeriodMicros < 2) halfPeriodMicros = 2;
  lastEdgeMicros = micros();
}

void processCommand(const char *command) {
  if (command[0] == 'V') {
    char *endPointer = nullptr;
    const long requestedSpeed = strtol(command + 1, &endPointer, 10);
    if (endPointer == command + 1 || *endPointer != '\0') {
      Serial.println("ERR FORMAT");
      return;
    }
    setSpeed(requestedSpeed);
    Serial.print("OK V");
    Serial.println(commandedSpeed);
  } else if (command[0] == 'S' && command[1] == '\0') {
    stopMotor();
    Serial.println("OK V0");
  } else if (command[0] == '?' && command[1] == '\0') {
    Serial.print("V");
    Serial.println(commandedSpeed);
  } else {
    Serial.println("ERR COMMAND");
  }
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    const char incoming = static_cast<char>(Serial.read());
    if (incoming == '\r') continue;
    if (incoming == '\n') {
      commandBuffer[commandLength] = '\0';
      if (commandLength > 0) processCommand(commandBuffer);
      commandLength = 0;
    } else if (commandLength < sizeof(commandBuffer) - 1) {
      commandBuffer[commandLength++] = incoming;
    } else {
      commandLength = 0;
      Serial.println("ERR TOO_LONG");
    }
  }
}

void updateStepPulse() {
  if (commandedSpeed == 0 || halfPeriodMicros == 0) return;
  const unsigned long now = micros();
  if (static_cast<unsigned long>(now - directionChangedMicros) < DIRECTION_SETUP_MICROS) {
    return;
  }
  if (static_cast<unsigned long>(now - lastEdgeMicros) >= halfPeriodMicros) {
    lastEdgeMicros = now;
    stepLevel = !stepLevel;
    digitalWrite(PULSE_PLUS_PIN, stepLevel ? !SIGNAL_REFERENCE_LEVEL : SIGNAL_REFERENCE_LEVEL);
  }
}

void setup() {
  pinMode(PULSE_MINUS_PIN, OUTPUT);
  pinMode(PULSE_PLUS_PIN, OUTPUT);
  pinMode(DIRECTION_MINUS_PIN, OUTPUT);
  pinMode(LOWER_DIRECTION_PLUS_PIN, OUTPUT);
  pinMode(UPPER_DIRECTION_PLUS_PIN, OUTPUT);

  digitalWrite(PULSE_MINUS_PIN, SIGNAL_REFERENCE_LEVEL);
  digitalWrite(PULSE_PLUS_PIN, SIGNAL_REFERENCE_LEVEL);
  digitalWrite(DIRECTION_MINUS_PIN, SIGNAL_REFERENCE_LEVEL);
  setTravelDirection(1);

  Serial.begin(115200);
  stopMotor();
  Serial.println("READY PROPULSION_V2");
}

void loop() {
  // 两个函数都是非阻塞的，运动时仍能随时接收 V0/S 急停。
  readSerialCommands();
  updateStepPulse();
}
