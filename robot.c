#include <Arduino.h>
#include <PID_v1_bc.h>

/* =========================
   MOTOR LEFT/ENCODER
========================= */
#define L_IN1 16
#define L_IN2 17
#define L_ENC_A 4
#define L_ENC_B 5

/* =========================
   MOTOR RIGHT/ENCODER
========================= */
#define R_IN1 18
#define R_IN2 8
#define R_ENC_A 6
#define R_ENC_B 7

/* =========================
   ULTRASONIC SENSORS
========================= */
const int trigPin_mid = 19;
const int echoPin_mid = 20;
const int trigPin_right = 41;
const int echoPin_right = 42;
const int trigPin_left = 40;
const int echoPin_left = 39;
const int trigPin_left45 = 13;
const int echoPin_left45 = 14;
const int trigPin_right45 = 21;
const int echoPin_right45 = 38;

/* =========================
   PWM CONFIG
========================= */
#define PWM_FREQ 1000
#define PWM_RES  8

/* =========================
   ENCODER COUNTER
========================= */
volatile long encoderLeft = 0;
volatile long encoderRight = 0;

/* =========================
   MOVEMENT & WALL CALIBRATION
========================= */
#define PULSES_PER_90_TURN 90    
#define TURN_PWM 220             

#define PULSES_PER_CELL 300 // 30cm = 300 xung
#define START_PULSES 300    // chạy thẳng 1 ô lúc bắt đầu để vào mê cung
#define HALF_CELL_PULSES 150
#define BASE_PWM 135      // giảm tốc nền để có thời gian né tường
#define MIN_PWM 60       // cho phép bánh trong cua chậm hơn
#define MAX_PWM 255      // bánh ngoài cua có thể mạnh hơn

#define MAZE_CELL_WIDTH_CM 30.0
#define WALL_DISTANCE_IDEAL 11.5  // tăng khoảng cách mục tiêu để xe chạy xa tường hơn
#define FRONT_DISTANCE_IDEAL 12 

#define SIDE_WALL_THRESHOLD 28  // nhận tường sớm hơn
#define FRONT_WALL_THRESHOLD 20 
#define CORNER_AVOID_DISTANCE 16 // Khoảng cách ép né mép tường
#define SENSOR_CENTER_OFFSET_CM 0.0  // +: xe bám xa tường phải hơn, -: xe bám xa tường trái hơn

// Né tường mạnh khi xe đã quá sát tường
#define DANGER_SIDE_DISTANCE_CM 8.0
#define VERY_DANGER_SIDE_DISTANCE_CM 6.0
#define WALL_CORRECTION_GAIN 1.45
#define DANGER_BOOST_PWM 44
#define VERY_DANGER_BOOST_PWM 72
#define MAX_TOTAL_CORRECTION 110

/* =========================
   PID VARIABLES
========================= */
double setpointXung = 0;   
double inputXung = 0;      
double outputPWM = 0;     

double Kp = 1.5;
double Ki = 0.0;
double Kd = 0.1;
PID myPID(&inputXung, &outputPWM, &setpointXung, Kp, Ki, Kd, DIRECT);

double setpointWall = WALL_DISTANCE_IDEAL;
double inputWall = 0;
double outputWallPWM = 0;
// Giảm Kp để tránh lắc lư (oscillate), tăng Kd để phản ứng dập tắt dao động
double Kp_wall = 12.0;  // tăng lực kéo xe ra xa tường
double Ki_wall = 0.0;
double Kd_wall = 16.0;  // tăng D để phản ứng mạnh khi xe đang lao sát tường 
PID wallPID(&inputWall, &outputWallPWM, &setpointWall, Kp_wall, Ki_wall, Kd_wall, DIRECT);

/* =========================
   FILTER CHO CẢM BIẾN (Chống nhiễu vọt biến)
========================= */
long lastDistRight = WALL_DISTANCE_IDEAL;
long lastDistLeft = WALL_DISTANCE_IDEAL;
long lastDistMid = 999;
long lastDistLeft45 = 999;
long lastDistRight45 = 999;

double setpointTurn = 0;
double inputTurn = 0;
double outputTurnPWM = 0;
double Kp_turn = 2.0;
double Ki_turn = 0.05;
double Kd_turn = 0.1;
PID turnPID(&inputTurn, &outputTurnPWM, &setpointTurn, Kp_turn, Ki_turn, Kd_turn, DIRECT);

/* =========================
   ENCODER ISR
========================= */
void IRAM_ATTR leftISR() {
    if (digitalRead(L_ENC_B)) encoderLeft++;
    else encoderLeft--;
}

void IRAM_ATTR rightISR() {
    if (digitalRead(R_ENC_B)) encoderRight++;
    else encoderRight--;
}

/* =========================
   MOTOR CONTROL
========================= */
void setMotorLeft(int pwm) {
    pwm = constrain(pwm, -255, 255);
    if (pwm > 0) {
        ledcWrite(L_IN1, pwm);
        ledcWrite(L_IN2, 0);
    } else if (pwm < 0) {
        ledcWrite(L_IN1, 0);
        ledcWrite(L_IN2, -pwm);
    } else {
        ledcWrite(L_IN1, 0);
        ledcWrite(L_IN2, 0);
    }
}

void setMotorRight(int pwm) {
    pwm = constrain(pwm, -255, 255);
    if (pwm > 0) {
        ledcWrite(R_IN1, pwm);
        ledcWrite(R_IN2, 0);
    } else if (pwm < 0) {
        ledcWrite(R_IN1, 0);
        ledcWrite(R_IN2, -pwm);
    } else {
        ledcWrite(R_IN1, 0);
        ledcWrite(R_IN2, 0);
    }
}

/* =========================
   ULTRASONIC DISTANCE READING
========================= */
long getDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    // Timeout 6000us ~ 100cm.
    long duration = pulseIn(echoPin, HIGH, 6000); 
    if (duration == 0) return 999;
    return duration * 0.034 / 2;
}

// Bộ lọc Low-Pass Filter cơ bản (Chống giật do nhiễu cảm biến)
long filterDistance(long newValue, long &lastValue) {
    // Nếu không thấy vật thì giữ giá trị 999 để biết là đang mất tường
    if (newValue == 999) {
        lastValue = 999;
        return 999;
    }

    // Nếu lần trước mất tường, lấy ngay giá trị mới để tránh kéo sai về 999
    if (lastValue == 999) {
        lastValue = newValue;
        return newValue;
    }

    // Lọc bằng cách lấy 60% giá trị mới + 40% giá trị cũ
    long filtered = (newValue * 6 + lastValue * 4) / 10;
    lastValue = filtered;
    return filtered;
}

void printSensors(long left, long mid, long right, long left45, long right45) {
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 200) {
        lastPrint = millis();
        Serial.print("L45:"); Serial.print(left45);
        Serial.print(" | L90:"); Serial.print(left);
        Serial.print(" | M:"); Serial.print(mid);
        Serial.print(" | R90:"); Serial.print(right);
        Serial.print(" | R45:"); Serial.println(right45);
    }
}

/* =========================
   TURN RIGHT 90 DEGREES (PID)
========================= */
void turnRight90() {
    Serial.println("\n>>> TURN RIGHT 90°");
    setMotorLeft(0);
    setMotorRight(0);
    delay(100); 

    encoderLeft = 0;
    encoderRight = 0;
    long startEncoderLeft = 0;
    long startEncoderRight = 0;
    unsigned long lastPID = 0;
    
    turnPID.SetMode(MANUAL);
    turnPID.SetMode(AUTOMATIC);
    
    while (true) {
        long currentLeft = encoderLeft - startEncoderLeft;
        long currentRight = encoderRight - startEncoderRight;
        
        if (abs(currentLeft) >= PULSES_PER_90_TURN && abs(currentRight) >= PULSES_PER_90_TURN) {
            break;
        }

        if (millis() - lastPID >= 20) { 
            lastPID = millis();
            
            setpointTurn = abs(currentLeft);
            inputTurn = abs(currentRight);
            turnPID.Compute(); 
            
            int pwmRight = TURN_PWM + outputTurnPWM;
            
            if (abs(currentLeft) >= PULSES_PER_90_TURN) setMotorLeft(0);
            else setMotorLeft(TURN_PWM);
            
            if (abs(currentRight) >= PULSES_PER_90_TURN) setMotorRight(0);
            else setMotorRight(-pwmRight); 
        }
        delay(1); 
    }
    setMotorLeft(0);
    setMotorRight(0);
    delay(100);
}

/* =========================
   TURN LEFT 90 DEGREES (PID)
========================= */
void turnLeft90() {
    Serial.println("\n>>> TURN LEFT 90°");
    setMotorLeft(0);
    setMotorRight(0);
    delay(100); 

    encoderLeft = 0;
    encoderRight = 0;
    long startEncoderLeft = 0;
    long startEncoderRight = 0;
    unsigned long lastPID = 0;

    turnPID.SetMode(MANUAL);
    turnPID.SetMode(AUTOMATIC);
    
    while (true) {
        long currentLeft = encoderLeft - startEncoderLeft;
        long currentRight = encoderRight - startEncoderRight;
        
        if (abs(currentLeft) >= PULSES_PER_90_TURN && abs(currentRight) >= PULSES_PER_90_TURN) {
            break;
        }

        if (millis() - lastPID >= 20) { 
            lastPID = millis();
            
            setpointTurn = abs(currentRight);
            inputTurn = abs(currentLeft);
            turnPID.Compute(); 
            
            int pwmLeft = TURN_PWM + outputTurnPWM;
            
            if (abs(currentRight) >= PULSES_PER_90_TURN) setMotorRight(0);
            else setMotorRight(TURN_PWM);
            
            if (abs(currentLeft) >= PULSES_PER_90_TURN) setMotorLeft(0);
            else setMotorLeft(-pwmLeft); 
        }
        delay(1); 
    }
    setMotorLeft(0);
    setMotorRight(0);
    delay(100);
}

/* =========================
   READ FILTERED SENSORS
========================= */
void readFilteredSensors(long &distLeft, long &distMid, long &distRight, long &distLeft45, long &distRight45) {
    long rawMid = getDistance(trigPin_mid, echoPin_mid);
    long rawRight = getDistance(trigPin_right, echoPin_right);
    long rawLeft = getDistance(trigPin_left, echoPin_left);
    long rawLeft45 = getDistance(trigPin_left45, echoPin_left45);
    long rawRight45 = getDistance(trigPin_right45, echoPin_right45);

    distMid = filterDistance(rawMid, lastDistMid);
    distRight = filterDistance(rawRight, lastDistRight);
    distLeft = filterDistance(rawLeft, lastDistLeft);
    distLeft45 = filterDistance(rawLeft45, lastDistLeft45);
    distRight45 = filterDistance(rawRight45, lastDistRight45);

    if (distMid <= 0) distMid = 999;
    if (distRight <= 0) distRight = 999;
    if (distLeft <= 0) distLeft = 999;
    if (distLeft45 <= 0) distLeft45 = 999;
    if (distRight45 <= 0) distRight45 = 999;
}

/* =========================
   WALL CENTERING ERROR
   wallError > 0: cần bẻ trái
   wallError < 0: cần bẻ phải
========================= */
bool computeWallCenterError(long distLeft, long distRight, long distLeft45, long distRight45, double &wallError) {
    bool hasLeftWall = (distLeft > 0 && distLeft <= SIDE_WALL_THRESHOLD);
    bool hasRightWall = (distRight > 0 && distRight <= SIDE_WALL_THRESHOLD);
    bool dangerLeft45 = (distLeft45 > 0 && distLeft45 <= CORNER_AVOID_DISTANCE);
    bool dangerRight45 = (distRight45 > 0 && distRight45 <= CORNER_AVOID_DISTANCE);

    // 1. NÉ MÉP TƯỜNG SỚM TẠI NGÃ TƯ (EARLY CORNER AVOIDANCE)
    // Cảm biến ngang bị mù, nhưng 45 độ phát hiện mép tường/cột lao tới
    if (!hasLeftWall && dangerLeft45) {
        wallError = -20.0; // Phát sinh lỗi ảo lớn để bẻ gắt sang phải
        return true;
    }
    if (!hasRightWall && dangerRight45) {
        wallError = 20.0; // Phát sinh lỗi ảo lớn để bẻ gắt sang trái
        return true;
    }

    // 2. KẾT HỢP DẬP DAO ĐỘNG KHI XE NGHIÊNG XÉO
    if (hasLeftWall && hasRightWall) {
        double error90 = (double)distLeft - (double)distRight;
        double error45 = 0;
        
        // Cả 2 cảm biến 45 độ đều thấy khoảng cách ý nghĩa
        if (distLeft45 > 0 && distLeft45 <= 35 && distRight45 > 0 && distRight45 <= 35) {
            error45 = (double)distLeft45 - (double)distRight45;
        }

        // Tỉ lệ: 70% lệch thân xe (90 độ) + 30% chĩa đầu xe (45 độ)
        wallError = (error90 * 0.7) + (error45 * 0.3) + SENSOR_CENTER_OFFSET_CM;
        return true;
    }

    if (hasRightWall) {
        wallError = WALL_DISTANCE_IDEAL - (double)distRight;
        return true;
    }

    if (hasLeftWall) {
        wallError = (double)distLeft - WALL_DISTANCE_IDEAL;
        return true;
    }

    wallError = 0;
    return false;
}

// Tăng lực né khi xe đã quá sát tường.
// correction > 0: bẻ trái, correction < 0: bẻ phải.
double emergencyWallBoost(long distLeft, long distRight) {
    double boost = 0;

    if (distRight > 0 && distRight <= VERY_DANGER_SIDE_DISTANCE_CM) {
        boost += VERY_DANGER_BOOST_PWM;
    } else if (distRight > 0 && distRight <= DANGER_SIDE_DISTANCE_CM) {
        boost += DANGER_BOOST_PWM;
    }

    if (distLeft > 0 && distLeft <= VERY_DANGER_SIDE_DISTANCE_CM) {
        boost -= VERY_DANGER_BOOST_PWM;
    } else if (distLeft > 0 && distLeft <= DANGER_SIDE_DISTANCE_CM) {
        boost -= DANGER_BOOST_PWM;
    }

    return boost;
}

void applyForwardPWM(double encoderCorrection, double wallCorrection, long distLeft, long distRight) {
    // Encoder chỉ giữ 2 bánh đều nhau, không được thắng lực né tường.
    double correction = 0.35 * encoderCorrection
                      + WALL_CORRECTION_GAIN * wallCorrection
                      + emergencyWallBoost(distLeft, distRight);

    correction = constrain(correction, -MAX_TOTAL_CORRECTION, MAX_TOTAL_CORRECTION);

    int leftPWM = constrain((int)(BASE_PWM - correction), MIN_PWM, MAX_PWM);
    int rightPWM = constrain((int)(BASE_PWM + correction), MIN_PWM, MAX_PWM);

    setMotorLeft(leftPWM);
    setMotorRight(rightPWM);
}

/* =========================
   MOVE FORWARD PULSES
   - PID encoder giữ 2 bánh đi đều
   - PID wall canh giữa bằng 3 cảm biến
   - stopWhenFrontWall = false dùng lúc mới khởi động để chạy đủ 300 xung vào mê cung
========================= */
void moveForwardPulses(long targetPulses, bool stopWhenFrontWall = true) {
    noInterrupts();
    encoderLeft = 0;
    encoderRight = 0;
    interrupts();

    myPID.SetMode(MANUAL);
    wallPID.SetMode(MANUAL);
    myPID.SetMode(AUTOMATIC);
    wallPID.SetMode(AUTOMATIC);

    unsigned long lastPID = 0;

    while (true) {
        long leftPulse = abs(encoderLeft);
        long rightPulse = abs(encoderRight);
        long avgPulse = (leftPulse + rightPulse) / 2;

        if (avgPulse >= targetPulses) break;

        long distLeft, distMid, distRight, distLeft45, distRight45;
        readFilteredSensors(distLeft, distMid, distRight, distLeft45, distRight45);
        printSensors(distLeft, distMid, distRight, distLeft45, distRight45);

        if (stopWhenFrontWall && distMid <= FRONT_DISTANCE_IDEAL) {
            break;
        }

        if (millis() - lastPID >= 20) {
            lastPID = millis();

            /* PID 1: đồng bộ encoder để đi thẳng */
            setpointXung = leftPulse;
            inputXung = rightPulse;
            myPID.Compute();
            double encoderCorrection = outputPWM;

            /* PID 2: canh giữa tường bằng cảm biến trái/phải/chéo */
            double wallError = 0;
            double wallCorrection = 0;
            if (computeWallCenterError(distLeft, distRight, distLeft45, distRight45, wallError)) {
                setpointWall = 0;
                inputWall = -wallError; // đảo dấu để outputWallPWM dương = bẻ trái
                wallPID.Compute();
                wallCorrection = outputWallPWM;
            }

            applyForwardPWM(encoderCorrection, wallCorrection, distLeft, distRight);
        }
        delay(1);
    }

    setMotorLeft(0);
    setMotorRight(0);
    delay(100);
}

/* =========================
   SETUP
========================= */
void setup() {
    Serial.begin(115200);
    delay(500);

    /* PWM SETUP */
    ledcAttach(L_IN1, PWM_FREQ, PWM_RES);
    ledcAttach(L_IN2, PWM_FREQ, PWM_RES);
    ledcAttach(R_IN1, PWM_FREQ, PWM_RES);
    ledcAttach(R_IN2, PWM_FREQ, PWM_RES);

    /* ENCODER */
    pinMode(L_ENC_A, INPUT_PULLUP);
    pinMode(L_ENC_B, INPUT_PULLUP);
    pinMode(R_ENC_A, INPUT_PULLUP);
    pinMode(R_ENC_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(L_ENC_A), leftISR, RISING);
    attachInterrupt(digitalPinToInterrupt(R_ENC_A), rightISR, RISING);

    /* ULTRASONIC SETUP */
    pinMode(trigPin_mid, OUTPUT);
    pinMode(echoPin_mid, INPUT);
    pinMode(trigPin_right, OUTPUT);
    pinMode(echoPin_right, INPUT);
    pinMode(trigPin_left, OUTPUT);
    pinMode(echoPin_left, INPUT);
    pinMode(trigPin_left45, OUTPUT);
    pinMode(echoPin_left45, INPUT);
    pinMode(trigPin_right45, OUTPUT);
    pinMode(echoPin_right45, INPUT);

    /* PID SETUP */
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(-50, 50);  
    myPID.SetSampleTime(20);         

    turnPID.SetMode(AUTOMATIC);
    turnPID.SetOutputLimits(-75, 75);  
    turnPID.SetSampleTime(20);         

    wallPID.SetMode(AUTOMATIC);
    wallPID.SetOutputLimits(-110, 110); 
    wallPID.SetSampleTime(20);
}

/* =========================
   LOOP (CONTINUOUS MAZE SOLVER)
========================= */
void loop() {
    static bool firstRun = true;

    // Khi bật robot: đi thẳng đúng 300 xung để vào ô đầu tiên của mê cung.
    // Không dừng vì tường trước trong đoạn này, nhưng vẫn dùng encoder + cảm biến để giữ thẳng.
    if (firstRun) {
        firstRun = false;
        Serial.println("\n>>> START: DI THANG 300 XUNG VAO ME CUNG");
        moveForwardPulses(START_PULSES, false);
    }

    long distLeft, distMid, distRight, distLeft45, distRight45;
    readFilteredSensors(distLeft, distMid, distRight, distLeft45, distRight45);

    // In giá trị 5 cảm biến ra Serial Monitor
    printSensors(distLeft, distMid, distRight, distLeft45, distRight45);

    // 1. NGÃ RẼ PHẢI
    if (distRight > SIDE_WALL_THRESHOLD) {
        Serial.println(">>> NGA RE PHAI");
        Serial.println(distRight);
        moveForwardPulses(HALF_CELL_PULSES); // Tiến lên nửa ô để vào tâm ngã tư
        turnRight90();
        moveForwardPulses(HALF_CELL_PULSES); // Thoát ngã tư
        return;
    }

    // 2. CÓ TƯỜNG TRƯỚC MẶT
    if (distMid <= FRONT_DISTANCE_IDEAL) {
        Serial.println(">>> CO TUONG TRUOC MAT");
        Serial.println(distMid);
        if (distLeft > SIDE_WALL_THRESHOLD) {
            turnLeft90();
            moveForwardPulses(HALF_CELL_PULSES); // Thoát ngã tư
        } else {
            turnRight90();
            turnRight90();
            moveForwardPulses(HALF_CELL_PULSES); // Thoát ngã tư
        }
        return;
    }

    // 3. CHẠY THẲNG LIÊN TỤC: dùng cùng logic với moveForwardPulses nhưng không reset encoder.
    static unsigned long lastPID = 0;
    if (millis() - lastPID >= 20) {
        lastPID = millis();

        long leftPulse = abs(encoderLeft);
        long rightPulse = abs(encoderRight);

        setpointXung = leftPulse;
        inputXung = rightPulse;
        myPID.Compute();
        double encoderCorrection = outputPWM;

        double wallError = 0;
        double wallCorrection = 0;
        if (computeWallCenterError(distLeft, distRight, distLeft45, distRight45, wallError)) {
            setpointWall = 0;
            inputWall = -wallError;
            wallPID.Compute();
            wallCorrection = outputWallPWM;
        }

        applyForwardPWM(encoderCorrection, wallCorrection, distLeft, distRight);
    }
}
