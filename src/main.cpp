#include "vex.h"
#include <thread>

using namespace vex;
using namespace std::chrono;

brain Brain;
optical SensoreOttico(PORT2); 
optical SensoreOttico2(PORT6);
motor braccio(PORT9,gearSetting::ratio18_1, false);
motor pinza(PORT4, gearSetting::ratio18_1, false);

motor leftA = motor(PORT11, gearSetting::ratio18_1, false);
motor leftB = motor(PORT1, gearSetting::ratio18_1, false);

motor_group leftMotors = motor_group(leftA, leftB);

motor rightA = motor(PORT10, gearSetting::ratio18_1, true);
motor rightB = motor(PORT12, gearSetting::ratio18_1,  true);

motor_group rightMotors = motor_group(rightA, rightB);


inertial Inertial = inertial(PORT7);

smartdrive Smartdrive = smartdrive(leftMotors, rightMotors, Inertial, 100.0, 260.9, 127.3, mm);

double posPinza = 0;
bool giallo = false;
bool verde = false;
char col;
bool osFront = false;
bool osLat = false;
int col1 = 5;
int col2 = 5;

void checkFront(){
    while(true){
        osFront=SensoreOttico2.isNearObject();
        this_thread::sleep_for(milliseconds(50));
    }
}

void inizio(){
    float d = 200; //millimetri
    int r = 55;
    int v=0;
    int g=0;

    float corr = 0.3333;
    float dCorretta = d * corr;
    Smartdrive.driveFor(forward, dCorretta, mm, r, rpm);

    while(Smartdrive.isMoving()){
        if(osFront){
            Smartdrive.stop(hold);
            break;
        }
        this_thread::sleep_for(milliseconds(10));
    }

    SensoreOttico.setLightPower(50, percent);
    SensoreOttico.setLight(ledState::on);

    for(int i = 0; i < 5; i++){
       if (SensoreOttico.isNearObject()) {
        int hue = SensoreOttico.hue(); 
        double brightness = SensoreOttico.brightness(); 
        int lightPower = 50;

        if (brightness < 20) {  
            lightPower = 100;
            SensoreOttico.setLightPower(lightPower, percent);
        } 
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);

        if (hue >= 30 && hue < 90){
            g++;
        }else if (hue >= 90 && hue < 150){
            v++;
        }

        }
        this_thread::sleep_for(milliseconds(200));
    }

    if(v>g){
        Brain.Screen.print("Verde");
        verde = true;
    }else if(g>v){
        giallo = true;
        Brain.Screen.print("giallo");
    }
    SensoreOttico.setLight(ledState::off);
}


triport expander = triport(PORT3);
potV2 potBraccio(expander.A);
potV2 potPinza(expander.B);
double pD = 110.0;

void move(char d, int D){
    float corr = 0.3333;
    float dCorretta = D * corr;
    int r = 35;

    if(d=='f'){
        Smartdrive.driveFor(forward, dCorretta, mm, r, rpm);
    }else if (d == 'b'){
        Smartdrive.driveFor(reverse, dCorretta, mm, r, rpm);
    }
}

void turn(int g){
    // double corr = 0.3333;
    // double angCorrDx = g * corr;
    // double angCorrSx = g * 0.30;
    int r = 25;
    Smartdrive.setTurnVelocity(r, percent);
    
    Smartdrive.turnToHeading(g, degrees);
}

void Pinza(char a, double d){
    const double posizioneAperta = d; // -70 
    const double posizioneChiusa = d; // 50
    const double posizioneAlzata = d; // -70
    const double posizioneAbbassata = d; // 60
    const int velocita = 25;// velocità in %
    switch(a){
        case 'u':
            braccio.spinToPosition(posizioneAlzata, rotationUnits::deg, (velocita -10), velocityUnits::pct);
            braccio.stop(hold);
            break;
        case 'd':
            braccio.spinToPosition(posizioneAbbassata, rotationUnits::deg, velocita, velocityUnits::pct);
            braccio.stop(hold);
            break;
        case 'o':
            pinza.spinToPosition(posizioneAperta, degrees, (velocita + 10), velocityUnits::pct, true);
            pinza.stop(hold);
            break;
        case 'c':
            pinza.spinToPosition(posizioneChiusa, degrees, velocita, velocityUnits::pct, false);
            pinza.stop(hold);
        default:
            break;
    }
}

void prendi(){
    Pinza('o', -60);
    Pinza('d', 120);
    Pinza('c', 42);
    Pinza('u', -45);
}

void lascia(){
    Pinza('d', 120);
    Pinza('o', -70);
    Pinza('u', -45);
    Pinza('c', 45);
}

void leggiFront(){
    SensoreOttico2.setLightPower(50, percent);
SensoreOttico2.setLight(ledState::on);

int r = 0, g = 0, v = 0, b = 0;

for (int i = 0; i < 5; i++) {
    if (SensoreOttico2.isNearObject()) {
        int hue = SensoreOttico2.hue(); 
        double brightness = SensoreOttico2.brightness(); 
        int lightPower = 50;

        if (brightness < 20) {
            lightPower = 100;
            SensoreOttico2.setLightPower(lightPower, percent);
        }

        // Classifica il colore in base all'hue
        if (hue < 30 || hue >= 330) { // rosso
            r++;
        } else if (hue >= 30 && hue < 90) { // giallo
            g++;
        } else if (hue >= 90 && hue < 150) { // verde
            v++;
        } else if (hue >= 180 && hue < 250) { // blu
            b++;
        }

        this_thread::sleep_for(milliseconds(200));
    }
}

Brain.Screen.clearScreen();
Brain.Screen.setCursor(1, 1);

// Determina il colore prevalente
if (r > g && r > v && r > b) {
    Brain.Screen.print("Rosso");
    col = 'r';
} else if (g > r && g > v && g > b) {
    Brain.Screen.print("Giallo");
    col = 'g';
} else if (v > r && v > g && v > b) {
    Brain.Screen.print("Verde");
    col = 'y';
} else if (b > r && b > g && b > v) {
    Brain.Screen.print("Blu");
    col = 'b';
} else {
    Brain.Screen.print("Colore incerto");
}

SensoreOttico2.setLight(ledState::off);

}

int main() {
    braccio.resetPosition();
    pinza.resetPosition();

    //Prima calibrazione sensore inerziale

    Inertial.calibrate();

    while(Inertial.isCalibrating()){
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1,1);
        Brain.Screen.print("Calibrazione...");
        this_thread::sleep_for(milliseconds(30));
    }
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,1);

    //fine calibrazione

    //inizio
    // Smartdrive.setHeading(0, degrees);

    thread check = thread(checkFront);
    thread Move = thread(inizio);

    Move.join();

    //inizio istruzioni

    move('f', 350);
    turn(90); //true heading verso cui deve svoltare
    move('f', 170);
    turn(0);
    move('f', 380);
    turn(90);
    move('f',60);
    prendi();
    move('b', 60);
    turn(270);
    move('f', 70);
    lascia();

    // move('b', 40);
    // turn(0);
    // move('f', 100);
    // turn(90);
    // move('f', 40);

    //fine istruzioni
    

    // SensoreOttico.setLightPower(50, percent);
    // SensoreOttico.setLight(ledState::on);
    
    // while (true) {
    //     if (SensoreOttico.isNearObject()) {
    //         int hue = SensoreOttico.hue(); 
    //         double brightness = SensoreOttico.brightness(); 
    //         int lightPower = 50;

    //         if (brightness < 20) {  
    //             lightPower = 100;
    //             SensoreOttico.setLightPower(lightPower, percent);
    //         } 
    //         Brain.Screen.clearScreen();
    //         Brain.Screen.setCursor(1, 1);

    //         if (hue >= 330 || hue < 30)
    //             Brain.Screen.print("Colore: Rosso");
    //         else if (hue >= 30 && hue < 90)
    //             Brain.Screen.print("Colore: Giallo");
    //         else if (hue >= 90 && hue < 150)
    //             Brain.Screen.print("Colore: Verde");
    //         else if (hue >= 150 && hue < 210)
    //             Brain.Screen.print("Colore: Ciano");
    //         else if (hue >= 210 && hue < 270)
    //             Brain.Screen.print("Colore: Blu");
    //         else if (hue >= 270 && hue < 330)
    //             Brain.Screen.print("Colore: Magenta");
    //         else
    //             Brain.Screen.print("Colore sconosciuto");

    //         Brain.Screen.newLine();
    //         Brain.Screen.print("Luminosita': %.2f", brightness);
    //         Brain.Screen.newLine();
    //         Brain.Screen.print("Luminosita' LED: %d%%", lightPower);
    //     } else {
    //         Brain.Screen.clearScreen();
    //         Brain.Screen.setCursor(1, 1);
    //         Brain.Screen.print("Nessun oggetto rilevato!");
    //     }

    //     this_thread::sleep_for(milliseconds(500));
    // }

    return 0;
}