#include "vex.h"
#include <thread>
#include <cmath>
using namespace vex;
using namespace std::chrono;

// Dichiarazione dei componenti hardware
brain Brain;
optical SensoreOttico(PORT2); 
optical SensoreOttico2(PORT6);
motor braccio(PORT9, gearSetting::ratio18_1, false);
motor pinza(PORT4, gearSetting::ratio18_1, false);
struct Vector3f {
float x, y, z;
};
struct Orientation {
float roll; // Angolo di rollio in gradi
float pitch; // Angolo di beccheggio in gradi
float yaw; // Angolo di imbardata (heading) in gradi
};

#define FILTER_ALPHA 0.98f // Peso del giroscopio nel filtro complementare
#define DT 0.01f // Intervallo di tempo tra le letture (10ms)
Orientation currentOrientation = {0.0f, 0.0f, 0.0f};
Vector3f gyroData = {0.0f, 0.0f, 0.0f};
Vector3f accelData = {0.0f, 0.0f, 0.0f};
bool fusionActive = true;
thread sensorFusionThread;
// Dichiarazione dei motori
motor leftA = motor(PORT11, gearSetting::ratio18_1, false);
motor leftB = motor(PORT1, gearSetting::ratio18_1, false);
motor_group leftMotors = motor_group(leftA, leftB);

motor rightA = motor(PORT10, gearSetting::ratio18_1, true);
motor rightB = motor(PORT12, gearSetting::ratio18_1, true);
motor_group rightMotors = motor_group(rightA, rightB);



inertial Inertial = inertial(PORT7);


smartdrive Smartdrive = smartdrive(leftMotors, rightMotors, Inertial, 100.0, 260.9, 127.3, mm);
const float SogliaErroreHeading = 0.80;
// Costanti per le posizioni
const double PINZA_APERTA = -40;
const double PINZA_CHIUSA = 14;
const double BRACCIO_ALZATO = 35.5;
const double BRACCIO_ABBASSATO = 120.0;
const int VELOCITA_BRACCIO = 20;
const int VELOCITA_PINZA = 20;

// Costanti per il movimento
const double FATTORE_CORREZIONE_AVANTI = 0.3333;
const double FATTORE_CORREZIONE_INDIETRO = 0.3333;
const double FATTORE_CORREZIONE_ROTAZIONE = 1.0;
const int VELOCITA_ROTAZIONE = 30;
const int VELOCITA_AVANZAMENTO = 40;

// Costanti per i sensori
const int POTENZA_LED_NORMALE = 50;
const int POTENZA_LED_BASSA_LUMINOSITA = 100;
const int SOGLIA_BASSA_LUMINOSITA = 80;
const int NUMERO_CAMPIONI_COLORE = 5;
const int RITARDO_LETTURA_COLORE = 200; // millisecondi

// Variabili globali
bool osFront = false;
bool osLat = false;
bool timerScaduto = false;
char coloreRilevato = 'n'; // 'n' = nessuno, 'r' = rosso, 'g' = giallo, 'v' = verde, 'b' = blu
bool threadAttivo = true;
bool giallo = false;
bool verde = false;
int c1,c2,c3;
int distanzaTotale = 0; // Contatore distanza totale avanti
int conta = 1;




// Espansione triport
triport expander = triport(PORT3);
potV2 potBraccio(expander.A);
potV2 potPinza(expander.B);

// Ottiene i dati grezzi dal giroscopio
Vector3f getGyroData(inertial& sensor) {
Vector3f gyro;
gyro.x = sensor.gyroRate(xaxis, dps); // Roll rate
gyro.y = sensor.gyroRate(yaxis, dps); // Pitch rate
gyro.z = sensor.gyroRate(zaxis, dps); // Yaw rate
return gyro;
}

// Ottiene i dati grezzi dall'accelerometro
Vector3f getAccelData(inertial& sensor) {
Vector3f accel;
accel.x = sensor.acceleration(xaxis);
accel.y = sensor.acceleration(yaxis);
accel.z = sensor.acceleration(zaxis);
return accel;
}

// Estrae angoli di rollio e beccheggio dall'accelerometro
void getAnglesFromAccel(Vector3f& accel, float& roll, float& pitch) {
// Calcola gli angoli di rollio e beccheggio usando l'accelerometro
roll = atan2f(accel.y, accel.z) * 57.295779513f; // Conversione radianti->gradi
pitch = atan2f(-accel.x, sqrtf(accel.y * accel.y + accel.z * accel.z)) * 57.295779513f;
}

// Aggiorna l'orientamento del sensore usando un filtro complementare
void updateOrientation() {
// Ottieni angoli dall'accelerometro
float accelRoll, accelPitch;
getAnglesFromAccel(accelData, accelRoll, accelPitch);

// Integra i dati del giroscopio
currentOrientation.roll = FILTER_ALPHA * (currentOrientation.roll + gyroData.x * DT) +
(1.0f - FILTER_ALPHA) * accelRoll;

currentOrientation.pitch = FILTER_ALPHA * (currentOrientation.pitch + gyroData.y * DT) +
(1.0f - FILTER_ALPHA) * accelPitch;

// Aggiorna lo yaw (heading) solo dal giroscopio, ma applicando correzioni periodiche
currentOrientation.yaw += gyroData.z * DT;

// Normalizza l'angolo di yaw nell'intervallo [0, 360]
if (currentOrientation.yaw < 0) currentOrientation.yaw += 360.0f;
if (currentOrientation.yaw >= 360.0f) currentOrientation.yaw -= 360.0f;
}

// Thread principale per la fusione dei sensori
void sensorFusionLoop() {
while (fusionActive) {
// Aggiorna i dati dai sensori
gyroData = getGyroData(Inertial);
accelData = getAccelData(Inertial);

// Aggiorna l'orientamento usando i dati dei sensori
updateOrientation();

// Pausa breve per rispettare DT
this_thread::sleep_for(milliseconds(int(DT * 1000)));
}
}

// Ottieni l'heading corretto dalla fusione dei sensori
float getCorrectedHeading() {
return currentOrientation.yaw;
}

float getCurrentHeading() {
    return Inertial.heading();  // Restituisce l'angolo attuale (in gradi)
}
float getTargetHeading(char direzione) {
    if (direzione == 'f') {
        return 0.0;  // Orientamento desiderato per andare avanti
    } else if (direzione == 'b') {
        return 180.0;  // Orientamento desiderato per andare indietro
    }
    return 0.0;
}

// Funzione per ruotare il robot a destra

/**
 * Thread di controllo per il rilevamento di oggetti frontali
 */
void checkFront() {
    while(threadAttivo) {
        osFront = SensoreOttico2.isNearObject();
        this_thread::sleep_for(milliseconds(50));
    }
}
// Funzione per ruotare il robot a destra
// Funzione per ruotare il robot a destra
// Funzione per ruotare il robot a destra



/**
 * Inizializza il robot e rileva il colore dell'area di partenza
 */

/**
 * Muove il robot nella direzione specificata per la distanza data
 * @param direzione 'f' per avanti, 'b' per indietro
 * @param distanza Distanza in millimetri
 */

void turn(double angolo_target) {
// Ottieni l'angolo corrente (ora usando il sensore con fusione)
double angolo_corrente = getCorrectedHeading();

// Calcola la differenza per determinare la direzione di rotazione
double differenza = angolo_target - angolo_corrente;

// Normalizza la differenza nell'intervallo [-180, 180]
if (differenza > 180) differenza -= 360;
if (differenza < -180) differenza += 360;

// Determina la direzione e applica la compensazione appropriata
double angolo_finale = angolo_target;

// Implementa un controllo PID semplificato per la rotazione
const double Kp = 0.5; // Fattore proporzionale
const double tolleranza = 0.5; // Tolleranza in gradi

// Usa SmartDrive per ruotare inizialmente vicino all'obiettivo
Smartdrive.turnToHeading(angolo_finale, degrees, VELOCITA_ROTAZIONE, rpm);

// Controlla l'errore residuo e correggi con precisione
angolo_corrente = getCorrectedHeading();
differenza = angolo_target - angolo_corrente;
if (differenza > 180) differenza -= 360;
if (differenza < -180) differenza += 360;

// Correzione fine se necessario
while (fabs(differenza) > tolleranza) {
int velocitaCorrezione = int(Kp * differenza);
if (fabs(velocitaCorrezione) < 5) {
velocitaCorrezione = (velocitaCorrezione > 0) ? 5 : -5;
}

// Applica la correzione
if (differenza > 0) {
leftMotors.spin(forward, velocitaCorrezione, percent);
rightMotors.spin(reverse, velocitaCorrezione, percent);
} else {
leftMotors.spin(reverse, velocitaCorrezione, percent);
rightMotors.spin(forward, velocitaCorrezione, percent);
}

// Attendi un po' e ricontrolla
this_thread::sleep_for(milliseconds(20));

// Aggiorna la differenza
angolo_corrente = getCorrectedHeading();
differenza = angolo_target - angolo_corrente;
if (differenza > 180) differenza -= 360;
if (differenza < -180) differenza += 360;
}

// Ferma i motori quando l'orientamento è raggiunto
leftMotors.stop(brake);
rightMotors.stop(brake);

// Breve pausa per assicurarsi che il movimento sia completato
this_thread::sleep_for(10);
}

void move(char direzione, int distanza) {
// Memorizza l'orientamento target
float targetHeading = Smartdrive.heading(degrees);

// Fattore di correzione base
float fattoreCorrezione = (direzione == 'f') ?
FATTORE_CORREZIONE_AVANTI :
FATTORE_CORREZIONE_INDIETRO;
float distanzaCorretta = distanza * fattoreCorrezione;

// Parametri per la correzione di direzione durante il movimento
const float Kp = 0.5; // Fattore proporzionale
const int velBase = VELOCITA_AVANZAMENTO;

if (direzione == 'f') {
// Prima parte: 95% della distanza
float distanzaPrima = distanzaCorretta * 0.95;

// Imposta il movimento ma non aspetta il completamento
Smartdrive.driveFor(forward, distanzaPrima, mm, velBase, rpm, false);

// Loop per la correzione durante il movimento
while (Smartdrive.isMoving()) {
// Ottieni l'orientamento attuale e calcola l'errore
float currentHeading = getCorrectedHeading();
float headingError = targetHeading - currentHeading;

// Normalizza l'errore
if (headingError > 180) headingError -= 360;
if (headingError < -180) headingError += 360;

// Applica una correzione proporzionale
if (fabs(headingError) > SogliaErroreHeading) {
int correzione = int(Kp * headingError);
leftMotors.setVelocity(velBase - correzione, percent);
rightMotors.setVelocity(velBase + correzione, percent);
}

// Breve pausa
this_thread::sleep_for(milliseconds(20));
}

// Breve pausa
this_thread::sleep_for(milliseconds(50));

distanzaTotale += distanza; // Aggiorna contatore distanza
}
else if (direzione == 'b') {
    // Per il movimento all'indietro
    Smartdrive.driveFor(reverse, distanzaCorretta, mm, velBase, rpm, false);

    // Loop per la correzione durante il movimento
    while (Smartdrive.isMoving()) {
    // Ottieni l'orientamento attuale e calcola l'errore
        float currentHeading = getCorrectedHeading();
        float headingError = targetHeading - currentHeading;

        // Normalizza l'errore
        if (headingError > 180) headingError -= 360;
        if (headingError < -180) headingError += 360;

        // Applica una correzione proporzionale (invertita per il reverse)
        if (fabs(headingError) > SogliaErroreHeading) {
            int correzione = int(Kp * headingError);
            leftMotors.setVelocity(velBase - correzione, percent);
            rightMotors.setVelocity(velBase + correzione, percent);
        }

        // Breve pausa
        this_thread::sleep_for(milliseconds(10));
        }
    }
}

/**
 * Controlla il braccio e la pinza
 * @param azione 'u' alza braccio, 'd' abbassa braccio, 'o' apri pinza, 'c' chiudi pinza
 */
void controllaRobotBraccio(char azione) {
    // Controlla se il timer è scaduto
    if (timerScaduto) return;
    
    int velocitaMotore = VELOCITA_BRACCIO;
    double posizione = 0.0;
    motor* motoreTarget = nullptr;
    
    switch(azione) {
        case 'u':
            braccio.spinToPosition(BRACCIO_ALZATO, rotationUnits::deg, (VELOCITA_BRACCIO - 10), velocityUnits::pct);
            // Attendi fino a quando il movimento è completato o il timer scade
            while(braccio.isSpinning() && !timerScaduto) {
                this_thread::sleep_for(milliseconds(10));
            }
            braccio.stop(hold);
            break;
        case 'd':
            braccio.spinToPosition(BRACCIO_ABBASSATO, rotationUnits::deg, VELOCITA_BRACCIO, velocityUnits::pct);
            // Attendi fino a quando il movimento è completato o il timer scade
            while(braccio.isSpinning() && !timerScaduto) {
                this_thread::sleep_for(milliseconds(10));
            }
            braccio.stop(hold);
            break;
        case 'o':
            pinza.spinToPosition(PINZA_APERTA, degrees, (VELOCITA_PINZA + 10), velocityUnits::pct, true);
            // Attendi fino a quando il movimento è completato o il timer scade
            while(pinza.isSpinning() && !timerScaduto) {
                this_thread::sleep_for(milliseconds(10));
            }
            pinza.stop(hold);
            break;
        case 'c':
            pinza.spinToPosition(PINZA_CHIUSA, degrees, VELOCITA_PINZA, velocityUnits::pct, false);
            if (!timerScaduto) {
                this_thread::sleep_for(milliseconds(1000));
            }
            pinza.stop(hold);
            break;
        default:
            break;
    }
    
    if (motoreTarget != nullptr && !timerScaduto) {
        motoreTarget->spinToPosition(posizione, rotationUnits::deg, velocitaMotore, velocityUnits::pct, false);
        // Attendi fino a quando il movimento è completato o il timer scade
        while(motoreTarget->isSpinning() && !timerScaduto) {
            this_thread::sleep_for(milliseconds(10));
        }
        motoreTarget->stop(hold);
    }
}

/**
 * Sequenza per prendere un oggetto
 */
void prendi() {
    controllaRobotBraccio('o'); // Apri pinza
    controllaRobotBraccio('d'); // Abbassa braccio
    controllaRobotBraccio('c'); // Chiudi pinza
    controllaRobotBraccio('u'); // Alza braccio
}

/**
 * Sequenza per lasciare un oggetto
 */
void lascia() {
    controllaRobotBraccio('d'); // Abbassa braccio
    controllaRobotBraccio('o'); // Apri pinza
    controllaRobotBraccio('u'); // Alza braccio
    controllaRobotBraccio('c'); // Chiudi pinza
}

/**
 * Verifica se un colore è affidabile in base alla luminosità
 * @param brightness Valore di luminosità dal sensore (0-100)
 * @return true se il colore è affidabile, false altrimenti
 */
bool coloreAffidabile(double brightness) {
    // Con luminosità molto bassa, i colori sono difficili da rilevare correttamente
    const double SOGLIA_LUMINOSITA_MINIMA = 10.0;
    
    // Se la luminosità è troppo bassa, il colore non è affidabile
    if (brightness < SOGLIA_LUMINOSITA_MINIMA) {
        return false;
    }
    
    return true;
}

/**
 * Riconosce il colore considerando Hue e Luminosità
 * @param hue Valore dell'hue dal sensore (0-359)
 * @param brightness Valore di luminosità dal sensore (0-100)
 * @return Carattere che rappresenta il colore: 'r' rosso, 'g' giallo, 'v' verde, 'b' blu, 'n' non determinato
 */
char riconosciColoreHV(int hue, double brightness) {
    // Se il colore non è affidabile, restituisci 'n' (nessun colore)
    if (!coloreAffidabile(brightness)) {
        return 'n';
    }
    
    // Classifica il colore in base all'hue con range migliorati
    if (hue < 30) { // rosso
        return 'r';
    } else if (hue >= 30 && hue < 70) { // giallo - range ristretto
        return 'g';
    } else if (hue > 72 && hue < 150) { // verde - range specifico
        return 'v';
    } else if (hue >= 200 && hue < 358) { // blu
        return 'b';
    }
    
    // Se l'hue non rientra in nessun range definito
    return 'n';
}

/**
 * Legge il colore con il sensore frontale
 * @return Colore rilevato: 'r' rosso, 'g' giallo, 'v' verde, 'b' blu, 'n' non determinato
 */
char leggiFront() {
    
    /**
     * Versione migliorata della funzione leggiFront che utilizza Hue e Brightness
     * @return Colore rilevato: 'r' rosso, 'g' giallo, 'v' verde, 'b' blu, 'n' non determinato
     */
    SensoreOttico2.setLightPower(POTENZA_LED_NORMALE, percent);
    SensoreOttico2.setLight(ledState::on);

    int conteggiRosso = 0;
    int conteggioGiallo = 0;
    int conteggioVerde = 0;
    int conteggioBlu = 0;
    int conteggioNonDeterminato = 0;

    // Array per memorizzare i valori di ogni campione (per debug)
    int valoriHue[NUMERO_CAMPIONI_COLORE] = {0};
    double valoriBri[NUMERO_CAMPIONI_COLORE] = {0};

    for (int i = 0; i < NUMERO_CAMPIONI_COLORE; i++) {
        if (SensoreOttico2.isNearObject()) {
            int hue = SensoreOttico2.hue(); 
            double brightness = SensoreOttico2.brightness();
            
            // Salva i valori per debug
            valoriHue[i] = hue;
            valoriBri[i] = brightness;
            
            // Aumenta la potenza del LED se la luminosità è bassa
            if (brightness < SOGLIA_BASSA_LUMINOSITA) {
                SensoreOttico2.setLightPower(POTENZA_LED_BASSA_LUMINOSITA, percent);
            }

            // Usa la nuova funzione di riconoscimento colore
            char colore = riconosciColoreHV(hue, brightness);
            
            // Aggiorna i conteggi in base al risultato
            switch(colore) {
                case 'r': conteggiRosso++; break;
                case 'g': conteggioGiallo++; break;
                case 'v': conteggioVerde++; break;
                case 'b': conteggioBlu++; break;
                case 'n': conteggioNonDeterminato++; break;
            }
        }
        // Aggiungi un breve delay per stabilizzare la lettura
        this_thread::sleep_for(milliseconds(50));
        this_thread::sleep_for(milliseconds(RITARDO_LETTURA_COLORE));
    }

    // Visualizza i risultati
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);

    // Visualizza i valori HV dei campioni (debug)
    Brain.Screen.print("HV: ");
    for (int i = 0; i < NUMERO_CAMPIONI_COLORE; i++) {
        if (i < 2) { // Mostra solo i primi 2 per limitare lo spazio
            Brain.Screen.print("H:%d,B:%.1f ", valoriHue[i], valoriBri[i]);
        }
    }
    Brain.Screen.newLine();
    
    // Visualizza i conteggi di rilevamento
    Brain.Screen.print("R:%d G:%d V:%d B:%d N:%d", conteggiRosso, conteggioGiallo, conteggioVerde, conteggioBlu, conteggioNonDeterminato);
    Brain.Screen.newLine();

    char coloreRilevato = 'n'; // n = non determinato
    int maxConteggio = 0;
    
    // Trova il colore con il massimo conteggio
    if (conteggiRosso > maxConteggio) {
        maxConteggio = conteggiRosso;
        coloreRilevato = 'r';
    }
    if (conteggioGiallo > maxConteggio) {
        maxConteggio = conteggioGiallo;
        coloreRilevato = 'g';
    }
    if (conteggioVerde > maxConteggio) {
        maxConteggio = conteggioVerde;
        coloreRilevato = 'v';
    }
    if (conteggioBlu > maxConteggio) {
        maxConteggio = conteggioBlu;
        coloreRilevato = 'b';
    }
    
    // Controlla che ci sia abbastanza "certezza" nel risultato
    if (maxConteggio < NUMERO_CAMPIONI_COLORE / 3) {
        coloreRilevato = 'n'; // Non abbastanza certezza
        Brain.Screen.print("Colore incerto (conteggio basso)");
    } else {
        // Mostra il colore rilevato
        switch(coloreRilevato) {
            case 'r': Brain.Screen.print("Rosso"); break;
            case 'g': Brain.Screen.print("Giallo"); break;
            case 'v': Brain.Screen.print("Verde"); break;
            case 'b': Brain.Screen.print("Blu"); break;
            default: Brain.Screen.print("Non determinato"); break;
        }
    }

    // Spegni il LED dopo l'uso
    SensoreOttico2.setLight(ledState::off);
    return coloreRilevato;
}

int return1(int x){
    int c = x * 110;
    
    return c;
}

/**
 * Funzione che implementa percorsi diversi in base al colore rilevato
 * Gestisce percorsi specifici per rosso e giallo
 */
void colori_verde(){
    for (int i = 0; i < 17; i++)
    {
      
        char colore = leggiFront();
        

        // Visualizza il colore rilevato e implementa percorsi diversi
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Colore rilevato: ");
        int c;
        if (colore == 'r') { // Se colore rosso
            Brain.Screen.print("ROSSO");
            Brain.Screen.newLine();
            Brain.Screen.print("Eseguendo percorso ROSSO");
            
            move('b', 30);
            prendi();
            move('b', 100);
            turn(180);
            move('f', return1(conta));
            turn(270);
            move('f', 75);
            lascia();

            conta=conta+1;
            // Ritorno
            move('b', 60);
            turn(0);

            
            move('f', return1(conta));
            turn(90);
            move('f', 110);

        } 
        else if (colore == 'g') { // Se colore giallo
            Brain.Screen.print("GIALLO");
            Brain.Screen.newLine();
            Brain.Screen.print("Eseguendo percorso GIALLO");
            
            move('b', 30);
            prendi();
            move('b', 100);
            turn(180);
            move('f', return1(conta));
            turn(270);
            move('f', 75);
            lascia();

            conta=conta+1;
            // Ritorno
            move('b', 60);
            turn(0);

            move('f', return1(conta));
            turn(90);
            move('f', 110);

        }
        else if (colore=='v') {
            Brain.Screen.print("VERDE ");
            Brain.Screen.newLine();
            Brain.Screen.print("NON TOCCARE");
            conta=conta+1;
            move('b', 75);
            turn(0);
            move('f', 105);
            turn(90);
            move('f', 65);
        }
        else if (colore=='n'){
            while (true) {
                move('f', 10);
                c=c+1;
                colori_verde();
            }
            if (colore!='n'){
                int d;
                d=c*10;
                move('b', d);
                colori_verde();
                break;
            }   
        }
        else if(colore == 'b'){
            Brain.Screen.print("BLU");
            Brain.Screen.newLine();
            Brain.Screen.print("Officina gialla attiva");
            conta=conta+1;
            move('b', 40);
            prendi();
            move('b', 30);
            turn(180);
            

            move('f', return1(conta));
            move('f', 440);
            turn(90);
            move('f', 680);
            turn(0);
            move('f', 340);
            turn(90);
            move('f',40);
            lascia();
            move('b', 40);
            turn(180);
            //ritorno
            move('f',340);
            turn(270);
            move('f',680);
            turn(0);
            move('f', 440);
            conta=conta+1;
            move('f', return1(conta));
            turn(90);
        }
        else {
            int j=0;
            while (true) {
                turn(90-j);
                j=j+2;
                if(leggiFront()){
                    colori_verde();
                    break;
                }
            }
        }
    }
    
}





void colori_giallo() {
    for (int i = 0; i < 17; i++)
    {
      
        char colore = leggiFront();
        

        // Visualizza il colore rilevato e implementa percorsi diversi
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Colore rilevato: ");
        int c;
        if (colore == 'r') { // Se colore rosso
            Brain.Screen.print("ROSSO");
            Brain.Screen.newLine();
            Brain.Screen.print("Eseguendo percorso ROSSO");
            
            move('b', 30);
            prendi();
            move('b', 100);
            turn(180);
            move('f', return1(conta));
            turn(270);
            move('f', 75);
            lascia();

            conta=conta+1;
            // Ritorno
            move('b', 60);
            turn(0);

            move('f', return1(conta));
            turn(90);
            move('f', 110);

        } 
        else if (colore == 'g') { // Se colore giallo
            Brain.Screen.print("GIALLO");
            Brain.Screen.newLine();
            Brain.Screen.print("Eseguendo percorso GIALLO");
            
            move('b', 30);
            prendi();
            move('b', 100);
            turn(180);
            move('f', return1(conta));
            turn(270);
            move('f', 75);
            lascia();

            conta=conta+1;
            // Ritorno
            move('b', 60);
            turn(0);

            move('f', return1(conta));
            turn(90);
            move('f', 110);

        }
        else if (colore=='v') {
            Brain.Screen.print("VERDE ");
            Brain.Screen.newLine();
            Brain.Screen.print("NON TOCCARE");
            conta=conta+1;
            move('b', 75);
            turn(0);
            move('f', 100);
            turn(90);
            move('f', 65);
        }
        else if (colore=='n'){
            while (true) {
                move('f', 10);
                c=c+1;
                colori_giallo();
            }
            if (colore!='n'){
                int d;
                d=c*10;
                move('b', d);
                colori_giallo();
                break;
            }   
        }
        else if(colore == 'b'){
            Brain.Screen.print("BLU");
            Brain.Screen.newLine();
            Brain.Screen.print("Officina gialla attiva");
            conta +=1;
            move('b', 40);
            prendi();
            move('b', 30);
            turn(180);

            move('f', return1(conta));
            turn(0);
            move('f', 1100);
            turn(270);
            move('f', 100);
            lascia();
            move('b', 100);
            turn(180);
            move('f', 1100);
            turn(0);
            move('f', return1(conta));
            turn(90);
            move('f', 110);
        }
        else {
            int j=0;
            while (true) {
                turn(90-j);
                j=j+2;
                if(leggiFront()){
                    colori_giallo();
                    break;
                }
            }
        }
    }
    
}
void inizio() {
    float distanza = 200; // millimetri
    int velocita = 55;
    int conteggioVerde = 0;
    int conteggioGiallo = 0;

    if (timerScaduto) return;
    
    // Resto del codice della funzione inizio...
    
    // Aggiungi controlli del timer nei cicli while e nei punti critici
    
    // Esempio:
    while(Smartdrive.isMoving() && !timerScaduto) {
        if(osFront) {
            Smartdrive.stop(hold);
            break;
        }
        this_thread::sleep_for(milliseconds(10));
    }
    
    // Controlla il timer prima di continuare
    if (timerScaduto) return;

    // Correzione della distanza
    float distanzaCorretta = distanza * FATTORE_CORREZIONE_AVANTI;
    Smartdrive.driveFor(forward, distanzaCorretta, mm, velocita, rpm);
    distanzaTotale += distanza; // Aggiorna contatore distanza

    // Controlla continuamente se un oggetto è rilevato frontalmente
    while(Smartdrive.isMoving()) {
        if(osFront) {
            Smartdrive.stop(hold);
            break;
        }
        this_thread::sleep_for(milliseconds(10));
    }

    // Configurazione del sensore ottico
    SensoreOttico.setLightPower(POTENZA_LED_NORMALE, percent);
    SensoreOttico.setLight(ledState::on);

    // Rileva il colore con più campioni per aumentare l'affidabilità
    for(int i = 0; i < NUMERO_CAMPIONI_COLORE; i++) {
        if (SensoreOttico.isNearObject()) {
            int hue = SensoreOttico.hue(); 
            double brightness = SensoreOttico.brightness(); 
            int lightPower = POTENZA_LED_NORMALE;

            // Aumenta la potenza del LED se la luminosità è bassa
            if (brightness < SOGLIA_BASSA_LUMINOSITA) {  
                lightPower = POTENZA_LED_BASSA_LUMINOSITA;
                SensoreOttico.setLightPower(lightPower, percent);
            } 

            // Classifica il colore basato sull'hue con range migliorati
            // Range di verde più specifico: 110-150
            if (hue >= 30 && hue < 70) {
                conteggioGiallo++;
                Brain.Screen.setCursor(i+2, 1);
                Brain.Screen.print("Campione %d: Giallo (Hue: %d)", i+1, hue);
            } else if (hue >= 70 && hue < 110) {
                conteggioVerde++;
                Brain.Screen.setCursor(i+2, 1);
                Brain.Screen.print("Campione %d: Verde (Hue: %d)", i+1, hue);
            }
            
            // Aggiungi un breve delay per stabilizzare la lettura
            this_thread::sleep_for(milliseconds(50));
        }
        this_thread::sleep_for(milliseconds(RITARDO_LETTURA_COLORE));
    }
    SensoreOttico.setLight(ledState::off);
    // Determina il colore finale basato sul conteggio
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    
    if(conteggioVerde > conteggioGiallo) {
        Brain.Screen.print("Verde (Conteggio: %d)", conteggioVerde);
        verde = true;
        giallo = false;
        move('f', 350);
                turn(90);
                move('f', 170);
                turn(0);
                move('f', 430);
                turn(90);
                move('f', 100);


                colori_verde();
    } else if(conteggioGiallo > conteggioVerde) {
        giallo = true;
        verde = false;
        Brain.Screen.print("Giallo (Conteggio: %d)", conteggioGiallo);
        move('f', 350);
        turn(90);
        move('f', 170);
        turn(0);

        move('f', 430);
        turn(90);
        move('f', 100);

        colori_giallo();
    } else {
        // In caso di parità, effettua una seconda serie di letture con più potenza
        if (conteggioVerde == conteggioGiallo && conteggioVerde > 0) {
            SensoreOttico.setLightPower(POTENZA_LED_BASSA_LUMINOSITA, percent);
            conteggioVerde = 0;
            conteggioGiallo = 0;
            
            for(int i = 0; i < NUMERO_CAMPIONI_COLORE; i++) {
                if (SensoreOttico.isNearObject()) {
                    int hue = SensoreOttico.hue();
                    
                    // Range più precisi per il secondo tentativo
                    if (hue >= 30 && hue < 70) {
                        conteggioGiallo++;
                    } else if (hue >= 110 && hue < 150) {
                        conteggioVerde++;
                    }
                }
                this_thread::sleep_for(milliseconds(RITARDO_LETTURA_COLORE));
            }
            
            if(conteggioVerde > conteggioGiallo) {
                Brain.Screen.print("Verde (2° lettura: %d)", conteggioVerde);
                verde = true;
                giallo = false;


                move('f', 350);
                turn(90);
                move('f', 170);
                turn(0);
                
                
                move('f', 430);
                turn(90);
                move('f', 100);


                colori_verde();


            } else if(conteggioGiallo > conteggioVerde) {
                Brain.Screen.print("Giallo (2° lettura: %d)", conteggioGiallo);
                giallo = true;
                verde = false;

                move('f', 350);
                turn(90);
                move('f', 170);
                turn(0);

                move('f', 430);
                turn(90);
                move('f', 100);

                colori_giallo();
            } else {
                Brain.Screen.print("Colore non determinato");
            }
        } else {
            Brain.Screen.print("Colore non determinato");
        }
        
    }
    
    // Spegni il LED dopo l'uso
    SensoreOttico.setLight(ledState::off);
}

void timerThread() {
    // Attende 210 secondi
    this_thread::sleep_for(milliseconds(210000));
    
    // Imposta la variabile di controllo
    timerScaduto = true;
    threadAttivo = false;
    
    // Ferma tutti i motori
    leftMotors.stop(brakeType::brake);
    rightMotors.stop(brakeType::brake);
    braccio.stop(brakeType::brake);
    pinza.stop(brakeType::brake);
    
    // Visualizza messaggio di fine tempo
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print("TEMPO SCADUTO - 210 secondi");
    Brain.Screen.newLine();
    Brain.Screen.print("Distanza totale: %d mm", distanzaTotale);
}

/**
 * Programma principale
 */
int main() {
// Reset iniziale dei motori
braccio.resetPosition();
pinza.resetPosition();

// Calibrazione del sensore inerziale
Brain.Screen.clearScreen();
Brain.Screen.setCursor(1,1);
Brain.Screen.print("Calibrazione sensore inerziale...");

Inertial.calibrate();
while(Inertial.isCalibrating()) {
this_thread::sleep_for(milliseconds(30));
}

Brain.Screen.clearScreen();
Brain.Screen.setCursor(1,1);
Brain.Screen.print("Calibrazione completata!");
this_thread::sleep_for(milliseconds(1000));

// Inizializza i valori dell'orientamento
currentOrientation.roll = 0.0f;
currentOrientation.pitch = 0.0f;
currentOrientation.yaw = 0.0f;

// Avvia il thread per la fusione dei sensori
fusionActive = true;
sensorFusionThread = thread(sensorFusionLoop);

// Imposta l'orientamento iniziale
Smartdrive.setHeading(0, degrees);

// Avvia thread per il monitoraggio frontale
threadAttivo = true;
thread check = thread(checkFront);

// Inizializzazione e lettura colore iniziale
inizio();

// Segnala completamento e mostra distanza totale
Brain.Screen.clearScreen();
Brain.Screen.setCursor(1,1);
Brain.Screen.print("Missione completata!");

// Termina i thread
threadAttivo = false;
fusionActive = false;
check.join();
sensorFusionThread.join();

return 0;
}