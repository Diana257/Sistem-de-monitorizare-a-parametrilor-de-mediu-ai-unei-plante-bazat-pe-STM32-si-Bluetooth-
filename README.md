# Sistem de Monitorizare și Control al Îngrijirii unei Plante de Interior Bazat pe FreeRTOS și Comunicație Bluetooth

Acest proiect implementează un sistem inteligent de monitorizare și irigare automată a plantelor de interior, folosind:
 - microcontroller STM32 Nucleo-G071RB
 - sistem de operare în timp real FreeRTOS
 - comunicație wireless prin Bluetooth HC-05
 - senzori de umiditate sol, nivel apă, temperatură și umiditate aer

Utilizatorul poate controla pompa și poate vizualiza datele în timp real prin aplicația Bluetooth Serial Terminal disponibilă în Magazin Play.



## Funcționalități principale 

| Categoria            | Descriere                                                                 |
|----------------------|---------------------------------------------------------------------------|
| **Monitorizare**  | Temperatură + umiditate aer (DHT11)                                       |
|                      | Umiditate sol – ADC, pin **PA0**                                       |
|                      | Nivel apă în rezervor – ADC, pin **PA1**                               |
| **Control automat** | Udare automată pe baza pragurilor configurate                            |
|                      | Protecție la mers în gol (pompa NU pornește dacă nu e apă)               |
| **Semnalizare LED** | **Roșu:** lipsă apă                                                       |
|                      | **Galben:** sol uscat                                                     |
|                      | **Verde:** sol umed (clipire = funcționare normală)                      |
| **Bluetooth**      | Control prin aplicația *Bluetooth Serial Terminal*                        |
|                      | Comenzi: `ON`, `OFF`, `STATUS`, `AUTO=ON`, `AUTO=OFF`                     |



## Comenzi Bluetooth

| Comandă    | Funcție                                         |
|------------|--------------------------------------------------|
| `ON`       | Pornește pompa timp de 3 secunde                 |
| `OFF`      | Oprește pompa imediat                            |
| `STATUS`   | Afișează toți parametrii sistemului              |
| `AUTO=ON`  | Activează udarea automată                        |
| `AUTO=OFF` | Dezactivează udarea automată                     |


## Semnalizări LED

| LED     | Culoare | Semnificație            |
|---------|---------|--------------------------|
| PB5     | Roșu | Lipsă apă                |
| PA10    | Galben | Sol uscat               |
| PA8     | Verde | Sol umed / clipire = ok |



## Senzori, actuatori și pinii aferenți

| Componentă                | Pin STM32 | Tip interfață      | Funcție / Parametru |
|---------------------------|-----------|---------------------|----------------------|
| **DHT11**                 | PA7       | Digital (GPIO)      | Temperatură + umiditate aer |
| **Senzor umiditate sol**  | PA0       | ADC (analogic)      | Umiditate sol |
| **Senzor nivel apă**      | PA1       | ADC (analogic)      | Prezență/absență apă |
| **LED roșu**              | PB5       | GPIO Output         | Lipsă apă |
| **LED galben**            | PA10      | GPIO Output         | Sol uscat |
| **LED verde**             | PA8       | GPIO Output         | Sol umed / funcționare normală |
| **Driver pompă TA6586**  | PB4       | GPIO Output         | Control pompă apă |
| **Modul Bluetooth HC-05** | PC4 / PC5 | UART1 (TX/RX)       | Comunicație cu telefonul |
| **Interfață USB UART**    | PA2 / PA3 | UART2 (TX/RX)       | Debug prin USB virtual COM |





## Montaj hardware

Sistemul este construit pe placa Nucleo-G071RB și include:

- Senzor DHT11 conectat la PA7
- Senzor sol -> ADC channel 0 (PA0)
- Senzor nivel apă -> ADC channel 1 (PA1)
- Pompa controlată prin driver TA6586 → PB4
- LED-uri status:
  - PB5 – lipsă apă (roșu)
  - PA10 – sol uscat (galben)
  - PA8 – sol umed (verde)

Imaginea de mai jos prezintă schema bloc a sistemului:

<img width="2644" height="1964" alt="schemaHW" src="https://github.com/user-attachments/assets/9c580336-60be-49c2-8890-2251968199c4" />
