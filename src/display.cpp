// void displayKMeasResults(long *time_us, double meanVal, double sd){
//     // Format and display results of k measurements on ssd1306
//         display.clearDisplay();
//         display.setCursor(0, 0);
//         display.print("/\\t: ");
//         display.setCursor(60, 0);
//         display.print(time_us[1]);
//         display.setCursor(0, 8);
//         display.print("meas: ");
//         display.print(meanVal);
//         display.setCursor(0, 16);
//         display.print("sd: ");
//         display.print(sd);
//         display.display();
// }

// void debugDisplay(auto i){
//     display.clearDisplay();
//     display.setCursor(0, 0);
//     display.print(i);
//     display.display();
// }

// void readAndDisplayKValues(int k, long interval){
//     static long readLast = 0;
//     long timeSinceLastRead = micros() - readLast;

//     if (timeSinceLastRead > interval){
        
//         Point pnt;

//         getKMeasurements(k, &pnt);
        
//         double meanVal = getMeanMeasVal(k, &pnt);

//         double sd = getStdDev(k, &pnt, meanVal);

//         double v = meanVal * (12.0 / 65356.0);
//         displayKMeasResults(pnt.time_us, v, sd);
//         // debugDisplay(pnt.aq[0]);
//         readLast = micros();

//     }
// }

// double toVoltage(double val){
//     return val / 65536.0 * 10;
// }

// void getKMeasurements(int k, Point* pnt){
//     pnt->time_us[0] = micros();
    
//     for (int i = 0; i < k; i++){
//         pnt->aq[i] = adc.read();
//     }

//     pnt->time_us[1] = micros() - pnt->time_us[0];
// }

// double getMeanMeasVal(int k, Point* pnt){
//     // mean Voltage
//     double sum{0};

//     for (int i = 0; i < k; i++){
//         sum += pnt->aq[i];
//     }
    
//     return sum / static_cast<double>(k);
// }

// double getStdDev(int k, Point* pnt, double meanVal){
//     // calculate std dev from 
//     double ss = 0;
//     for (int i = 0; i < k; i++){
//         ss += sq(pnt->aq[i] - meanVal);
//     }
//     return sqrt(ss / k);
// }


// void oledPrint(char* str){
//     // visual feedback
//     display.clearDisplay();
//     display.setCursor(0, 15);
//     display.print(str);
//     display.display();
// }
