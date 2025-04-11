#ifndef OXIMETER_SENSOR_H
#define OXIMETER_SENSOR_H

void setupOximeter();
void updateDisplayStart();
void updateDisplayNoFinger();
void updateDisplayData(int bpm, int spo2);
bool detectFinger();
float readSpO2();
float readHeartRate();
void updateOximeterDisplay();
void getOximeterData(int* outBpm, int* outSpO2);

#endif