#ifndef TIMER_H
#define TIMER_H
extern volatile unsigned long millisCounter;   // compteur global en ms

extern unsigned long timestamp;
extern unsigned long tstop;

void InitTimer23(void);
void InitTimer1(void);
void InitTimer4(void);
void SetFreqTimer1(float Freq);
void SetFreqTimer4(float Freq);
#endif /* TIMER_H */