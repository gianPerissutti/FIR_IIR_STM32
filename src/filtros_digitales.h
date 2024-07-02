#ifndef FILTROS_DIGITALES
#define FILTROS_DIGITALES


void IIR_filter(float *x, float *y,float *a, float *b, unsigned int order);

void FIR_filter(float *x, float *y,float *b, unsigned int order);



#endif  // FILTROS_DIGITALES_H
