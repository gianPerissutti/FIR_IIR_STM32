#include "filtros_digitales.h"

void IIR_filter(float *x, float *y,float *a, float *b, unsigned int order)
{
	y[0]= 0;
	  for(unsigned int i = 0; i<(order+1); i++)
	        {
	            y[0] += x[i]*b[i] ;
	        }
	        for(unsigned int i = 1; i<(order+1);i++)
	        {
	            y[0] -= y[i]*a[i];
	        }

	for(unsigned int i = (order+1); i>0; i--)
        {
            y[i] = y[i-1];
            x[i] = x[i-1];
        }


    
}
void FIR_filter(float *x, float *y,float *b, unsigned int order)
{
	y[0]= 0;
	 for(unsigned int i = 0; i<(order+1); i++)
	        {
	            y[0] += x[i]*b[i] ;
	        }

      for(unsigned int i = (order+1); i>0; i--)
        {
            x[i] = x[i-1];
        }


}
