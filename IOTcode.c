#include "contiki.h"
#include "dev/light-sensor.h"
#include "dev/sht11-sensor.h"
#include <stdio.h> /* For printf() */

int getTemperature(void)
{
  int tempData;
  tempData = sht11_sensor.value(SHT11_SENSOR_TEMP_SKYSIM); // For Cooja Sim
  int temp = ((tempData*0.04)-39.6);//transfer function for temperatire of sht11
  return temp;
}

int getLight(void)
{
  float vSensor = 1.5*light_sensor.value(LIGHT_SENSOR_PHOTOSYNTHETIC)/4096;
  float I = vSensor/100000;//ohms law
  float lightLux = 0.625*1e6*I*1000;//transfer function of raw light data
  return lightLux;
}
//find the best guess of the square root of a 'number'
static int getGuessNumber(long number)
{
  static char strNum[10];
  ltoa(number,strNum,10);
  int digit = 0;

  int i = 0;
  while(strNum[i] != '\0')
  {
    if(strNum[i] >= '0' && strNum[i] <= '9')//count the number of digits of number
    {
       ++digit;
    }
   ++i;
  }
  //if the digits are even half it if odd round up and half
  //best guess normally is half the digits
  int d = 0;
  if(digit%2 == 0)
  {
    d = digit/2;
  } 
  else
  {
    d = (digit + 1)/2;
  }
  int multipler = 0;
  if(d == 1)
  { 
   multipler = 1;
  }
  else if(d == 2)
  { 
    multipler = 10;
  }
  else if(d == 3)
  { 
    multipler = 100;
  }
  else 
  { 
    multipler = 1000;
  }
  //best guess usually starts with a 2, so 2 and digits found
  int guess = 2 * multipler;
  return guess;
}

static float getSqrt(long number, int guessNum)
{ 
  float betterGuess = 0;
  float errorInGuess = 0;
  float squaredGuessNum = guessNum*guessNum;

  //improve guess by finding the error to the variance and add together
  errorInGuess = (number - squaredGuessNum)/(2*guessNum);
  betterGuess = guessNum + errorInGuess;
  
  float numToSqrt = number;
  float approxSqrt = betterGuess;
  float prevApproxSqrt = 0;
  //use babylonian method to find the root 
  while((approxSqrt-prevApproxSqrt >= 0.1 && prevApproxSqrt-approxSqrt <= -0.1) || 
        (prevApproxSqrt-approxSqrt >= 0.1 && approxSqrt-prevApproxSqrt <= -0.1))  
  {
     approxSqrt = (approxSqrt + (numToSqrt/approxSqrt))/2;
     prevApproxSqrt = (approxSqrt + (numToSqrt/approxSqrt))/2;
  }

    return approxSqrt;
}

void printFloat(float floatToPrint)
{ 
  char arrayInt[15];
  int arraySize = 0;
  double decimalToInt = (double)(floatToPrint * 100);//push decimal point to the right
  long truncateFloat = (long)decimalToInt;//truncate the decimal
  ltoa(truncateFloat, arrayInt, 10);//convert to char array
  int j = 0;
  int i = 0;
  while(arrayInt[i] != '\0')//find the size of the array
  {
    ++arraySize;
    ++i;  
  }
  for(j = 0; j < (i - 2); ++j) 
  {
    printf("%c", arrayInt[j]);//print all ints up to the last
  }
  printf("%c", '.');//print the decimal
  printf("%c", arrayInt[i-2]);//print last int after decimal
  printf("%c", arrayInt[i-1]);

}
float findMean(int array[], int start, int end,double divider)
{
 int i = 0;
 double sum = 0;
 double mean = 0;
 
 for(i = start; i < end; ++i)//define start and end of index
   {
     sum = sum + array[i];//sum all variables in sum
   }
  
  mean = sum/divider; //find the mean
  
  return mean;
}
static float getStdDev(int array[], int arraySize)
{ 
  float stdDev;
  long sum = 0;
  long sumSquare = 0;
  int i = 0;
  int j = 0;
  long arrayMean = 0;
  long variance = 0;

  for(i = 0; i < arraySize; ++i)
  {
    sum = sum + array[i];//find sum of array
  }

  arrayMean = sum / arraySize;//find mean
  for(j = 0; j < arraySize; ++j)
  {
    //find each points distance from mean squared
    sumSquare = sumSquare + ((array[j] - arrayMean) * (array[j] - arrayMean));
  }

  long guessNumber = 0;
 
  variance = sumSquare / arraySize;
  if(variance == 0)//if variance is 0 so is standard deviation
  { 
    return 0;
  }
  else
  { 
    guessNumber = getGuessNumber(variance);
    stdDev = getSqrt(variance, guessNumber);//find square root of variance
    return stdDev;
  }
}
void getEMA(int array[], float smoothNumber)
{
  int i = 0;
  int j = 0;
  float prevEMA = 0;
  float EMA = (float)array[0]; 
  printf("EMA = [");
  printf("%d\, ", array[0]);
  for(i = 1; i < 12; ++i)
  {
    EMA = (smoothNumber * (float)array[i]) + ((1 - smoothNumber) * EMA);
    prevEMA = (smoothNumber * (float)array[i+1]) + ((1 - smoothNumber) * EMA);

    if(i != 11)
    {
      printFloat(EMA);
      printf(", ");
    }
    else
    {
      printFloat(EMA);
      printf("]\n\n");
    }
  }
  
}
/*---------------------------------------------------------------------------*/
PROCESS(sensor_reading_process, "Sensor reading process");
AUTOSTART_PROCESSES(&sensor_reading_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(sensor_reading_process, ev, data)
{
  
  static struct etimer timer;
  PROCESS_BEGIN();
  
  etimer_set(&timer, CLOCK_CONF_SECOND/2);//2 readings per second 

  SENSORS_ACTIVATE(light_sensor);
  SENSORS_ACTIVATE(sht11_sensor);
  const static int READINGS_ARRAY_SIZE = 12;
  static int readingsArray [12];
  static int counter = 0;
  static int i = 0;
  int j = 0;
  
  while(1)
  {
    PROCESS_WAIT_EVENT_UNTIL(ev=PROCESS_EVENT_TIMER);

    int lightLux = getLight();
    printf("reading: %d\n",lightLux);//get and print reading every 2 seconds

    etimer_reset(&timer);

    readingsArray[i] = lightLux;//enter 12 light readings into an array
    ++i;

    if (i == READINGS_ARRAY_SIZE)//if end of array
    {
      i = 0;
      printf("\nB: [");//print start of buffer
      for(j = 0; j < READINGS_ARRAY_SIZE; ++j)//loop through array
      { 
        if(j != READINGS_ARRAY_SIZE - 1)
        {
          printf("\%d\, ", readingsArray[j]);//if not at last index print
        }
        else
        {
          printf("\%d\]", readingsArray[j]);//at last index frint index and closing bracket
          printf("\n");
        }
      }
      
      static float stdDev = 0;
      PROCESS_WAIT_EVENT_UNTIL(ev=PROCESS_EVENT_TIMER);
      stdDev = getStdDev(readingsArray, READINGS_ARRAY_SIZE);//get standard deviation
      etimer_reset(&timer);

      float smoothNum = 0.7;
      printf("\nSmoothing factor = ");
      printf("%d", 0);
      printFloat(smoothNum);
      printf("\n");
      getEMA(readingsArray, smoothNum);
      printf("StdDev = ");

      if(stdDev != 0)//if not zero print float
      {
        printFloat(stdDev);
      }
      else
      {
        printf("0 \n");//else print 0
      }
      
      if(stdDev <= 70 && stdDev != 0)//threshold minimum at 49 and below
      { 
        float X[1]; //create out buffer
        float mean = 0;
        int start = 0;
        int end = 12;
        double divider = 12;
        printf("\nAggregation = 12-into-1\n");
        mean = findMean(readingsArray, start,end,divider);//find mean of all array values and add to X
        X[0] = mean;
        printf("X[");
        printFloat(X[0]);
        printf("]\n\n");
        
      }
      else if(stdDev >= 70 && stdDev <= 199)//middle threshold
      {
        float mean = 0;
        printf("\nAggregation = 4-into-1\n");
        float X[3];//create out buffer 12 divided by 4
        int tempX[4];//create a temporary buffer
        int start = 0;
        int end = 0;
        double divider = 0;
        
        
        start = 0;//declare find mean input values
	end = 4;//take first 4 values and find mean
        divider = 4;
	X[0] = findMean(readingsArray, start, end, divider);//add to first index of out buffer
  
	
	start = 4;//repeat above with the next four
	end = 8;
        divider = 4;
	X[1] = findMean(readingsArray, start, end, divider);

	
	start = 8;
	end = 12;
        divider = 4;
	X[2] = findMean(readingsArray, start, end, divider);
	
	printf("X[");//print layout and floats of the out buffer
        printFloat(X[0]);
	printf(", ");
	printFloat(X[1]);
	printf(", ");
	printFloat(X[2]);
        printf("]\n\n");
        
      }
      else if(stdDev >= 200)//max threshold
      {
        int i = 0;
        printf("\nNo Aggregation\n");
        printf("X[");
        for(j = 0; j < READINGS_ARRAY_SIZE; ++j)//loop through array
        { 
        if(j != READINGS_ARRAY_SIZE - 1)
        {
          printf("\%d\, ", readingsArray[j]);//if not at last index print
        }
        else
        {
          printf("\%d\]", readingsArray[j]);//at last index frint index and closing bracket
          printf("\n\n");
        }

       }

      }

    }
      
  }
  
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

