/*
 * temp_lookup_table.h
 *
 *  Sam Parent, 8 April 2022
 */

#ifndef INC_TEMP_LOOKUP_TABLE_H_
#define INC_TEMP_LOOKUP_TABLE_H_

#include "main.h"

#define TEMPLOOKUPTABLE_SIZE 33
#define NUM_ADC_CHANNELS 6
#define ADC_ROLLING_AVG_WIN_SIZE 20

//function prototypes
void getRollingAvg(uint16_t* data_points, uint16_t* averages);
int8_t getTemp(uint16_t ADC_value);
uint8_t getDutyCycle(int8_t temp);
void getTempVals(uint16_t* data, int8_t* output_temp, uint8_t num_channels);
void linearInterpolation(int8_t y1, int8_t y2, uint16_t x1, uint16_t x2, double* m, double* b);

//Lookup table datapoint
typedef struct{
	uint16_t ADC_value;
	int8_t temp;
	uint8_t duty_cycle;
}TransferDataPoint;

//lookup table

// This is changed from the archived repo on GitHub. Talk to Blake F. or Josh D.
// The conversion is the inverted composition of 3 steps:
// 1. Temperature to sensor voltage (from the cell data sheet)
// 2. Sensor voltage to STM voltage (V_STM = 1.44 + V_SENS/2 due do buffer output)
// 3. STM voltage to ADC value (ADC = V_STM / 3.3 * 4095)
const static TransferDataPoint const temp_lookup_table[TEMPLOOKUPTABLE_SIZE] = {
    {3052, -40, 0},
    {3042, -35, 0},
    {3031, -30, 0},
    {3021, -25, 0},
    {3005, -20, 0},
    {2990, -15, 0},
    {2964, -10, 0},
    {2943, -5, 0},
    {2912, 0, 30},
    {2881, 5, 37},
    {2850, 10, 44},
    {2819, 15, 51},
    {2782, 20, 58},
    {2751, 25, 65},
    {2720, 30, 72},
    {2689, 35, 79},
    {2658, 40, 86},
    {2632, 45, 93},
    {2611, 50, 100},
    {2590, 55, 100},
    {2570, 60, 100},
    {2554, 65, 100},
    {2539, 70, 100},
    {2528, 75, 100},
    {2513, 80, 100},
    {2502, 85, 100},
    {2497, 90, 100},
    {2487, 95, 100},
    {2481, 100, 100},
    {2476, 105, 100},
    {2471, 110, 100},
    {2466, 115, 100},
    {2461, 120, 100},
};


void getRollingAvg(uint16_t* data_points, uint16_t* output_averages)
{
    static uint32_t sums [NUM_ADC_CHANNELS] = {};
    static uint16_t num_vals = 0;
    static uint16_t value_windows[NUM_ADC_CHANNELS][ADC_ROLLING_AVG_WIN_SIZE] = {};

    if (num_vals == ADC_ROLLING_AVG_WIN_SIZE)
    {
        for (int i = 0; i<NUM_ADC_CHANNELS; i++)
        {
            sums[i] = sums[i] - value_windows[i][0] + data_points[i];
            output_averages[i] = sums[i]/num_vals;

            //Shift the values
            for (int j = 0; j < ADC_ROLLING_AVG_WIN_SIZE - 1; j++)
            {
                value_windows[i][j] = value_windows[i][j+1];
            }
            value_windows[i][ADC_ROLLING_AVG_WIN_SIZE - 1] = data_points[i];
        }
    }
    else
    {
        num_vals++;
        for (int i = 0; i < NUM_ADC_CHANNELS; i++)
        {
            sums[i] += data_points[i];
            value_windows[i][num_vals-1] = data_points[i];
            output_averages[i] = sums[i]/(num_vals);
        }
    }
}

void getTempVals(uint16_t* data, int8_t* output_temp, uint8_t num_channels)
{
	for(uint8_t i = 0; i<num_channels; i++)
	{
		output_temp[i] = getTemp(data[i]);
	}
}

uint8_t getDutyCycle(int8_t temp)
{
	uint8_t i = 0;
	while((i<TEMPLOOKUPTABLE_SIZE) && (temp > temp_lookup_table[i].temp))
	{
		i++;
	}
	if (i == TEMPLOOKUPTABLE_SIZE)
	{
		return 100;
	}
	else if (i == 0)
	{
		return 0;
	}
	return temp_lookup_table[i-1].duty_cycle;
}

int8_t getTemp(uint16_t ADC_val)
{
	uint8_t i = 0;
	while((i<TEMPLOOKUPTABLE_SIZE) && (ADC_val < temp_lookup_table[i].ADC_value))
	{
		i++;
	}
	if (i == TEMPLOOKUPTABLE_SIZE)
	{
		return temp_lookup_table[TEMPLOOKUPTABLE_SIZE-1].temp;
	}
	else if (i == 0)
	{
		return temp_lookup_table[0].temp;
	}
	double slope;
	double y_intercept;
	linearInterpolation(temp_lookup_table[i].temp, temp_lookup_table[i-1].temp, temp_lookup_table[i].ADC_value, temp_lookup_table[i-1].ADC_value, &slope, &y_intercept);

	return (int8_t)(ADC_val*slope + y_intercept);
}

void linearInterpolation(int8_t y1, int8_t y2, uint16_t x1, uint16_t x2, double* m, double* b)
{
	*m = (double)(( y2 - y1 ) / ((double)( x2 - x1 )));
	*b = (double)( y2 - (*m) * x2 );
}

#endif /* INC_TEMP_LOOKUP_TABLE_H_ */
