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
const static TransferDataPoint const temp_lookup_table[TEMPLOOKUPTABLE_SIZE] = {
		{3077, -40, 0},
		{3066, -35, 0},
		{3056, -30, 0},
		{3045, -25, 0},
		{3029, -20, 0},
		{3014, -15, 0},
		{2987, -10, 0},
		{2966, -5, 0},
		{2934, 0, 30},
		{2903, 5, 37},
		{2871, 10, 44},
		{2839, 15, 51},
		{2802, 20, 58},
		{2771, 25, 65},
		{2739, 30, 72},
		{2707, 35, 79},
		{2675, 40, 86},
		{2649, 45, 93},
		{2628, 50, 100},
		{2607, 55, 100},
		{2586, 60, 100},
		{2570, 65, 100},
		{2554, 70, 100},
		{2543, 75, 100},
		{2528, 80, 100},
		{2517, 85, 100},
		{2512, 90, 100},
		{2501, 95, 100},
		{2496, 100, 100},
		{2491, 105, 100},
		{2485, 110, 100},
		{2480, 115, 100},
		{2475, 120, 100}
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
