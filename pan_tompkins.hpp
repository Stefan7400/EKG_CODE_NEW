#ifndef PAN_TOMPKINS_H
#define PAN_TOMPKINS_H

int count_peaks(short* source, std::size_t size)
{
	int low_pass_result_buffer[size];
	low_pass_filter(source, low_pass_result_buffer, size);
	int high_pass_result_buffer[size];
	high_high_pass_filter(low_pass_result_buffer, high_pass_result_buffer,size);
	int differentiate_result_buffer[size];
	differentiate(high_pass_result_buffer, differentiate_result_buffer, size);
}

void low_pass_filter(short *source, int*dest, std::size_t size)
{
	init_zero(dest, size);

	for (int n = 0; n < size; n++)
	{
		if (n >= 12)
		{
			dest[n] = (2 * dest[n - 1] - dest[n - 2] + source[n] - 2 * source[n - 6] + source[n - 12]);
		}
		else
		{
			dest[n] = 0;
		}
	}
}

void high_high_pass_filter(int* source, int* dest, std::size_t size)
{
	init_zero(dest, size);

	for (int n = 0; n < size; n++)
	{
		if (n >= 32)
		{
			dest[n] = dest[n - 1] - (source[n] / 32) + source[n - 16] - source[n - 17] + (source[n - 32] / 32);
		}
		else
		{
			dest[n] = 0;
		}
	}
}

void differentiate(int* source, int* dest, std::size_t size)
{
	init_zero(dest, size);

	for (int n = 0; n < size; n++)
	{
		if (n >= 4)
		{
			dest[n] = (2 * source[n] + source[n - 1] - source[n - 3] - 2 * source[n - 4]) / 8;
		}
		else
		{
			dest[n] = 0;
		}
	}
}

void square(int* source, int* dest, std::size_t size)
{
	init_zero(dest, size); 

	for (int n = 0; n < n; n++)
	{
		dest[n] = source[n] * source[n];
	}
}

void moving_window_integration(int* source, int* dest, int window_size, std::size_t size)
{
	init_zero(dest, size);
	int sum = 0;

	for (int n = 0; n < size; n++)
	{
		if (n < window_size)
		{
			sum += source[n];
			dest[n] = sum / (n + 1);
		}
		else
		{
			sum += source[n] - source[n - window_size];
			dest[n] = sum / window_size;
		}
	}
}

void init_zero(int* source, std::size_t size)
{
	for (int i = 0; i < size; i++)
	{
		source[i] = 0;
	}
}


#endif 