#ifndef PAN_TOMPKINS_H
#define PAN_TOMPKINS_H

#include <cstddef>

// Initialize array to zero
void init_zero(int* source, std::size_t size)
{
    for (std::size_t i = 0; i < size; i++)
    {
        source[i] = 0;
    }
}

// Low-pass filter
void low_pass_filter(short* source, int* dest, std::size_t size)
{
    // Initialize dest array
    init_zero(dest, size);

    for (std::size_t n = 0; n < size; n++)
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

// High-pass filter
void high_pass_filter(int* source, int* dest, std::size_t size)
{
    // Initialize dest array
    init_zero(dest, size);

    for (std::size_t n = 0; n < size; n++)
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

// Differentiation
void differentiate(int* source, int* dest, std::size_t size)
{
    // Initialize dest array
    init_zero(dest, size);

    for (std::size_t n = 0; n < size; n++)
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

// Squaring (can be done in-place)
void square(int* source, int* dest, std::size_t size)
{
    for (std::size_t n = 0; n < size; n++)
    {
        dest[n] = source[n] * source[n];
    }
}

// Moving window integration
void moving_window_integration(int* source, int* dest, int window_size, std::size_t size)
{
    // Initialize dest array
    init_zero(dest, size);

    int sum = 0;

    for (std::size_t n = 0; n < size; n++)
    {
        if (n < static_cast<std::size_t>(window_size))
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

// Find peaks and return peak count
int find_peaks(int* signal, std::size_t size, int fs)
{
    // Calculate threshold
    long long sum = 0;
    for (std::size_t n = 0; n < size; n++)
    {
        sum += signal[n];
    }
    int threshold = static_cast<int>(sum / size);

    int peak_count = 0;
    int rr_interval = static_cast<int>(0.2 * fs); // Minimum distance of 200 ms between peaks
    int last_peak = -rr_interval;

    for (std::size_t n = 1; n < size - 1; n++)
    {
        if (signal[n] > threshold && signal[n] > signal[n - 1] && signal[n] > signal[n + 1])
        {
            if (static_cast<int>(n) - last_peak >= rr_interval)
            {
                peak_count++;
                last_peak = static_cast<int>(n);
            }
        }
    }

    return peak_count;
}

// Count peaks function
int count_peaks(short* source, std::size_t size)
{
    // Reuse buffers to minimize memory usage
    int* buffer1 = new int[size];
    int* buffer2 = new int[size];

    // Step 1: Low-pass filter
    low_pass_filter(source, buffer1, size);

    // Step 2: High-pass filter
    high_pass_filter(buffer1, buffer2, size);

    // Step 3: Differentiation
    differentiate(buffer2, buffer1, size);

    // Step 4: Squaring
    square(buffer1, buffer2, size);

    // Step 5: Moving window integration
    //125 hz
    int window_size = static_cast<int>(0.150 * 125);
    moving_window_integration(buffer2, buffer1, window_size, size);

    // Step 6: Find peaks
    int peak_count = find_peaks(buffer1, size, 125);

    // Clean up dynamic memory
    delete[] buffer1;
    delete[] buffer2;

    return peak_count;
}

#endif // PAN_TOMPKINS_H