#ifndef STRING_UTILS_H
#define STRING_UTILS_H

const char DELIMITER = ';';

/**
 * Check if the provided data contains the delimiter ";"
 * @param data The provided data to check
 * @param length The length of the data
 * @return true if it contains the delimtiter, otherwise false
*/
inline bool contains_delimiter(char* data, size_t length)
{
	for (int i = 0; i < length; i++)
	{
		if (data[i] == DELIMITER)
		{
			return true;
		}
	}

	return false;
}

/**
 * Check at which pos the delimiter can be found
 * There has to be a delimiter, this has to be checkd beforehand!
 * @param data The data to check
 * @return the index where the delimiter is at in the provided data
*/
inline size_t delimiter_at_index(char* data)
{
	size_t pos = 0;
	while (data[pos] != DELIMITER)
	{
		//delimiter not reached increase pos
		pos++;
	}

	return pos;
}

/**
 * Check if a provided char is the delimiter
 * @param data The char data to check
 * @return true if its the delimiter, otherwise false
*/
inline bool is_delimiter(char data)
{
	return (data == DELIMITER);
}

inline void fill_buffer_with_null_terminator(char* buffer, size_t length)
{
	for (int i = 0; i < length; i++)
	{
		buffer[i] = '\0';
	}
}

String bool_to_string(const bool boolean)
{
	return boolean ? String("true") : String("false");
}

String ullToString(unsigned long long value) {
	char buffer[21]; // Max digits for unsigned long long is 20 + 1 for '\0'
	int index = 20;  // Start from the end of the buffer
	buffer[index--] = '\0'; // Null-terminate the string

	// Handle zero explicitly
	if (value == 0) {
		buffer[index--] = '0';
	}
	else {
		// Convert each digit to a character
		while (value > 0 && index >= 0) {
			buffer[index--] = '0' + (value % 10);
			value /= 10;
		}
	}

	// Return the string starting from the first non-empty character
	return String(&buffer[index + 1]);
}

#endif