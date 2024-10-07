#ifndef DOUBLEDRINGBUFFER_H
#define DOUBLEDRINGBUFFER_H

#include <stdio.h>

//Size of one buffer, total doubled ring buffer size is SINGLE_BUFFER_SIZE * 2
const int SINGLE_BUFFER_SIZE = 1000;
const int DOUBLED_BUFFER_SIZE = SINGLE_BUFFER_SIZE * 2;

/**
* The usage for this buffer is to store a lot of samples together in order to limit the access to the sd-card as 
* writing to the sd-card takes quite some time..
* 
* The buffer is divided into two sections of equal size
* 
* While one section is filled with data the other on can be read and stored to the sdcard
*/
template <typename T>
class DoubledRingBuffer {

private:
    bool readable;
    T doubleBuffer[DOUBLED_BUFFER_SIZE];
    /*
     * The read_index is either 0 or (SINGLE_BUFFER_SIZE / 2)
     */
    int read_index;
    int write_index;

public:

    /**
     * Resets the buffer (this includes its read and write pointer)
     */
    inline void reset() {
        this->readable = false;
        this->read_index = 0;
        this->write_index = 0;
    }

    /**
     * Writes the provided value into the buffer
     * @param value The provided value
     */
    inline void write(T value) {
        this->doubleBuffer[write_index] = value;
        this->write_index = (write_index + 1) % DOUBLED_BUFFER_SIZE;
        //Is readable if value the next write index is 0 or (SINGLE_BUFFER_SIZE)
        if (!this->readable) {
            this->readable = (this->write_index == 0) || (this->write_index == SINGLE_BUFFER_SIZE);
        }
    }

    /**
     * Checks if a the buffer can be read
     * The ringbuffer can be read when one of the two buffers is completely full
     * @return true if the buffer can be read, otherwise false
     */
    inline bool isReadable() {
        return this->readable;
    }

    /*
     * @return returns a pointer to the current read index
     */
    T* readIndex() {
        return &this->doubleBuffer[read_index];
    }

    inline void readDone() {
        if (this->read_index == 0) {
            this->read_index = SINGLE_BUFFER_SIZE;
        }
        else {
            this->read_index = 0;
        }

        this->readable = false;
    }

};

#endif