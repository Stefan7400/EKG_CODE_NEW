#ifndef RESULT_H
#define RESULT_H

#include <string>

// Class which represents the result of a operation
class Result {
public:
    // Factory method for Ok result
    static Result Ok(const std::string& message = "") {
        return Result(true, message);
    }

    // Factory method for Error result
    static Result Error(const std::string& error_message) {
        return Result(false, error_message);
    }

    // Check if the result is Ok
    bool is_ok() const { return is_ok_; }

    // Check if the result is Error
    bool is_error() const { return !is_ok_; }

    // Get the message as uint8_t*
    const uint8_t* message() const {
        return reinterpret_cast<const uint8_t*>(message_.data());
    }

    // Optionally, provide a way to get the length of the message
    size_t message_length() const {
        return message_.size();
    }

private:
    
    Result(bool is_ok, const std::string& message)
        : is_ok_(is_ok), message_(message) {}

    bool is_ok_;
    std::string message_;
};

#endif // RESULT_H
