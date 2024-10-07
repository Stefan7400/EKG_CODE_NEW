#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <map>

//OPCodes to identify Packets
enum class OPCodes {

	NO_OP,
	LIST_EKG,
	START_24H_EKG,
	ABORT_24_H_EKG,
	ERROR_RESPONSE,
	SUCCESS_RESPONSE,
	DELETE_EKG_FILE,
	FETCH_MODE,

};


#endif